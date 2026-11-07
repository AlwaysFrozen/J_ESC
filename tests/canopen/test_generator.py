"""CSV schema, deterministic generation and user-file ownership regression tests."""
import csv
import importlib.util
from pathlib import Path
import re
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("generate_od", ROOT / "tools/generate_od.py")
generator = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = generator
spec.loader.exec_module(generator)


class GeneratorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=ROOT / "build/od-tests")
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.csv = self.directory / "od.csv"

    def write_csv(self, rows, header=None):
        with self.csv.open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(header or ["name", "index", "subindex", "data_type", "access"])
            writer.writerows(rows)

    def test_sort_types_and_no_business_storage(self):
        self.write_csv([["later", "0x2001", "0x02", "REAL32", "rw"],
                        ["earlier", "0x2000", "0x00", "UNSIGNED16", "ro"]])
        entries = generator.load_entries(self.csv)
        self.assertEqual([entry.name for entry in entries], ["earlier", "later"])
        files = generator.render(entries)
        self.assertIn("OD_NAME_earlier = 0", files["od_generated.h"])
        self.assertIn("OD_REAL32, OD_RW, 4", files["od_generated.c"])
        self.assertIn("static OD_Binding bindings", files["od_generated.c"])
        self.assertNotIn("float later", files["od_generated.c"])

    def test_invalid_rows(self):
        base = ["value", "0x2000", "0x00", "REAL32", "rw"]
        cases = [(0, "bad-name"), (0, "1value"), (1, "2000"), (1, "0x10000"),
                 (2, "0x100"), (3, "FLOAT"), (4, "readwrite")]
        for column, value in cases:
            with self.subTest(value=value):
                row = base.copy()
                row[column] = value
                self.write_csv([row])
                with self.assertRaisesRegex(ValueError, "line 2"):
                    generator.load_entries(self.csv)

    def test_duplicates(self):
        for second, message in [(["value", "0x2001", "0x00", "REAL32", "ro"], "duplicate name"),
                                (["other", "0x2000", "0x00", "REAL32", "ro"], "duplicate index")]:
            self.write_csv([["value", "0x2000", "0x00", "REAL32", "rw"], second])
            with self.assertRaisesRegex(ValueError, message):
                generator.load_entries(self.csv)

    def test_schema_and_empty(self):
        for header in [["name", "index"], ["name", "index", "subindex", "data_type", "access", "id"],
                       ["name", "index", "subindex", "data_type", "access", "name"]]:
            self.write_csv([], header)
            with self.assertRaisesRegex(ValueError, "header"):
                generator.load_entries(self.csv)
        self.write_csv([])
        with self.assertRaisesRegex(ValueError, "at least one"):
            generator.load_entries(self.csv)
        self.write_csv([["a", "0x2000", "0x00", "REAL32"]])
        with self.assertRaisesRegex(ValueError, "number of columns"):
            generator.load_entries(self.csv)

    def test_generation_check_and_no_overwrites(self):
        self.write_csv([["value", "0x2000", "0x00", "REAL32", "rw"]])
        args = ["--csv", str(self.csv), "--out-dir", str(self.directory)]
        user_file = self.directory / "od_user.c"
        user_file.write_text("USER MAPPINGS", encoding="utf-8")
        self.assertEqual(generator.main(args), 0)
        output = self.directory / "od_generated.c"
        original = output.read_bytes()
        mtime = output.stat().st_mtime_ns
        self.assertEqual(generator.main(args), 0)
        self.assertEqual(output.stat().st_mtime_ns, mtime)
        self.assertEqual(generator.main(args + ["--check"]), 0)
        self.write_csv([["value", "0x2000", "0x00", "INVALID", "rw"]])
        self.assertEqual(generator.main(args), 1)
        self.assertEqual(output.read_bytes(), original)
        self.assertEqual(user_file.read_text(), "USER MAPPINGS")
        self.write_csv([["renamed", "0x2000", "0x00", "REAL32", "rw"]])
        self.assertEqual(generator.main(args + ["--check"]), 1)
        self.assertEqual(output.read_bytes(), original)

    def test_comments_cannot_inject_source(self):
        self.write_csv([["BOOLEAN", "0x2000", "0x00", "BOOLEAN", "rw", "*/ bad\\\ntext"]],
                       ["name", "index", "subindex", "data_type", "access", "description"])
        source = generator.render(generator.load_entries(self.csv))["od_generated.c"]
        self.assertIn("OD_NAME_BOOLEAN", source)
        self.assertIn("/* * / bad/ text */", source)


class ConfigMappingTests(unittest.TestCase):
    def test_all_configuration_fields_have_correct_csv_and_binding(self):
        entries = {entry.name: entry for entry in generator.load_entries(ROOT / "canopen/object_dictionary.csv")
                   if 0x2000 <= entry.index <= 0x2006}
        user = (ROOT / "canopen/od_user.c").read_text(encoding="utf-8")
        bindings = re.findall(r"BIND_(CONFIG|ENUM)\((\w+),\s*(\w+),\s*([\w.]+)\);", user)
        structures, enums = {}, set()
        for header in ["motor_control/global.h", "motor_control/motor.h", "motor_control/encoder/encoder.h",
                       "motor_control/observer/observer.h", "motor_control/math/pid.h",
                       "motor_control/math/compliance_control.h", "motor_control/math/filter.h",
                       "motor_control/control/hfi.h"]:
            source = (ROOT / header).read_text(encoding="utf-8")
            source = re.sub(r"/\*.*?\*/|//[^\n]*", "", source, flags=re.S)
            for body, structure in re.findall(r"typedef\s+struct\s*\{([^{}]*)\}\s*(\w+)\s*;", source):
                structures[structure] = re.findall(r"(\w+)\s+(\w+)\s*(?:\[[^]]+\])?\s*;", body)
            enums.update(re.findall(r"typedef\s+enum\s*\{[^{}]*\}\s*(\w+)\s*;", source))

        def flatten(structure, prefix=""):
            for c_type, member in structures[structure]:
                path = prefix + member
                if c_type in structures:
                    yield from flatten(c_type, path + ".")
                else:
                    self.assertIn(c_type, {"float", "int", "uint32_t"} | enums)
                    yield c_type, path

        groups = [
            ("Motor_Config_t", "motor_config", "motor0", 26),
            ("Encoder_Config_t", "encoder_config", "encoder0", 11),
            ("Motor_Controller_Config_t", "motor_controller_config", "motor_controller0", 17),
            ("Motor_Observer_Config_t", "motor_observer_config", "motor_observer0", 8),
            ("Loop_Controller_Config_t", "loop_controller_config", "loop_controller0", 58),
            ("Loop_Filter_Config_t", "loop_filter_config", "loop_filter0", 36),
            ("HFI_Config_t", "hfi_config", "hfi0", 10),
        ]
        self.assertEqual(structures["Global_Configs_t"], [(s, g) for s, g, _, _ in groups])
        expected_names = set()
        for index, (structure, group, prefix, count) in enumerate(groups, 0x2000):
            metadata = prefix + "_max_subindex"
            expected_names.add(metadata)
            fields = list(flatten(structure))
            self.assertEqual(len(fields), count)
            actual = [(kind, name, member) for kind, name, bound_group, member in bindings if bound_group == group]
            self.assertEqual([member for _, _, member in actual], [member for _, member in fields])
            entry = entries[metadata]
            self.assertEqual((entry.index, entry.subindex, entry.data_type, entry.access),
                             (index, 0, "UNSIGNED8", "ro"))
            self.assertIn(f"OD_BIND_RO(od_dictionary, {metadata}, &{metadata})", user)
            self.assertRegex(user, rf"OD_NAME_{actual[-1][1]}\s*-\s*OD_NAME_{metadata}")
            for offset, ((kind, name, _), (c_type, _)) in enumerate(zip(actual, fields)):
                entry = entries[name]
                self.assertEqual((entry.index, entry.subindex, entry.access), (index, offset + 1, "rw"))
                expected_type = {"float": "REAL32", "int": "INTEGER32", "uint32_t": "UNSIGNED32"}.get(c_type, "UNSIGNED32")
                self.assertEqual(entry.data_type, expected_type)
                self.assertEqual(kind, "ENUM" if c_type in enums else "CONFIG")
                if c_type in enums:
                    self.assertIn(f"CHECK_ENUM_ABI({c_type});", user)
                expected_names.add(name)
        self.assertEqual(set(entries), expected_names)
        self.assertEqual(len(bindings) + len(groups), len(entries))


class StatusMappingTests(unittest.TestCase):
    def test_all_exposed_status_fields_are_bound_readonly(self):
        entries = {entry.name: entry for entry in generator.load_entries(ROOT / "canopen/object_dictionary.csv")
                   if entry.index >= 0x2100}
        user = (ROOT / "canopen/od_user.c").read_text(encoding="utf-8")
        bindings = re.findall(r"BIND_(STATUS|STATUS_ENUM)\((\w+),\s*([\w.\[\]]+)\);", user)
        structures, enums = {}, set()
        for header in ["global.h", "motor.h", "encoder/encoder.h", "observer/observer.h",
                       "math/pid.h", "math/compliance_control.h", "math/foc_utils.h",
                       "control/foc.h", "control/hfi.h"]:
            source = (ROOT / "motor_control" / header).read_text(encoding="utf-8")
            for body, structure in re.findall(r"typedef\s+struct\s*\{([^{}]*)\}\s*(\w+)\s*;", source):
                body = re.split(r"/\*\s*(?:not in od|以下部分暂时不介入对象字典)\s*\*/", body)[0]
                body = re.sub(r"/\*.*?\*/|//[^\n]*", "", body, flags=re.S)
                structures[structure] = re.findall(r"(?:volatile\s+)?(\w+)\s+(\w+)(?:\[([^]]+)\])?\s*;", body)
            enums.update(re.findall(r"typedef\s+enum\s*\{[^{}]*\}\s*(\w+)\s*;", source))

        def flatten(structure, prefix):
            for c_type, member, dimension in structures[structure]:
                if dimension:
                    self.assertEqual(dimension, "BOARD_SUPPORTED_MOTOR_NUMBER")
                path = prefix + member + ("[0]" if dimension else "")
                if c_type in structures:
                    yield from flatten(c_type, path + ".")
                else:
                    yield c_type, path

        groups = [
            ("Core_Status_t", "core_status", "core", 10),
            ("Power_Status_t", "power_status", "power", 8),
            ("Peripheral_Raw_Status_t", "peripheral_raw_status", "peripheral_raw", 10),
            ("Motor_Basic_Status_t", "motor_basic_status[0]", "motor_basic0", 32),
            ("Encoder_Status_t", "encoder_status[0]", "encoder_status0", 27),
            ("Motor_Status_t", "motor_status[0]", "motor_status0", 88),
            ("Motor_Observer_t", "motor_observer[0]", "motor_observer_status0", 21),
            ("Loop_Controller_t", "loop_controller_status[0]", "loop_controller_status0", 128),
            ("FOC_Status_t", "foc_status[0]", "foc_status0", 78),
            ("HFI_Status_t", "hfi_status[0]", "hfi_status0", 30),
        ]
        self.assertEqual([(t, name) for t, name, _ in structures["Global_Status_t"]],
                         [(t, path.split("[")[0]) for t, path, _, _ in groups])
        expected_names, expected_paths = set(), []
        for index, (structure, path, prefix, count) in enumerate(groups, 0x2100):
            fields = list(flatten(structure, path + "."))
            actual = [(kind, name, member) for kind, name, member in bindings if member.startswith(path + ".")]
            self.assertEqual(len(fields), count)
            self.assertEqual([p for _, _, p in actual], [p for _, p in fields])
            expected_paths.extend(p for _, p in fields)
            metadata = prefix + "_max_subindex"
            entry = entries[metadata]
            self.assertEqual((entry.index, entry.subindex, entry.data_type, entry.access),
                             (index, 0, "UNSIGNED8", "ro"))
            self.assertIn(f"OD_BIND_RO(od_dictionary, {metadata}, &{metadata})", user)
            self.assertRegex(user, rf"OD_NAME_{actual[-1][1]}\s*-\s*OD_NAME_{metadata}")
            expected_names.add(metadata)
            for subindex, ((kind, name, _), (c_type, _)) in enumerate(zip(actual, fields), 1):
                wire = {"float": "REAL32", "int": "INTEGER32", "int32_t": "INTEGER32",
                        "uint32_t": "UNSIGNED32", "uint16_t": "UNSIGNED16", "bool": "BOOLEAN"}
                self.assertIn(c_type, set(wire) | enums)
                entry = entries[name]
                self.assertEqual((entry.index, entry.subindex, entry.data_type, entry.access),
                                 (index, subindex, wire.get(c_type, "UNSIGNED32"), "ro"))
                short_enum = c_type in enums and c_type != "FOC_Run_State_t"
                self.assertEqual(kind, "STATUS_ENUM" if short_enum else "STATUS")
                if short_enum:
                    self.assertIn(f"CHECK_ENUM_ABI({c_type});", user)
                expected_names.add(name)
        self.assertEqual(set(entries), expected_names)
        self.assertEqual(len(bindings), len(expected_paths))
        for excluded in ["motor_status[0].force_status", "motor_status[0].identify_done",
                         "encoder_status[0].encoder_delta_cnt_inv", "foc_status[0].identify_state",
                         "hfi_status[0].uab_k0", "hfi_status[0].is_open_loop"]:
            self.assertFalse(any(path.startswith(excluded) for path in expected_paths))


if __name__ == "__main__":
    (ROOT / "build/od-tests").mkdir(parents=True, exist_ok=True)
    unittest.main()

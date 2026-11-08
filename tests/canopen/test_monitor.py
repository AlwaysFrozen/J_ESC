import ctypes
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/od_monitor"))
from client import Client, DeviceError, ProtocolError
from model import Entry, FORMATS, load_dictionary
from serial_win32 import DCB, Timeouts
from commands import COMMANDS, IDENTIFY, IDENTIFICATIONS, STATUS_NAMES, find_command, is_command
from app import Monitor


class FakePort:
    def __init__(self, reply):
        self.reply = reply
        self.pending = bytearray()
        self.writes = []

    def write(self, data):
        self.writes.append(data)
        sequence = data.split()[1]
        self.pending.extend(self.reply(sequence))

    def read(self, size):
        # Simulate arbitrary USB packet/serial read fragmentation.
        block = self.pending[:min(size, 3)]
        del self.pending[:len(block)]
        return bytes(block)

    def close(self):
        pass


class MonitorTests(unittest.TestCase):
    def setUp(self):
        self.entry = Entry("rated_voltage", 0x2000, 3, "REAL32", "rw")

    def test_golden_request_and_fragmented_reply(self):
        port = FakePort(lambda seq: b"boot log\nO 00000000 00 FF\nO " + seq + b" 00 00004841\n")
        client = Client(port)
        client.sequence = 0
        self.assertEqual(client.read(self.entry), "12.5")
        self.assertEqual(port.writes, [b"R 00000001 2000 03 0008 04\n"])

    def test_write_ack_and_single_transmission(self):
        port = FakePort(lambda seq: b"O " + seq + b" 00 -\n")
        client = Client(port)
        client.sequence = 0
        self.assertEqual(client.write(self.entry, "12.5"), "12.5")
        self.assertEqual(port.writes, [b"W 00000001 2000 03 0008 04 00004841\n"])

    def test_device_rejection_and_bad_response(self):
        client = Client(FakePort(lambda seq: b"O " + seq + b" 81 -\n"))
        with self.assertRaises(DeviceError) as caught:
            client.write(self.entry, "12.5")
        self.assertEqual(caught.exception.status, 0x81)
        client = Client(FakePort(lambda seq: b"O " + seq + b" 00 00\n"))
        with self.assertRaises(ProtocolError):
            client.read(self.entry)

    def test_write_timeout_never_retries(self):
        port = FakePort(lambda seq: b"")
        client = Client(port, timeout=0.001)
        with self.assertRaisesRegex(TimeoutError, "写入结果未知"):
            client.write(self.entry, "12.5")
        self.assertEqual(len(port.writes), 1)

    def test_permissions_and_invalid_values_do_not_transmit(self):
        port = FakePort(lambda seq: b"")
        client = Client(port)
        for text in ["nan", "inf", "1e100", "not_a_number"]:
            with self.subTest(text=text), self.assertRaises(ValueError):
                client.write(self.entry, text)
        with self.assertRaises(ValueError):
            client.write(Entry("maximum", 0x2000, 0, "UNSIGNED8", "ro"), "1")
        with self.assertRaises(ValueError):
            client.read(Entry("command", 0x2000, 5, "UNSIGNED8", "wo"))
        self.assertEqual(port.writes, [])

    def test_all_types_and_integer_boundaries(self):
        for name in FORMATS:
            entry = Entry("value", 0x2000, 1, name, "rw")
            text = "12.5" if name.startswith("REAL") else "1"
            self.assertEqual(entry.decode(entry.encode(text)), text)
        entry = Entry("value", 0x2000, 1, "INTEGER32", "rw")
        self.assertEqual(entry.encode("-2147483648"), struct.pack("<i", -2147483648))
        self.assertEqual(entry.decode(entry.encode("0x7FFFFFFF")), "2147483647")
        self.assertEqual(entry.decode(entry.encode("08")), "8")
        for name, bad in [("INTEGER32", "2147483648"), ("UNSIGNED32", "-1"), ("BOOLEAN", "2")]:
            with self.assertRaises(ValueError):
                Entry("value", 0x2000, 1, name, "rw").encode(bad)

    def test_product_csv_and_native_structure_layout(self):
        entries = load_dictionary(ROOT / "canopen/object_dictionary.csv")
        self.assertEqual(len(entries), 626)
        self.assertEqual((entries[0].index, entries[0].subindex, entries[0].access), (0, 0, "ro"))
        self.assertEqual({entry.index for entry in entries}, {0, 0x1000} | set(range(0x2000, 0x2007)) | set(range(0x2100, 0x210A)))
        self.assertEqual((entries[-1].index, entries[-1].subindex), (0x2109, 0x1E))
        for entry in entries:
            if entry.index >= 0x2100:
                self.assertEqual(entry.access, "ro")
        if sys.platform == "win32":
            self.assertEqual(ctypes.sizeof(DCB), 28)
            self.assertEqual(ctypes.sizeof(Timeouts), 20)

    def test_command_metadata_and_status_names(self):
        entries = load_dictionary(ROOT / "canopen/object_dictionary.csv")
        for command in (*COMMANDS.values(), IDENTIFY):
            found = find_command(entries, command)
            self.assertIsNotNone(found)
            self.assertTrue(is_command(found[1]))
            self.assertIsNone(find_command([Entry(command.name, command.index + 1,
                                                 command.subindex, "UNSIGNED32", "wo")], command))
        self.assertTrue(set(STATUS_NAMES) <= {entry.name for entry in entries})

    def test_command_buttons_queue_one_write_and_honor_busy(self):
        monitor = Monitor.__new__(Monitor)
        monitor.entries = load_dictionary(ROOT / "canopen/object_dictionary.csv")
        monitor.connected, monitor.busy = True, False
        monitor._start = Mock()
        monitor.identification = Mock()
        for key, command in COMMANDS.items():
            monitor.send_command(key)
            number, entry = find_command(monitor.entries, command)
            monitor._start.assert_called_with("write", (number, entry, str(command.value)))
        for label, value in IDENTIFICATIONS.items():
            monitor.identification.get.return_value = label
            monitor.send_command("identify")
            number, entry = find_command(monitor.entries, IDENTIFY)
            monitor._start.assert_called_with("write", (number, entry, str(value)))
        monitor._start.reset_mock()
        monitor.busy = True
        monitor.send_command("enable")
        monitor._start.assert_not_called()
        monitor.busy, monitor.connected = False, False
        monitor.send_command("enable")
        monitor._start.assert_not_called()

    def test_commands_use_requested_addresses_and_never_retry(self):
        entries = load_dictionary(ROOT / "canopen/object_dictionary.csv")
        for command in (*COMMANDS.values(), IDENTIFY):
            entry = find_command(entries, command)[1]
            port = FakePort(lambda seq: b"O " + seq + b" 00 -\n")
            client = Client(port)
            client.sequence = 0
            client.write(entry, str(command.value), timeout=10 if command.name == "system_save_flash" else None)
            self.assertEqual(len(port.writes), 1)
            self.assertIn(f"{command.index:04X} {command.subindex:02X} 0007 04".encode(), port.writes[0])
        port = FakePort(lambda seq: b"")
        client = Client(port, timeout=0.001)
        with self.assertRaises(TimeoutError):
            client.write(find_command(entries, COMMANDS["save"])[1], "1")
        self.assertEqual(len(port.writes), 1)


if __name__ == "__main__":
    unittest.main()

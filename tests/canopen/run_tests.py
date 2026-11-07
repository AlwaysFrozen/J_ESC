"""Run Python regression tests and compile/run the real C object dictionary.

--compile-only --arm supports an Arm Compiler for Embedded installation.
--lld supports Windows hosts without a C runtime/development kit.
"""
import argparse
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="cc")
    parser.add_argument("--lld")
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--arm", action="store_true", help="Arm Compiler for Embedded")
    target.add_argument("--arm-gcc", action="store_true", help="ARM GNU Toolchain")
    parser.add_argument("--compile-only", action="store_true")
    args = parser.parse_args()
    if (args.arm or args.arm_gcc) and not args.compile_only:
        parser.error("ARM targets require --compile-only; run hardware/emulator tests separately")
    root = Path(__file__).resolve().parents[2]
    out = root / "build/od-tests"
    out.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable, str(root / "tests/canopen/test_generator.py")], check=True)
    subprocess.run([sys.executable, str(root / "tests/canopen/test_monitor.py")], check=True)
    subprocess.run([sys.executable, str(root / "tools/generate_od.py"), "--check"], check=True)
    generated = out / "generated"
    subprocess.run([sys.executable, str(root / "tools/generate_od.py"),
                    "--csv", str(root / "tests/canopen/object_dictionary.csv"),
                    "--out-dir", str(generated)], check=True)
    flags = ["-std=c11", "-ffreestanding", "-fno-builtin", "-Wall", "-Wextra", "-Werror"]
    if args.arm:
        flags += ["--target=arm-arm-none-eabi", "-mcpu=cortex-m4"]
    if args.arm_gcc:
        flags += ["-mcpu=cortex-m4", "-mthumb"]
    if args.lld:
        flags += ["-DOD_TEST_NO_CRT"]
    base_flags = flags.copy()
    flags += ["-I", str(root / "tests/canopen/stubs"), "-I", str(root / "canopen"), "-I", str(generated)]
    sources = [root / "canopen/od.c", root / "canopen/od_protocol.c", generated / "od_generated.c",
               root / "canopen/od_cdc.c", root / "tests/canopen/test_od.c",
               root / "tests/canopen/test_protocol.c", root / "tests/canopen/test_cdc.c"]
    objects = []
    for source in sources:
        obj = out / (source.stem + ".obj")
        subprocess.run([args.cc, *flags, "-c", str(source), "-o", str(obj)], check=True)
        objects.append(str(obj))
    # Generic binders must reject unsafe pointer types at compile time.
    for name, statement in {
        "void_pointer": "void *p = 0; return OD_BIND(od_dictionary, bus_voltage, p);",
        "const_write": "const float v = 1; return OD_BIND(od_dictionary, motor0_target_speed, &v);",
    }.items():
        bad = out / (name + ".c")
        bad.write_text('#include "od_generated.h"\nOD_Result bad(void) { ' + statement + ' }\n')
        result = subprocess.run([args.cc, *flags, "-c", str(bad), "-o", str(out / (name + ".obj"))],
                                capture_output=True, text=True)
        if result.returncode == 0:
            raise SystemExit(f"FAIL: unsafe {name} compiled successfully")
        if ("_Generic" not in result.stderr and "generic" not in result.stderr.lower()
                and "od_bind_unsupported_pointer" not in result.stderr):
            raise SystemExit(f"FAIL: {name} failed for an unexpected reason:\n{result.stderr}")
    if args.compile_only:
        if args.arm or args.arm_gcc:
            # Compile product mappings against the actual headers and BSP,
            # independently from the stable scalar-test dictionary fixture.
            mapping_flags = ["-I", str(root / "canopen/generated"), "-I", str(root / "canopen"), *base_flags,
                             "-DHW_VESC_6_4", "-DSTM32F405xx", "-DUSE_HAL_DRIVER"]
            for directory in ["motor_control", "motor_control/math", "motor_control/encoder",
                              "motor_control/observer", "motor_control/control", "motor_control/config",
                              "motor_control/hal", "bsp", "Core/Inc",
                              "Drivers/STM32F4xx_HAL_Driver/Inc", "Drivers/CMSIS/Include",
                              "Drivers/CMSIS/Device/ST/STM32F4xx/Include", "USB_DEVICE/App",
                              "USB_DEVICE/Target", "Middlewares/ST/STM32_USB_Device_Library/Core/Inc",
                              "Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"]:
                mapping_flags += ["-I", str(root / directory)]
            for name in ["canopen/od_user.c", "canopen/generated/od_generated.c", "canopen/od_cdc.c",
                         "USB_DEVICE/App/usbd_cdc_if.c", "Core/Src/main.c"]:
                source = root / name
                subprocess.run([args.cc, *mapping_flags, "-c", str(source),
                                "-o", str(out / (source.stem + "_product.obj"))], check=True)
        print("PASS: generator tests, C compilation and unsafe-pointer rejection; C runtime tests NOT executed")
        return
    exe = out / ("od_tests.exe" if sys.platform == "win32" else "od_tests")
    if args.lld:
        subprocess.run([args.lld, "-flavor", "link", "/entry:main", "/subsystem:console",
                        "/nodefaultlib", "/out:" + str(exe), *objects], check=True)
    else:
        subprocess.run([args.cc, *objects, "-o", str(exe)], check=True)
    result = subprocess.run([str(exe)])
    if result.returncode:
        raise SystemExit(f"FAIL: C assertion line {result.returncode} (POSIX truncates exit codes to 8 bits)")
    print("PASS: C binding, lookup, permissions, freeze, all scalar types and wire format")


if __name__ == "__main__":
    main()

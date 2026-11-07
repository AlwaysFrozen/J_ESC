"""Run the real storage implementation against a fault-injecting Flash backend.

python tests/config_storage/run_tests.py --cc clang
On Windows without a CRT/toolchain installation, also supply --lld path/to/lld.exe.
"""
import argparse
from pathlib import Path
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--cc", default="cc")
parser.add_argument("--lld")
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
out = root / "build" / "config-tests"
out.mkdir(parents=True, exist_ok=True)
test_dir = root / "tests" / "config_storage"
includes = [test_dir / "stubs", root / "motor_control" / "config", root / "motor_control" / "hal"]
sources = [root / "motor_control" / "config" / "config_storage.c", test_dir / "test_config_storage.c"]
flags = ["-std=c11", "-ffreestanding", "-fno-builtin", "-Wall", "-Wextra", "-Werror"]
for include in includes:
    flags += ["-I", str(include)]
objects = []
for source in sources:
    obj = out / (source.stem + ".obj")
    subprocess.run([args.cc, *flags, "-c", str(source), "-o", str(obj)], check=True)
    objects.append(str(obj))
exe = out / ("config_storage_tests.exe" if sys.platform == "win32" else "config_storage_tests")
if args.lld:
    subprocess.run([args.lld, "-flavor", "link", "/entry:main", "/subsystem:console", "/nodefaultlib",
                    "/out:" + str(exe), *objects], check=True)
else:
    subprocess.run([args.cc, *objects, "-o", str(exe)], check=True)
result = subprocess.run([str(exe)])
if result.returncode:
    raise SystemExit(f"FAIL: test_config_storage.c assertion line {result.returncode} (POSIX exit codes truncate to 8 bits)")
print("PASS: boot load, state guards, unchanged saves, all interrupted-write boundaries, erase/unlock/lock errors, read-back verification, corruption fallback, schema/size rejection, sequence wrap")

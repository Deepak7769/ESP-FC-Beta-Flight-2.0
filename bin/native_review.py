#!/usr/bin/env python3
"""Network-free native review checks using already installed ArduinoFake/Unity.
This is a host regression runner, not a replacement for target PlatformIO CI.
"""
from pathlib import Path
import argparse, concurrent.futures, hashlib, os, subprocess, sys
ap = argparse.ArgumentParser()
ap.add_argument("--deps", type=Path, required=True, help="Directory containing ArduinoFake and Unity")
ap.add_argument("--profile", choices=["base", "production", "land-ack"], default="production")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--address-sanitizer", action="store_true")
ap.add_argument("tests", nargs="*")
a = ap.parse_args()
root = Path(__file__).resolve().parents[1]
os.chdir(root)
out = a.out.resolve(); out.mkdir(parents=True, exist_ok=True)
deps = a.deps.resolve()
for required in [deps/"ArduinoFake/src/ArduinoFake.h", deps/"Unity/src/unity.c"]:
    if not required.exists(): ap.error(f"Missing preinstalled dependency: {required}")
includes = list(root.joinpath("lib").rglob("src")) + [deps/"ArduinoFake/src", deps/"ArduinoFake/src/arduino", deps/"Unity/src"]
flags = ["-O0", "-g0", "-ffunction-sections", "-fdata-sections", "-DUNIT_TEST", "-DNO_GLOBAL_INSTANCES", "-DIRAM_ATTR="]
if a.profile != "base":
    flags += ["-DESPFC_ASSISTED_V2_ACTIVE", "-DESPFC_ASSISTED_V2_OUTPUT_ACK", "-DESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL=6", "-DESPFC_ANTI_GRAVITY_ACTIVE"]
if a.profile == "land-ack": flags += ["-DESPFC_LAND_NO_RANGE_AUTODISARM_ACK"]
if a.address_sanitizer: flags += ["-fsanitize=address", "-fno-omit-frame-pointer"]
for inc in includes: flags += ["-I", str(inc)]
hooks = out/"unity_hooks.c"
hooks.write_text("void __attribute__((weak)) setUp(void) {}\nvoid __attribute__((weak)) tearDown(void) {}\n")
sources = list(root.joinpath("lib").rglob("*.cpp")) + list(root.joinpath("lib").rglob("*.c"))
sources += list(deps.joinpath("ArduinoFake/src").rglob("*.cpp")) + [deps/"Unity/src/unity.c", hooks]
def compile_source(source):
    obj = out/(hashlib.sha256(str(source).encode()).hexdigest()[:16]+".o")
    cpp = source.suffix == ".cpp"
    cmd = ["g++" if cpp else "gcc", "-std=c++17" if cpp else "-std=gnu11"] + flags + ["-c", str(source), "-o", str(obj)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode: print(r.stderr, file=sys.stderr, flush=True); return None
    return obj
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
    objects = list(pool.map(compile_source, sources))
if None in objects: sys.exit(1)
archive = out/"firmware.a"
if archive.exists(): archive.unlink()
subprocess.run(["ar", "rcs", str(archive)] + list(map(str,objects)), check=True)
tests = [root/t for t in a.tests] if a.tests else sorted(root.joinpath("test").rglob("*.cpp"))
failed = False
for test in tests:
    obj = compile_source(test)
    if obj is None: failed = True; continue
    exe = out/test.parent.name
    cmd = ["g++", "-Wl,--gc-sections"]
    if a.address_sanitizer: cmd += ["-fsanitize=address"]
    r = subprocess.run(cmd+[str(obj),str(archive),"-o",str(exe)],capture_output=True,text=True)
    if r.returncode: print(r.stderr,file=sys.stderr); failed=True; continue
    r = subprocess.run([str(exe)],capture_output=True,text=True)
    (out/(test.parent.name+".log")).write_text(r.stdout+r.stderr)
    print(test.parent.name, "exit", r.returncode, "\n"+"\n".join(r.stdout.splitlines()[-4:]), flush=True)
    if r.stderr: print(r.stderr,file=sys.stderr)
    failed |= r.returncode != 0
sys.exit(1 if failed else 0)


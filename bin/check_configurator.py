#!/usr/bin/env python3
"""Run real-packet compatibility checks in a separately checked-out client.

Usage: ESPFC_MSP_FIXTURES=/absolute/packets.json python bin/check_configurator.py CLIENT
Generate packets with the native test_closure suite first. ESPFC_MSP_WRITES
exports real client setter payloads for a second native run and readback check.
This is a protocol test, not a physical USB, sensor, or motor test.
"""
import pathlib
import shutil
import subprocess
import sys

client = pathlib.Path(sys.argv[1]).resolve()
source = pathlib.Path(__file__).with_name("configurator_protocol.test.js")
dest = client / "test/espfc_protocol.test.js"
if dest.exists():
    raise SystemExit("Refusing to overwrite an existing client test: " + str(dest))
try:
    shutil.copyfile(source, dest)
    subprocess.run(["npx", "vitest", "run", "test/espfc_protocol.test.js"], cwd=client, check=True)
finally:
    dest.unlink(missing_ok=True)

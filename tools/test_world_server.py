#!/usr/bin/env python3
"""What can be checked without MuJoCo in this container: the contract of the body parser and the fail-closed
behaviour. The physics itself is only verifiable on the machine that has MuJoCo.

    python3 tools/test_world_server.py
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, os.path.join(ROOT, "sim"))
import world_server as ws  # noqa: E402

ok = fail = 0


def check(label, got, want):
    global ok, fail
    if got == want:
        ok += 1
        print("ok   %-56s %r" % (label, got))
    else:
        fail += 1
        print("FAIL %-56s got %r want %r" % (label, got, want))


check("six values pass", sorted(ws.parse_write({"angles": [1, 2, 3, 4, 5, 6]})), sorted(ws.JOINT_ORDER))
check("five with hold pass and leave the jaw out", "gripper" in ws.parse_write(
    {"angles": [1, 2, 3, 4, 5], "hold": ["gripper"]}), False)
for label, body in (("four values", {"angles": [1, 2, 3, 4]}),
                    ("five without hold", {"angles": [1, 2, 3, 4, 5]}),
                    ("NaN", {"angles": [1, 2, 3, 4, float("nan")]}),
                    ("900 degrees", {"angles": [1, 2, 3, 4, 900]}),
                    ("a string", {"angles": [1, 2, 3, 4, "x"]})):
    try:
        ws.parse_write(body)
        check("%s is refused" % label, False, True)
    except ws.Refused:
        check("%s is refused" % label, True, True)

# fail closed: no mujoco here, so starting it must refuse with a sentence and a non-zero exit
out = subprocess.run([sys.executable, os.path.join(ROOT, "sim", "world_server.py"),
                      "--model", "/tmp/none.xml", "--arm-port", "so100_cell=8004"],
                     capture_output=True, text=True, timeout=60)
check("without mujoco it refuses to start", out.returncode, 1)
check("...and says why", "refusing to run without physics" in (out.stdout + out.stderr), True)

out = subprocess.run([sys.executable, os.path.join(ROOT, "sim", "world_server.py"),
                      "--model", "x.xml", "--arm-port", "a=1", "--arm-port", "b=2", "--prefix", ""],
                     capture_output=True, text=True, timeout=60)
check("a prefix per arm is required", "one --prefix per --arm-port" in (out.stdout + out.stderr), True)

print("\n%s — %d checks, %d failed" % ("PASS" if not fail else "FAIL", ok + fail, fail))
sys.exit(1 if fail else 0)

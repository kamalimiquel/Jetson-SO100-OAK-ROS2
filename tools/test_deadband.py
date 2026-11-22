#!/usr/bin/env python3
"""The deadband is a measurement with a range, not a number with a decimal point.

Measured on the bench 2026-11-21 (arm 1, shoulder_pan): two 0.5 deg steps from the same pose moved nothing,
1.0 deg moved 1.231. So the threshold lives between 0.5 and 1.0, and the table records 1.0 with the basis
written next to it — the number a command has to reach, not a claim that the servo switches at exactly that.

    python3 tools/test_deadband.py
"""
import copy
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docker", "ws", "lib"))
import joint_scale as js  # noqa: E402

ok = fail = 0


def check(label, got, want):
    global ok, fail
    if got == want:
        ok += 1
        print("ok   %-56s %r" % (label, got))
    else:
        fail += 1
        print("FAIL %-56s got %r want %r" % (label, got, want))


table = js.load(js.path_for("so100_cell"))
check("the arm's table loads", table["arm"], "so100_cell")
check("pan carries its measured deadband", js.deadband(table, "shoulder_pan"), 1.0)
check("the basis is recorded, not just the number",
      "between 0.5 and 1.0" in table["joints"]["shoulder_pan"].get("deadband_basis", ""), True)
check("an unmeasured joint says None, not zero", js.deadband(table, "elbow_flex"), None)
check("the gripper has no deadband either", js.deadband(table, "gripper"), None)
try:
    js.deadband(table, "tail")
    check("unknown joints are refused", False, True)
except js.ScaleError:
    check("unknown joints are refused", True, True)


def raises(table_, joint, label):
    try:
        js.deadband(table_, joint)
    except js.ScaleError:
        check(label, True, True)
        return
    check(label, False, True)


bad = copy.deepcopy(table)
bad["joints"]["shoulder_pan"]["deadband_deg"] = 0.0
raises(bad, "shoulder_pan", "zero is refused (unmeasured is None, not 0)")
bad2 = copy.deepcopy(table)
bad2["joints"]["shoulder_pan"]["deadband_deg"] = 12
raises(bad2, "shoulder_pan", "a deadband of 12 deg is not a deadband")
bad3 = copy.deepcopy(table)
bad3["joints"]["shoulder_pan"]["deadband_deg"] = "1"
raises(bad3, "shoulder_pan", "a string is refused")

# The other arm has no measurement, so the same helper must not answer for it.
try:
    table_b = js.load(js.path_for("so100_cell_b"))
    check("arm 2 has its own, larger deadband", js.deadband(table_b, "shoulder_pan"), 2.0)
    check("arm 1 did not inherit arm 2's number", js.deadband(table, "shoulder_pan"), 1.0)
except js.ScaleError as exc:
    check("arm 2 table loads (%s)" % exc, False, True)

print("\n%s — %d checks, %d failed" % ("PASS" if not fail else "FAIL", ok + fail, fail))
sys.exit(1 if fail else 0)

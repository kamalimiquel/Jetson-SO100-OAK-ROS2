#!/usr/bin/env python3
"""Checks for the parked pose: the state autonomy is allowed to start from.

    python3 tools/test_park.py
"""
import copy
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docker", "ws", "lib"))
import park  # noqa: E402

ok = fail = 0


def check(label, got, want):
    global ok, fail
    if got == want:
        ok += 1
        print("ok   %-56s %r" % (label, got))
    else:
        fail += 1
        print("FAIL %-56s got %r want %r" % (label, got, want))


A1, A2 = "so100_cell", "so100_cell_b"
good1 = dict(park.PARKED[A1])
good2 = dict(park.PARKED[A2])

check("the exact park is parked", park.check_arm(A1, good1)[0], True)
check("and it says so with the worst joint", "parked (worst" in park.check_arm(A1, good1)[1], True)
check("...naming a joint even when the deviation is zero", "worst None" in park.check_arm(A1, good1)[1], False)
check("a missing joint is a refusal, not a pass",
      park.check_arm(A1, {k: v for k, v in good1.items() if k != "elbow_flex"})[0], False)
check("...and names it", "elbow_flex" in park.check_arm(
    A1, {k: v for k, v in good1.items() if k != "elbow_flex"})[1], True)

near = dict(good1); near["shoulder_lift"] += park.TOLERANCE_DEG - 0.5
check("inside the tolerance it passes", park.check_arm(A1, near)[0], True)
far = dict(good1); far["elbow_flex"] += park.TOLERANCE_DEG + 1
check("outside the tolerance it fails", park.check_arm(A1, far)[0], False)
check("...and says how far off", "deg off" in park.check_arm(A1, far)[1], True)
check("...and which joint", "elbow_flex" in park.check_arm(A1, far)[1], True)

# the two arms have their own poses and their own judgement
check("each arm has a pose", sorted(park.PARKED), [A1, A2])
check("arm 2 asked with arm 2's pose passes", park.check_arm(A2, good2)[0], True)
try:
    park.check_arm("so100_cell_c", good1)
    check("an unknown arm is refused", False, True)
except park.ParkError as exc:
    check("an unknown arm is refused", "no parked pose" in str(exc), True)
try:
    park.check_arm(A1, None)
    check("no readings is an error, not a pass", False, True)
except park.ParkError:
    check("no readings is an error, not a pass", True, True)
check("a non-numeric reading fails closed", park.check_arm(A1, dict(good1, gripper="x"))[0], False)

# the cell: all or nothing, and silence from one arm is a refusal
ok_cell, why = park.check_cell({A1: good1, A2: good2})
check("both parked is a parked cell", ok_cell, True)
ok_cell, why = park.check_cell({A1: good1})
check("one arm silent is not a parked cell", ok_cell, False)
check("...and says which and why", "not answering" in why[A2], True)
ok_cell, why = park.check_cell({A1: good1, A2: far})
check("one arm off is not a parked cell", ok_cell, False)
check("...and the other arm still reports", "parked" in why[A1], True)

# the tolerance is a decision, not a default: it must clear the deadband measured on both arms
check("tolerance clears arm 2's 2 deg deadband with margin", park.TOLERANCE_DEG >= 3 * 2.0, True)

print("\n%s — %d checks, %d failed" % ("PASS" if not fail else "FAIL", ok + fail, fail))
sys.exit(1 if fail else 0)

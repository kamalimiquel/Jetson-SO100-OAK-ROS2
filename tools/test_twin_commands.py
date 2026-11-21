#!/usr/bin/env python3
"""Checks for the commanded twin's arithmetic — no model, no cell, no viewer.

Every case here is one that fails silently in production: a five-value command read as six, a NaN that
becomes a huge error, a stale command reported as a fault the operator then chases.

    python3 tools/test_twin_commands.py
"""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "windows"))
import twin_commands as tc  # noqa: E402

ok = fail = 0


def check(label, got, want):
    global ok, fail
    if got == want:
        ok += 1
        print("ok   %-58s %r" % (label, got))
    else:
        fail += 1
        print("FAIL %-58s got %r want %r" % (label, got, want))


# ---- decode: position in the array is the contract
check("five values name the five joints and hold the jaw",
      sorted(tc.decode_targets([1, 2, 3, 4, 5])), sorted(tc.JOINT_ORDER[:5]))
check("the order is the contract, not the value",
      tc.decode_targets([1, 2, 3, 4, 5])["shoulder_pan"], 1.0)
check("six values include the jaw", tc.decode_targets([1, 2, 3, 4, 5, 6])["gripper"], 6.0)
check("five values leave the jaw out", "gripper" in tc.decode_targets([1, 2, 3, 4, 5]), False)
check("floats survive", tc.decode_targets(["12.5", -3, 0, 0, 0])["shoulder_pan"], 12.5)
check("an empty array is not a command", tc.decode_targets([]), {})


def raises(values, label):
    try:
        tc.decode_targets(values)
    except ValueError as exc:
        check(label, True, True)
        return str(exc)
    check(label, False, True)
    return ""


raises([1, 2, 3, 4], "four values are refused")
raises([1, 2, 3, 4, 5, 6, 7], "seven values are refused")
raises([1, 2, 3, 4, float("nan")], "NaN is refused")
raises([1, 2, 3, 4, float("inf")], "inf is refused")
raises([1, 2, 3, 4, 181], "beyond 180 deg is refused")
raises([1, 2, 3, 4, "abc"], "a string that is not a number is refused")
check("the refusal names the index", "4" in raises([1, 2, 3, 4, float("nan")], "message has the index"), True)

# ---- compare: only joints both sides know
cmd = tc.decode_targets([10, 20, 30, 40, 50])
act = {"shoulder_pan": 9.5, "elbow_flex": 30.0}
err = tc.compare(cmd, act)
check("only the joints with a reading are compared", sorted(err), ["elbow_flex", "shoulder_pan"])
check("error is commanded minus actual", err["shoulder_pan"], 0.5)
check("a joint at its command has zero error", err["elbow_flex"], 0.0)
check("a missing reading is not invented", "shoulder_lift" in err, False)
check("no readings at all gives no errors", tc.compare(cmd, {}), {})
# 1.0005 in binary is slightly below 1.0005, so it rounds down; use an unambiguous value.
check("rounding keeps three decimals", tc.compare({"shoulder_pan": 1.0006}, {"shoulder_pan": 0.0})["shoulder_pan"], 1.001)

# ---- staleness: a command is an intention while it is young
now = 1_000_000.0
check("a fresh command counts", tc.fresh(now - 0.1, now), True)
check("exactly at the limit still counts", tc.fresh(now - tc.STALE_S, now), True)
check("an old command does not", tc.fresh(now - tc.STALE_S - 0.01, now), False)

# ---- the log: header and rows agree, in JOINT_ORDER
act2 = {"shoulder_pan": 9.75, "wrist_roll": 55.0}      # two joints, as a reading would be
err2 = tc.compare(cmd, act2)
rows = tc.csv_rows(12.5, "arm1", err2, cmd, act2)
check("one row per compared joint", len(rows), 2)
check("rows follow JOINT_ORDER, not the dict", [r.split(",")[2] for r in rows],
      ["shoulder_pan", "wrist_roll"])
check("a row carries all six fields", len(rows[0].split(",")), 6)
check("the header matches the row width", len(tc.CSV_HEADER.split(",")), 6)
check("the row parses as csv", len(rows[0].split(",")[0]), len("%.3f" % 12.5))

print("\n%s — %d checks, %d failed" % ("PASS" if not fail else "FAIL", ok + fail, fail))
sys.exit(1 if fail else 0)

#!/usr/bin/env python3
"""Headless test of the command contract — no ROS, no arm, no HTTP.

This is the interface that fails with no exception: `Float64MultiArray` carries no names, no stamp
and no owner, so five, six and seven values all "worked" and the jaw silently kept its old goal.
The contract is the answer, and these are the cases it must refuse — a rejection is only worth
anything if the sentence names the joint and says what to send instead.

    python3 tools/test_command_contract.py
"""

import importlib.util
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LIB = os.path.join(os.path.dirname(HERE), "docker", "ws", "lib", "command_contract.py")

fails = []


def check(name, got, want):
    ok = got == want
    print(("  ok   " if ok else "  FAIL ") + "%s: %r" % (name, got) +
          ("" if ok else " (expected %r)" % (want,)))
    if not ok:
        fails.append(name)


def reject(fn, *args):
    """(refused?, code, sentence) — the shape every rejection must have."""
    try:
        fn(*args)
    except cc.CommandError as exc:
        return True, exc.code, str(exc)
    return False, None, ""


_spec = importlib.util.spec_from_file_location("command_contract", LIB)
cc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cc)
sys.modules["command_contract"] = cc


def main():
    six = [10.0, -20.0, 30.0, -40.0, 50.0, 60.0]

    print("a complete command")
    driven, held, notes = cc.validate_body({"angles": six})
    check("six values drive all six joints", len(driven), 6)
    check("nothing is held", held, set())
    check("no clamp notes", notes, [])
    check("values land on the right joint", driven["gripper"], 60.0)

    print("five values, the old silent convention")
    ok, code, why = reject(cc.validate_body, {"angles": six[:5]})
    check("five values alone are refused", ok, True)
    check("as a client error, not a 500", code, 400)
    check("the sentence says how to say it", "hold" in why, True)

    driven, held, _ = cc.validate_body({"angles": six[:5], "hold": ["gripper"]})
    check("five + hold gripper is accepted", len(driven), 5)
    check("and the jaw is the held joint", held, {"gripper"})
    check("the five land in order, jaw absent", (driven["shoulder_pan"], driven["wrist_roll"]),
          (10.0, 50.0))
    check("no gripper value invented", "gripper" in driven, False)

    print("seven values, and the joint that may not be held")
    ok, code, why = reject(cc.validate_body, {"angles": six + [1.0]})
    check("seven values are refused", ok, True)
    check("the sentence mentions the seventh motor", "seventh" in why, True)
    ok, code, why = reject(cc.validate_body, {"angles": six[:5], "hold": ["shoulder_pan"]})
    check("an arm joint cannot be held", ok, True)
    check("and only the jaw is named as holdable", "gripper" in why, True)
    ok, _, why = reject(cc.validate_body, {"angles": six[:5], "hold": ["elbow"]})
    check("an unknown joint name is refused", ok, True)

    print("non-finite values")
    bad = list(six)
    bad[2] = float("nan")
    ok, code, why = reject(cc.validate_body, {"angles": bad})
    check("NaN in an arm joint is refused", ok, True)
    check("naming the joint", "elbow_flex" in why, True)
    bad = list(six)
    bad[0] = float("inf")
    ok, _, why = reject(cc.validate_body, {"angles": bad})
    check("infinite is refused", ok, True)
    bad = list(six)
    bad[1] = "twelve"
    check("a non-number is refused", reject(cc.validate_body, {"angles": bad})[0], True)

    print("limits are a rejection, not a silent clamp")
    over = list(six)
    over[0] = 190.0
    ok, code, why = reject(cc.validate_body, {"angles": over})
    check("190° on shoulder_pan is refused", ok, True)
    check("with the range in the sentence", "-170..170" in why, True)
    driven, _, _ = cc.validate_body({"angles": over, "clamp": True})
    check("clamp: true means 'as far as it goes'", driven["shoulder_pan"], 170.0)
    jaw = list(six)
    jaw[5] = 130.0
    ok, _, why = reject(cc.validate_body, {"angles": jaw})
    check("the jaw is 0..100 %, not ±170", (ok, "0..100" in why), (True, True))

    print("malformed bodies")
    check("no angles list", reject(cc.validate_body, {})[0], True)
    check("angles is not a list", reject(cc.validate_body, {"angles": 6})[0], True)
    check("hold as a bare string", reject(cc.validate_body, {"angles": six[:5], "hold": "gripper"})[0],
          True)
    check("a body that is not an object", reject(cc.validate_body, [six])[0], True)

    print("the topic carrier (no names, no stamp, NaN is the only 'leave it alone')")
    driven, held, _ = cc.validate_topic(six)
    check("six values, nothing held", (len(driven), held), (6, set()))
    ok, code, why = reject(cc.validate_topic, six[:5])
    check("five on the topic is refused", ok, True)
    check("and NaN is offered as the in-band way", "NaN" in why, True)
    check("seven on the topic is refused", reject(cc.validate_topic, six + [0.0])[0], True)
    nan_jaw = six[:5] + [float("nan")]
    driven, held, _ = cc.validate_topic(nan_jaw)
    check("NaN in the jaw slot means held", (len(driven), held), (5, {"gripper"}))
    nan_arm = [float("nan")] + six[1:]
    ok, _, why = reject(cc.validate_topic, nan_arm)
    check("NaN on an arm joint is refused", ok, True)
    over = list(six)
    over[3] = -200.0
    check("out of range on the topic is refused", reject(cc.validate_topic, over)[0], True)

    print("the log line")
    check("describe() names joints, not indices",
          cc.describe({"shoulder_pan": 1.0, "gripper": 2.0}), "shoulder_pan=1.0, gripper=2.0")

    print("\n%s" % ("all checks pass" if not fails else "%d check(s) FAILED" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

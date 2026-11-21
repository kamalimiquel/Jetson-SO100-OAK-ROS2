#!/usr/bin/env python3
"""Headless test of the supervisor's decision logic (docker/ws/lib/skill_plan.py)."""
import importlib.util
import os
import sys

LIB = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docker", "ws", "lib")
sys.path.insert(0, LIB)
import joint_scale as js        # noqa: E402
import skill_plan as sp         # noqa: E402

fails = []


def check(name, got, want):
    ok = got == want
    print(("  ok   " if ok else "  FAIL ") + "%s: %r" % (name, got) + ("" if ok else " (expected %r)" % (want,)))
    if not ok:
        fails.append(name)


A1, A2 = "so100_cell", "so100_cell_b"
CTX = {
    "arms": [A1, A2],
    "owned": [A1],
    "scales": {A1: js.load(js.path_for(A1)), A2: js.load(js.path_for(A2))},
    "jaw": {A1: {"closed_mm": 16.0, "mm_per_pct": 1.346}, A2: {"closed_mm": 12.4, "mm_per_pct": 0.733}},
    "present": {A1: {n: 0.0 for n in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
                                      "wrist_roll", "gripper")}},
}

six = [10.0, 0.0, 0.0, 0.0, 0.0, 50.0]
r = sp.plan("move_joint", "m1", '{"arm": "%s", "angles": %s}' % (A1, six), CTX)
check("a six-value physical command is accepted", r["accepted"], True)
check("and is converted to sdk on the way out",
      round(r["action"]["driven"]["shoulder_pan"], 2), round(10.0 / 1.059, 2))
check("the units it came in are recorded", r["action"]["units_in"], "physical")

check("no mission id is refused", sp.plan("move_joint", "", '{"arm": "%s"}' % A1, CTX)["accepted"], False)
check("an unknown skill is refused", sp.plan("fly", "m1", "{}", CTX)["accepted"], False)
check("and says what exists", "move_joint" in sp.plan("fly", "m1", "{}", CTX)["reason"], True)
check("broken json is refused", sp.plan("move_joint", "m1", "{oops", CTX)["accepted"], False)
check("an unowned arm is refused",
      sp.plan("move_joint", "m1", '{"arm": "%s", "angles": %s}' % (A2, six), CTX)["accepted"], False)
check("an unknown arm is refused",
      sp.plan("move_joint", "m1", '{"arm": "arm3", "angles": %s}' % six, CTX)["accepted"], False)
check("five values without hold is refused",
      sp.plan("move_joint", "m1", '{"arm": "%s", "angles": [0,0,0,0,0]}' % A1, CTX)["accepted"], False)
check("five values with hold is accepted",
      sp.plan("move_joint", "m1", '{"arm": "%s", "angles": [0,0,0,0,0], "hold": ["gripper"]}'
              % A1, CTX)["accepted"], True)
check("out of range is refused",
      sp.plan("move_joint", "m1", '{"arm": "%s", "angles": [200,0,0,0,0,0]}' % A1, CTX)["accepted"], False)
check("sdk units pass through unscaled",
      sp.plan("move_joint", "m1", '{"arm": "%s", "angles": %s, "units": "sdk"}' % (A1, six),
              CTX)["action"]["driven"]["shoulder_pan"], 10.0)

check("close_jaw goes to 0 %", sp.plan("close_jaw", "m1", '{"arm": "%s"}' % A1, CTX)["action"]["pct"], 0.0)
check("open_jaw below the closed gap is refused",
      sp.plan("open_jaw", "m1", '{"arm": "%s", "mm": 10}' % A1, CTX)["accepted"], False)
check("and says why", "crushed" in sp.plan("open_jaw", "m1", '{"arm": "%s", "mm": 10}' % A1, CTX)["reason"],
      True)
check("open_jaw past 50 % is refused",
      sp.plan("open_jaw", "m1", '{"arm": "%s", "mm": 90}' % A1, CTX)["accepted"], False)
check("open_jaw inside the parallel range is accepted",
      sp.plan("open_jaw", "m1", '{"arm": "%s", "mm": 40}' % A1, CTX)["accepted"], True)
check("and resolves mm to % with THAT arm's curve",
      sp.plan("open_jaw", "m1", '{"arm": "%s", "mm": 40}' % A1, CTX)["action"]["pct"], round((40 - 16) / 1.346, 2))
check("arm 2's curve gives a different percentage for the same 40 mm",
      sp.plan("open_jaw", "m1", '{"arm": "%s", "mm": 40}' % A2, dict(CTX, owned=[A2]))["action"]["pct"],
      round((40 - 12.4) / 0.733, 2))
check("an arm with no curve cannot be asked for millimetres",
      sp.plan("open_jaw", "m1", '{"arm": "%s", "mm": 40}' % A1, dict(CTX, jaw={}))["accepted"], False)

print("\n%s" % ("all checks pass" if not fails else "%d check(s) FAILED" % len(fails)))
sys.exit(1 if fails else 0)

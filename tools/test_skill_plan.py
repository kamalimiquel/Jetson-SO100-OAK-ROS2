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

# ---- move_pose: the pose has to survive the solver before it becomes motion
CTXP = dict(CTX, base_frame={A1: "arm1_base", A2: "arm2_base"})
POSE = '{"arm": "%s", "pose": {"x": 0.2, "y": 0.0, "z": 0.15}}'
r = sp.plan("move_pose", "m1", POSE % A1, CTXP)
check("a pose is dispatched to IK", (r["accepted"], r["action"]["kind"]), (True, "solve"))
check("with this arm's base frame", r["action"]["frame"], "arm1_base")
check("and the default quaternion", r["action"]["pose"][3:], [0.0, 0.0, 0.0, 1.0])
check("a pose in another frame is refused",
      sp.plan("move_pose", "m1", POSE.replace('"arm"', '"frame": "base", "arm"') % A1, CTXP)["accepted"], False)
check("an unnormalised quaternion is refused",
      sp.plan("move_pose", "m1",
              '{"arm": "%s", "pose": {"x":0.2,"y":0,"z":0.15,"qw":2.0}}' % A1, CTXP)["accepted"], False)
check("a pose without x/y/z is refused",
      sp.plan("move_pose", "m1", '{"arm": "%s", "pose": {"x":0.2}}' % A1, CTXP)["accepted"], False)
check("an unowned arm cannot be posed",
      sp.plan("move_pose", "m1", POSE % A2, CTXP)["accepted"], False)

# ---- a step below the deadband is accepted, but said out loud
CTX2 = dict(CTX, owned=list(CTX["owned"]) + [A2],       # arm 2 is owned and reporting only here
            present=dict(CTX["present"], **{A2: dict(CTX["present"][A1])}))


def warn_of(arm, pan, ctx=None):
    body = '{"arm": "%s", "angles": [%s, 0, 0, 0, 0, 0]}' % (arm, pan)
    r = sp.plan("move_joint", "m1", body, ctx or CTX)
    return r["accepted"], (r.get("action") or {}).get("warnings", [])

acc, w = warn_of(A1, 0.5)
check("a 0.5 deg step on arm 1 is still accepted", acc, True)
check("but it warns: below arm 1's deadband", bool(w) and "deadband (1.0 deg)" in w[0], True)
acc, w = warn_of(A1, 5.0)
check("a 5 deg step on arm 1 does not warn", w, [])
acc, w = warn_of(A2, 1.5, CTX2)
check("a 1.5 deg step on arm 2 warns: its deadband is 2.0", bool(w) and "deadband (2.0 deg)" in w[0], True)
acc, w = warn_of(A2, 5.0, CTX2)
check("the same 5 deg step on arm 2 does not", w, [])
check("the warning names the joint", "shoulder_pan" in warn_of(A1, 0.5)[1][0], True)
check("a refusal still refuses (no warning path)", sp.plan(
    "move_joint", "m1", '{"arm": "%s", "angles": [999, 0, 0, 0, 0, 0]}' % A1, CTX)["accepted"], False)

# ---- the deadline is part of the request, and a bad one is refused before anything moves
def dl(body):
    return sp.plan("move_joint", "m1", body, CTX)

check("a command without deadline gets one second", dl('{"arm": "%s", "angles": [5,0,0,0,0,0]}' % A1)
      .get("action", {}).get("deadline_s"), 1.0)
check("a command can ask for a shorter one", dl('{"arm": "%s", "angles": [5,0,0,0,0,0], "deadline_s": 0.2}'
      % A1).get("action", {}).get("deadline_s"), 0.2)
check("a deadline of zero is refused", dl('{"arm": "%s", "angles": [5,0,0,0,0,0], "deadline_s": 0}' % A1)["accepted"], False)
check("five seconds is the ceiling", dl('{"arm": "%s", "angles": [5,0,0,0,0,0], "deadline_s": 9}' % A1)["accepted"], False)
check("a non-numeric deadline is refused", dl('{"arm": "%s", "angles": [5,0,0,0,0,0], "deadline_s": "soon"}' % A1)["accepted"], False)

# ---- solve_ik (MCP Phase 3): report only, canonical arm names, the arm's own base frame
CTX2 = dict(CTX, base_frame={A1: "arm1_base", A2: "arm2_base"})
def ik(body):
    return sp.plan("solve_ik", "m1", body, CTX2)

r = ik('{"arm": "arm1", "pose": {"x": 0.2, "y": 0.0, "z": 0.18}}')
check("solve_ik is accepted (report only)", r["accepted"], True)
check("...and its action is solve_report, not solve", r.get("action", {}).get("kind"), "solve_report")
check("...the canonical arm1 maps to so100_cell", r.get("action", {}).get("arm"), A1)
check("...the default frame is the arm's own", r.get("action", {}).get("frame"), "arm1_base")
check("a pose in the old bare `base` is refused", ik('{"arm": "arm1", "pose": {"x": 0, "y": 0, "z": 0}, "frame": "base"}')["accepted"], False)
check("...and says which frame it wanted", "arm1_base" in ik('{"arm": "arm1", "pose": {"x": 0, "y": 0, "z": 0}, "frame": "base"}')["reason"], True)
check("a non-normalised quaternion is refused", ik('{"arm": "arm1", "pose": {"x": 0, "y": 0, "z": 0, "qw": 0.5}}')["accepted"], False)
check("an unowned arm is refused even for a report", ik('{"arm": "arm2", "pose": {"x": 0, "y": 0, "z": 0}}')["accepted"], False)
check("a pose without xyz is refused", ik('{"arm": "arm1", "pose": {"x": 0}}')["accepted"], False)

print("\n%s" % ("all checks pass" if not fails else "%d check(s) FAILED" % len(fails)))
sys.exit(1 if fails else 0)

#!/usr/bin/env python3
"""Arms parked: the state autonomy is allowed to start from, checked rather than assumed.

The spec makes a verified park a precondition, not a courtesy: a mission that starts with arm 1 anywhere near
the workspace is a collision waiting for the first trajectory. Until now "parked" was a phrase in a document;
here it is six numbers per arm and a tolerance that says why a pose does not qualify.

    from park import PARKED, check_arm
    check_arm("so100_cell", present)          # -> (True, "parked") or (False, "elbow_flex is 24.1 deg off")

Two decisions worth knowing:

  - **The park is per arm.** Arm 2 sits 0.3 m away in +y and its usable travel differs (its pan deadband is
    twice arm 1's), so a single canonical pose would be wrong for one of them.
  - **The jaw is held near closed but not at zero.** Closed on nothing means the fingers touch and the servo
    holds a position it cannot reach; the parked aperture leaves the fingers apart by the measured closed gap
    plus a millimetre.
"""
from __future__ import annotations

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")

# Canonical park, in the physical degrees the arm reports. Arm 1 folds back and down, away from the bench
# centre; arm 2 mirrors it around its own base so the two folded arms do not meet.
PARKED = {
    "so100_cell": {"shoulder_pan": 0.0, "shoulder_lift": -95.0, "elbow_flex": 95.0,
                   "wrist_flex": 60.0, "wrist_roll": 0.0, "gripper": 5.0},
    "so100_cell_b": {"shoulder_pan": 0.0, "shoulder_lift": -95.0, "elbow_flex": 95.0,
                     "wrist_flex": 60.0, "wrist_roll": 0.0, "gripper": 5.0},
}
# Wide enough for the deadband measured on both arms (1.0 and 2.0 deg on the pan) plus the settling seen at
# the bench, and narrow enough that an arm pointing at the workspace cannot pass.
TOLERANCE_DEG = 6.0


class ParkError(ValueError):
    """The pose cannot be judged. The message says which joint is missing."""


def for_arm(arm):
    if arm not in PARKED:
        raise ParkError("no parked pose is defined for %r; known: %s" % (arm, sorted(PARKED)))
    return PARKED[arm]


def check_arm(arm, present, tolerance=TOLERANCE_DEG):
    """(True, "parked") or (False, sentence). A missing reading is a refusal, never a pass."""
    target = for_arm(arm)
    if not isinstance(present, dict):
        raise ParkError("%s: present joints must be a mapping" % arm)
    missing = [j for j in JOINTS if j not in present]
    if missing:
        return False, "%s is not reporting %s, so the park cannot be verified" % (arm, ", ".join(missing))
    worst, worst_joint, worst_off = None, None, 0.0
    for joint in JOINTS:
        try:
            off = abs(float(present[joint]) - float(target[joint]))
        except (TypeError, ValueError):
            return False, "%s: %s is not a number (%r)" % (arm, joint, present[joint])
        if off > worst_off:
            worst, worst_joint, worst_off = joint, joint, off
    if worst_off <= tolerance:
        return True, "parked (worst %s at %.1f deg of %.1f allowed)" % (worst_joint, worst_off, tolerance)
    return False, ("%s is not parked: %s is %.1f deg off (%.1f allowed) — park it before a mission"
                   % (arm, worst_joint, worst_off, tolerance))


def check_cell(present_by_arm, tolerance=TOLERANCE_DEG):
    """Both arms, all or nothing. Returns (ok, {arm: sentence})."""
    out, ok = {}, True
    for arm in PARKED:
        state = present_by_arm.get(arm)
        if state is None:
            out[arm] = "no readings — the arm is not answering, so it cannot be parked"
            ok = False
            continue
        good, why = check_arm(arm, state, tolerance)
        out[arm] = why
        ok = ok and good
    return ok, out

"""What a skill request means, decided without ROS or an arm.

The supervisor node is thin on purpose: it owns the ports, renews the leases and carries messages. Every
judgement — is this skill known, is the arm owned, does the command survive the contract, does the jaw
aperture exist on this arm — happens here, as a pure function, so it can be tested with numbers instead of
a bench.

A skill is bounded by construction: it names one arm, one of a closed list of actions, and the arguments
are validated against `command_contract` (six values or five plus a held jaw) and against the measured jaw
curve. Anything else is a refusal with a sentence, never a partial move.

    plan(skill, mission_id, args, ctx) -> {"accepted": bool, "reason": str, "action": dict|None}

`ctx` carries what the caller cannot be trusted to know: which arms are owned, where each arm is, the
per-arm scale table, and the jaw curve of that arm.
"""

import json
import math

import command_contract as cc
import joint_scale as js

SKILLS = ("move_joint", "move_pose", "hold", "open_jaw", "close_jaw")


def _refuse(reason):
    return {"accepted": False, "reason": reason, "action": None}


def jaw_limits(curve):
    """(closed_mm, mm_per_pct) from a jaw curve row set, or None when the arm has no measured curve."""
    if not curve:
        return None
    k = curve.get("mm_per_pct")
    c = curve.get("closed_mm")
    if k is None or c is None:
        return None
    return float(c), float(k)


def plan(skill, mission_id, args, ctx):
    """Decide one request. No I/O, no clocks: same input, same answer, in a test or on the bench."""
    if not mission_id:
        return _refuse("mission_id is required: an outcome with no mission cannot be scored or traced")
    if skill not in SKILLS:
        return _refuse("unknown skill %r — this cell offers %s (a new skill needs a new .srv, not a "
                       "silent guess)" % (skill, ", ".join(SKILLS)))
    try:
        payload = json.loads(args or "{}")
    except ValueError as exc:
        return _refuse("args_json is not JSON: %s" % exc)
    if not isinstance(payload, dict):
        return _refuse("args_json must be an object")

    arm = payload.get("arm")
    if arm not in ctx["arms"]:
        return _refuse("arm must be one of %s, got %r" % (ctx["arms"], arm))
    if arm not in ctx["owned"]:
        return _refuse("arm %r is not owned by this supervisor — claim it before asking it to move" % arm)

    if skill == "move_pose":
        # A pose goes through the typed IK, not through a straight line of joint numbers: the solver is
        # the only thing that knows whether the arm can reach it, and its answer is the one that decides
        # whether anything is written.
        pose = payload.get("pose") or {}
        try:
            xyz = [float(pose[k]) for k in ("x", "y", "z")]
            quat = [float(pose.get(k, d)) for k, d in (("qx", 0.0), ("qy", 0.0), ("qz", 0.0), ("qw", 1.0))]
        except (KeyError, TypeError, ValueError) as exc:
            return _refuse("move_pose needs 'pose' as {x,y,z} plus optional qx,qy,qz,qw: %s" % exc)
        if not all(math.isfinite(v) for v in xyz + quat):
            return _refuse("move_pose: pose contains a non-finite number")
        if abs(sum(q * q for q in quat) - 1.0) > 0.01:
            return _refuse("move_pose: the quaternion is not normalised (norm^2 = %.3f)"
                           % sum(q * q for q in quat))
        frame = str(payload.get("frame") or ctx["base_frame"][arm])
        if frame != ctx["base_frame"][arm]:
            return _refuse("move_pose: frame %r is not this cell's base frame for %r (%r); no extrinsics "
                           "are applied here" % (frame, arm, ctx["base_frame"][arm]))
        try:
            max_pos = float(payload.get("max_pos_err_mm", 2.0))
            max_ori = float(payload.get("max_ori_err_deg", 3.0))
        except (TypeError, ValueError):
            return _refuse("move_pose: the tolerances must be numbers")
        return {"accepted": True, "reason": "dispatched to IK",
                "action": {"kind": "solve", "arm": arm, "frame": frame,
                           "pose": xyz + quat, "max_pos": max_pos, "max_ori": max_ori}}

    if skill in ("move_joint", "hold"):
        angles = payload.get("angles")
        hold = payload.get("hold") or []
        body = {"angles": angles, "hold": hold}
        units = str(payload.get("units") or "physical").strip().lower()
        if units not in ("physical", "sdk"):
            return _refuse("units must be 'physical' or 'sdk'")
        transform = None
        if units == "physical":
            table = ctx["scales"].get(arm)
            if not table:
                return _refuse("arm %r has no measured scale table: physical degrees would be a guess"
                               % arm)
            transform = lambda n, v: js.to_sdk(table, n, v)      # noqa: E731
        try:
            driven, held, notes = cc.validate_body(body, transform=transform)
        except cc.CommandError as exc:
            return _refuse(str(exc))
        for name in driven:
            if name not in ctx["present"].get(arm, {}):
                return _refuse("%s: the arm is not reporting this joint, so its present angle is unknown"
                               % name)
        return {"accepted": True, "reason": "ok", "action": {
            "kind": "write", "arm": arm, "driven": driven, "held": sorted(held),
            "notes": notes, "units_in": units}}

    # jaw skills: an aperture in millimetres, resolved against THIS arm's curve
    limits = jaw_limits(ctx["jaw"].get(arm))
    if limits is None:
        return _refuse("arm %r has no measured jaw curve: an aperture in millimetres cannot be resolved"
                       % arm)
    closed, mm_per_pct = limits
    if skill == "close_jaw":
        return {"accepted": True, "reason": "ok",
                "action": {"kind": "jaw", "arm": arm, "pct": 0.0, "achieved_mm": closed}}
    if skill == "open_jaw":
        want = payload.get("mm")
        try:
            want = float(want)
        except (TypeError, ValueError):
            return _refuse("open_jaw needs 'mm' as a number")
        if not math.isfinite(want):
            return _refuse("open_jaw: mm is not finite")
        if want < closed:
            return _refuse("asking for %.1f mm when the jaws do not close below %.1f mm on this arm: "
                           "the object would be crushed before it was gripped" % (want, closed))
        pct = (want - closed) / mm_per_pct
        if pct > 50.0:
            return _refuse("asking for %.1f mm needs %.1f %% on arm %r, past the ~50 %% where the jaw "
                           "stops being a parallel gripper — a wider aperture is an angle, not a width"
                           % (want, pct, arm))
        return {"accepted": True, "reason": "ok",
                "action": {"kind": "jaw", "arm": arm, "pct": round(pct, 2), "achieved_mm": want}}

"""The command contract for one arm — the array that carries no names, no stamp and no owner.

`std_msgs/Float64MultiArray` arriving on `/armN/joint_targets` can carry five, six or seven
numbers and every one of them "works": `zip` in the old `/write` truncated in silence, so five
values left the jaw at its previous goal and seven dropped the last. Nothing raised, the arm
moved, and a short array from a policy or a replay read as a valid command.

This module is the single place where a command becomes valid or a sentence. It is stdlib only
and has no ROS import, so it runs in py3.8 (rclpy image) and py3.12 (LeRobot venv) alike and is
tested headless by `tools/test_command_contract.py`.

Two carriers, two encodings, one rule set:

  * the HTTP body (`POST /write`, JSON) — the joint server's boundary:
        {"angles": [6 floats]}                      all six joints driven
        {"angles": [5 floats], "hold": ["gripper"]} five driven, one named as untouched
    and a body that is five long with no `hold` is an error, not a convention: the number of
    values must equal the joints that are NOT declared held, positionally in JOINT_ORDER.
    JSON has no NaN, so "leave it alone" has to be said in words.

  * the ROS topic (Float64MultiArray, no header, no names) — the arbiter's boundary:
        exactly six floats; NaN is accepted ONLY for a joint listed in HOLDABLE (the jaw), the
        five arm joints must be finite. The arm joints position the tool; a NaN there is a
        message an operator meant to be something else.

Limits are part of the contract, not a comfort: the old code clamped anything to the range and
answered 200, so an out-of-range command moved the arm to the limit and looked accepted. Here it
is a rejection naming the joint and its range, with an explicit `clamp: true` for a caller that
really means "as far as it goes".
"""

import math

# Order of the array — fixed, and the only meaning it has. There are no names on the wire.
JOINT_ORDER = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")

# Same numbers as the joint server's backstop: ±170° of travel, jaw in the LeRobot 0-100 %.
LIMITS = {
    "shoulder_pan": (-170.0, 170.0),
    "shoulder_lift": (-170.0, 170.0),
    "elbow_flex": (-170.0, 170.0),
    "wrist_flex": (-170.0, 170.0),
    "wrist_roll": (-170.0, 170.0),
    "gripper": (0.0, 100.0),
}

# The only joint allowed to be "not driven": the jaw is not part of a pose sweep, and a hand-eye
# capture must not open or close it between frames.
HOLDABLE = ("gripper",)


class CommandError(ValueError):
    """A rejected command. The message is what the caller reads; name the joint and the range."""

    def __init__(self, message, code=400):
        super().__init__(message)
        self.code = code


def _finite(name, value):
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise CommandError(f"{name}: {value!r} is not a number")
    if not math.isfinite(v):
        raise CommandError(f"{name}: {v!r} is not finite")
    return v


def _in_range(name, value, clamp):
    lo, hi = LIMITS[name]
    if lo <= value <= hi:
        return value
    if not clamp:
        raise CommandError(f"{name}: {value:.1f} is outside {lo:.0f}..{hi:.0f} (send clamp: true to "
                           f"mean 'as far as it goes')")
    return lo if value < lo else hi


def validate_body(body, transform=None):
    """`POST /write` body -> {joint: value} for the joints to drive, plus the held names.

    `transform(name, value)` is applied to each value before the range check. The joint server uses it
    to turn a command given in *physical* degrees into the number the SDK wants (`joint_scale.py`), and
    because it happens before `_in_range` the limits keep their meaning: they are the fuse, and the
    value on the wire is what the fuse protects.

    Raises CommandError(code=400) for anything the caller can fix, so the joint server answers
    400 instead of the 500 an uncaught ValueError used to produce.
    """
    if not isinstance(body, dict):
        raise CommandError("body must be a JSON object with an 'angles' list")
    angles = body.get("angles")
    if not isinstance(angles, (list, tuple)):
        raise CommandError("body has no 'angles' list")

    hold = body.get("hold") or []
    if isinstance(hold, str):
        raise CommandError("'hold' must be a list of joint names, not a string")
    unknown = [n for n in hold if n not in JOINT_ORDER]
    if unknown:
        raise CommandError("hold names unknown joint(s) %s — one of %s" % (unknown, list(JOINT_ORDER)))
    not_holdable = [n for n in hold if n not in HOLDABLE]
    if not_holdable:
        # The jaw is the one joint a command can decline; the five arm joints define the pose.
        raise CommandError("cannot hold %s — only %s may be left undriven" % (not_holdable, list(HOLDABLE)))
    if len(set(hold)) != len(hold):
        raise CommandError("'hold' repeats a joint: %s" % hold)

    driving = [n for n in JOINT_ORDER if n not in hold]
    if len(angles) != len(driving):
        hint = ""
        if len(angles) == len(JOINT_ORDER) - 1 and not hold:
            hint = (" — five values used to mean 'leave the jaw alone' in silence; say it: "
                    "{'angles': [...5], 'hold': ['gripper']}")
        elif len(angles) > len(JOINT_ORDER):
            hint = " — this arm has six joints, there is no seventh motor to drive"
        raise CommandError("expected %d angles for %s, got %d%s"
                           % (len(driving), driving, len(angles), hint))

    clamp = bool(body.get("clamp"))
    out, notes = {}, []
    for name, raw in zip(driving, angles):
        value = _finite(name, raw)
        if transform is not None:
            sent = _finite(name, transform(name, value))
            if abs(sent - value) > 0.05:
                notes.append("%s %.1f physical -> %.1f sdk" % (name, value, sent))
            value = sent
        clipped = _in_range(name, value, clamp)
        if clipped != value:
            notes.append(f"{name} clamped {value:.1f} -> {clipped:.1f}")
        out[name] = clipped
    return out, set(hold), notes


def validate_topic(data):
    """Float64MultiArray.data -> {joint: value} for the joints to drive.

    No names and no stamp on this carrier, so the only in-band way to say "do not drive this one"
    is NaN — legal for the jaw, refused for the five arm joints.
    """
    if len(data) != len(JOINT_ORDER):
        raise CommandError("expected %d values (%s), got %d%s"
                           % (len(JOINT_ORDER), ", ".join(JOINT_ORDER), len(data),
                              " — NaN in the gripper slot means 'leave the jaw alone'"
                              if len(data) == len(JOINT_ORDER) - 1 else ""))
    out, held = {}, []
    for name, raw in zip(JOINT_ORDER, data):
        value = _finite(name, raw) if not (isinstance(raw, float) and math.isnan(raw)) else math.nan
        if math.isnan(value):
            if name not in HOLDABLE:
                raise CommandError(f"{name}: NaN — only {list(HOLDABLE)} may be left undriven")
            held.append(name)
            continue
        out[name] = _in_range(name, value, clamp=False)
    return out, set(held), []


def describe(values):
    """One line for a log: `shoulder_pan=12.0 …` — the numbers, never the raw list."""
    return ", ".join("%s=%.1f" % (n, values[n]) for n in JOINT_ORDER if n in values)

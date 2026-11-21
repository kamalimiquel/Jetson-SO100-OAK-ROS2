"""The measured scale of each joint: what a commanded degree actually turns.

The SDK reports `deg = (tick - mid) * 360 / 4095`, and that is a *constant*, not a calibration. On this
bench an instrument outside the robot says the constant is off by a different amount on every joint —
+5.9 % on `shoulder_pan`, -12 % on `wrist_roll` — so "90 degrees" moves the arm somewhere between 79 and
95 degrees depending on which joint hears it. Nothing raises: the number is finite, in range, and the
model agrees with itself.

So the numbers live here as data, next to the code, with the CSV each one came from, and the joint
server refuses to start with an incomplete or implausible table the same way it refuses a wrong unit
system (`units_manifest.py`). Applying them is opt-in (`JOINT_SCALE_APPLY=1`) because it changes what a
command means: with it on, `/write` takes **physical** degrees and divides by the scale before the packet.

Stdlib only, no ROS, no lerobot: `tools/test_joint_scale.py` runs it headless.

Definition, and the only one used here:

    physical = scale * sdk          sdk = physical / scale

`gripper` has `scale: null` on purpose. Its percentage is neither an angle nor a length: the aperture
curve lives in `docs/qualification/.../jaw_curve_arm1.csv` and is only meaningful below ~50 %, where the
jaw is still (roughly) parallel. Anything that converts it as if it were a rotation is lying.
"""

import hashlib
import json
import math
import os

# A joint whose degrees are wrong by more than half is not a calibration, it is a different mechanism.
SCALE_MIN, SCALE_MAX = 0.5, 1.5

# Same order and names as command_contract.JOINT_ORDER; kept here so this module stands alone in tests.
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")

DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "joint_scale.json")


class ScaleError(ValueError):
    """The table does not describe this cell. The message says which joint and what to fix."""


def sha256(path=DEFAULT_PATH):
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def load(path=DEFAULT_PATH):
    """Read and validate the table. Returns {"joints": {...}, "path": ..., "sha256": ...}."""
    try:
        with open(path, "r") as fh:
            raw = json.load(fh)
    except FileNotFoundError:
        raise ScaleError("no scale table at %s — the joints have no measured scale, so a command in "
                         "physical degrees would be a guess" % path)
    except ValueError as exc:
        raise ScaleError("%s is not valid JSON: %s" % (path, exc))

    if not isinstance(raw, dict) or not isinstance(raw.get("joints"), dict):
        raise ScaleError("%s has no 'joints' object" % path)
    joints = raw["joints"]

    missing = [j for j in JOINTS if j not in joints]
    if missing:
        raise ScaleError("%s has no entry for %s" % (path, ", ".join(missing)))
    extra = [j for j in joints if j not in JOINTS]
    if extra:
        raise ScaleError("%s names joints this arm does not have: %s" % (path, ", ".join(extra)))

    for name in JOINTS:
        entry = joints[name]
        if not isinstance(entry, dict):
            raise ScaleError("%s: %s must be an object with a 'scale'" % (path, name))
        scale = entry.get("scale")
        if name == "gripper":
            if scale is not None:
                raise ScaleError("%s: the gripper's percent is not an angle — set its scale to null "
                                 "and put the aperture curve in the qualification CSV" % path)
            continue
        if not isinstance(scale, (int, float)) or isinstance(scale, bool):
            raise ScaleError("%s: %s has no numeric 'scale'" % (path, name))
        if not math.isfinite(float(scale)) or not (SCALE_MIN <= float(scale) <= SCALE_MAX):
            raise ScaleError("%s: %s scale %r is outside %s..%s — that is a different mechanism, not a "
                             "calibration" % (path, name, scale, SCALE_MIN, SCALE_MAX))

    return {"joints": joints, "path": path, "sha256": sha256(path),
            "session": raw.get("session"), "evidence": raw.get("evidence"),
            "definition": raw.get("definition", "physical = scale * sdk")}


def to_sdk(table, joint, physical):
    """A number the caller means physically -> the number the SDK wants. Gripper: unchanged."""
    if joint not in JOINTS:
        raise ScaleError("unknown joint %r" % (joint,))
    entry = table["joints"][joint]
    if entry.get("scale") is None:
        return physical
    return physical / entry["scale"]


def to_physical(table, joint, sdk):
    """What the encoder's number is worth on the bench. Gripper: unchanged."""
    if joint not in JOINTS:
        raise ScaleError("unknown joint %r" % (joint,))
    entry = table["joints"][joint]
    if entry.get("scale") is None:
        return sdk
    return sdk * entry["scale"]


def describe(table):
    """One line for the boot log: the scale of every joint, and the hash of the table."""
    parts = []
    for name in JOINTS:
        entry = table["joints"][name]
        if entry.get("scale") is None:
            parts.append("%s=n/a" % name)
        else:
            parts.append("%s=%.3f" % (name, entry["scale"]))
    return "%s (%s, session %s)" % (", ".join(parts), table["sha256"][:12], table.get("session"))

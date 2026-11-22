#!/usr/bin/env python3
"""What was commanded, next to what happened — the arithmetic behind a commanded twin.

The twin used to be a mirror: it followed `/armN/joint_states` and nothing else, so it could show a lagging
arm but never say *how far behind*, nor separate "the servo cannot reach that" from "the command never
arrived". This module is the part that can be tested without a model, a GPU or a cell: it decodes a
command, pairs it with a reading, and produces the error.

Wire facts it encodes, because getting them wrong is silent:

  - `/joint_targets` is a `Float64MultiArray` with **no names**: position in the array is the contract, and
    the order is `JOINT_ORDER`. The jaw is optional — five values mean "these five, jaw held".
  - Both are in **degrees**, as published. The twin converts to radians only when it puts them in qpos.
  - A command older than `STALE_S` is not a command any more: reporting an error against a stale command
    invents a fault the operator then chases.

    from twin_commands import decode_targets, compare, JOINT_ORDER
    cmd = decode_targets(msg.get("data") or [])          # {"shoulder_pan": 12.0, ...}
    err = compare(cmd, {"shoulder_pan": 11.4, ...})      # {"shoulder_pan": 0.6, ...}
"""
from __future__ import annotations

import math
import time

JOINT_ORDER = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
STALE_S = 0.5
MAX_ABS_DEG = 180.0


def decode_targets(values, order=JOINT_ORDER):
    """Array → {joint: degrees}. Five values drive five joints and hold the jaw; six drive all six."""
    out = {}
    if not values:
        return out
    n = len(values)
    if n not in (5, 6):
        raise ValueError("joint_targets carried %d values; the contract is 5 (jaw held) or 6" % n)
    for i, v in enumerate(values):
        try:
            v = float(v)
        except (TypeError, ValueError):
            raise ValueError("joint_targets[%d] is not a number: %r" % (i, values[i]))
        if not math.isfinite(v) or abs(v) > MAX_ABS_DEG:
            raise ValueError("joint_targets[%d] = %r is outside +-%g deg" % (i, v, MAX_ABS_DEG))
        out[order[i]] = v
    return out


def compare(commanded, actual):
    """Per-joint error in degrees, for the joints both sides know. Never invents one."""
    err = {}
    for joint, want in commanded.items():
        got = actual.get(joint)
        if got is None:
            continue
        err[joint] = round(want - got, 3)
    return err


def fresh(stamp, now=None, limit=STALE_S):
    """A command is an intention while it is young; afterwards it is history."""
    now = time.time() if now is None else now
    return (now - stamp) <= limit


def should_report(sig, last_sig, last_print, now, period=2.0, change_period=0.5):
    """Print now if the command changed — but not more often than `change_period` — or to repeat it.

    Two publishers have to work with one rule. A `--once` command must appear at once, or it arrives and
    goes stale between two periodic looks and a working twin looks broken. A 20 Hz pad stream changes the
    command at every message, so "changed" alone would print five lines a second forever; the change is
    therefore rate-limited, while an unchanged command still repeats every `period`.
    """
    if sig != last_sig:
        return (now - last_print) >= change_period
    return (now - last_print) >= period


CSV_HEADER = "t,arm,joint,commanded_deg,actual_deg,err_deg"


def csv_rows(t, arm, err, commanded, actual):
    rows = []
    for joint in JOINT_ORDER:
        if joint not in err or joint not in commanded or joint not in actual:
            continue        # a row missing a side would be a half-truth; leave it out
        rows.append("%.3f,%s,%s,%.3f,%.3f,%.3f"
                    % (t, arm, joint, commanded[joint], actual[joint], err[joint]))
    return rows


if __name__ == "__main__":  # a quick smoke check when someone runs the file itself
    c = decode_targets([1.0, 2.0, 3.0, 4.0, 5.0])
    a = {"shoulder_pan": 0.5, "shoulder_lift": 2.5}
    print(c)
    print(compare(c, a))

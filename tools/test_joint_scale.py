#!/usr/bin/env python3
"""Tests for the measured joint scales. stdlib only, no ROS, no serial port.

Two kinds of check, and the second is the one that matters:

  * the table is well formed and the conversions round-trip;
  * **the numbers in the table still match the CSVs they claim to come from**. A scale that drifts
    away from its evidence is how a calibration becomes folklore.

Run: python3 tools/test_joint_scale.py
"""

import csv
import importlib.util
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
LIB = os.path.join(REPO, "docker", "ws", "lib")
EVIDENCE = os.path.join(REPO, "docs", "qualification", "baseline", "2026-11-19-6d63efc")

fails, checks = [], 0


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, os.path.join(LIB, filename))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


js = load("joint_scale", "joint_scale.py")
cc = load("command_contract", "command_contract.py")


def check(cond, label, detail=""):
    global checks
    checks += 1
    if not cond:
        fails.append("%s %s" % (label, ("— " + detail) if detail else ""))
        print("FAIL %s %s" % (label, detail))


def check_raises(fn, exc, label, must_say=None):
    try:
        fn()
    except exc as e:
        if must_say and must_say not in str(e):
            check(False, label, "message was %r" % str(e))
        else:
            check(True, label)
        return
    except Exception as e:  # noqa: BLE001
        check(False, label, "raised %s instead" % type(e).__name__)
        return
    check(False, label, "did not raise")


def slope_from_csv(path, zero_row=0):
    """Least-squares slope of external against sdk, measured from the first row of the series."""
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    sdk0 = float(rows[zero_row]["sdk_deg"])
    ext0 = float(rows[zero_row]["external_deg"])
    pts = [(float(r["sdk_deg"]) - sdk0, float(r["external_deg"]) - ext0)
           for r in rows if r.get("external_deg") not in (None, "")]
    n = len(pts)
    sx = sum(p[0] for p in pts)
    sy = sum(p[1] for p in pts)
    sxx = sum(p[0] * p[0] for p in pts)
    sxy = sum(p[0] * p[1] for p in pts)
    return (n * sxy - sx * sy) / (n * sxx - sx * sx), n


# ---------------------------------------------------------------- the table as shipped
table = js.load()
check(len(table["joints"]) == 6, "table has six joints")
check(len(table["sha256"]) == 64 and all(c in "0123456789abcdef" for c in table["sha256"]),
      "sha256 is 64 hex chars")
check(table["joints"]["gripper"]["scale"] is None, "gripper has no angle scale")
for name in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
    check(isinstance(table["joints"][name]["scale"], float), "%s has a numeric scale" % name)

# the conversions are inverse of each other
for name in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
    sdk = js.to_sdk(table, name, 42.0)
    back = js.to_physical(table, name, sdk)
    check(abs(back - 42.0) < 1e-9, "%s round-trips physical -> sdk -> physical" % name)
    check(abs(sdk - 42.0) > 1e-6, "%s actually converts (scale is not 1)" % name)

# the jaw is passed through untouched: a percentage is not an angle
check(js.to_sdk(table, "gripper", 40.0) == 40.0, "gripper physical->sdk is identity")
check(js.to_physical(table, "gripper", 40.0) == 40.0, "gripper sdk->physical is identity")

check_raises(lambda: js.to_sdk(table, "elbow", 10.0), js.ScaleError, "unknown joint is refused")
check_raises(lambda: js.to_physical(table, "elbow", 10.0), js.ScaleError, "unknown joint refused (physical)")

line = js.describe(table)
check("shoulder_pan=1.059" in line and "wrist_roll=0.877" in line and "gripper=n/a" in line,
      "describe() names every joint with its scale", line)

# ---------------------------------------------------------------- bad tables are refused
def write_table(joints, extra=None):
    obj = {"definition": "physical = scale * sdk", "session": "test", "joints": joints}
    obj.update(extra or {})
    fd, path = tempfile.mkstemp(suffix=".json")
    with os.fdopen(fd, "w") as fh:
        json.dump(obj, fh)
    return path


good = {n: {"scale": 1.0} for n in js.JOINTS if n != "gripper"}
good["gripper"] = {"scale": None}

p = write_table(dict(good))
check(js.load(p)["joints"]["shoulder_pan"]["scale"] == 1.0, "a table of 1.0 loads")

bad = dict(good, shoulder_pan={"scale": 3.0})
p = write_table(bad)
check_raises(lambda: js.load(p), js.ScaleError, "a scale of 3.0 is refused (different mechanism)",
             must_say="different mechanism")

bad = dict(good, shoulder_pan={"scale": 0.1})
p = write_table(bad)
check_raises(lambda: js.load(p), js.ScaleError, "a scale of 0.1 is refused", must_say="different mechanism")

bad = {k: v for k, v in good.items() if k != "wrist_roll"}
p = write_table(bad)
check_raises(lambda: js.load(p), js.ScaleError, "a missing joint is refused", must_say="wrist_roll")

bad = dict(good, elbow_flex={"scale": "0.98"})
p = write_table(bad)
check_raises(lambda: js.load(p), js.ScaleError, "a scale given as a string is refused", must_say="numeric")

bad = dict(good, gripper={"scale": 1.0})
p = write_table(bad)
check_raises(lambda: js.load(p), js.ScaleError, "a numeric gripper scale is refused",
             must_say="not an angle")

bad = dict(good, extra_joint={"scale": 1.0})
p = write_table(bad)
check_raises(lambda: js.load(p), js.ScaleError, "a joint this arm does not have is refused",
             must_say="does not have")

check_raises(lambda: js.load("/nonexistent/joint_scale.json"), js.ScaleError,
             "a missing table is a sentence, not a traceback", must_say="no scale table")

# ---------------------------------------------------------------- the contract uses it
body = {"angles": [95.0, 0.0, 0.0, 0.0, 0.0, 0.0], "units": "physical"}
driven, held, notes = cc.validate_body(body, transform=lambda n, v: js.to_sdk(table, n, v))
check(abs(driven["shoulder_pan"] - 95.0 / 1.059) < 1e-6, "physical 95 on the pan becomes ~89.7 sdk",
      str(driven["shoulder_pan"]))
check(any("shoulder_pan 95.0 physical -> 89.7 sdk" in n for n in notes), "the conversion is reported", str(notes))

# the limits still bite after the conversion: 190 physical is 179 sdk, outside the fuse
try:
    cc.validate_body({"angles": [190.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                     transform=lambda n, v: js.to_sdk(table, n, v))
    check(False, "a physical command beyond the fuse is refused")
except cc.CommandError as e:
    check("outside" in str(e) and "clamp" in str(e), "a physical command beyond the fuse is refused", str(e))

# and without a transform nothing changes: SDK degrees in, SDK degrees out
driven, _, notes = cc.validate_body({"angles": [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]})
check(driven["shoulder_pan"] == 10.0 and not notes, "without a transform the numbers pass through")

# ---------------------------------------------------------------- the table still matches its evidence
for joint, csv_name, zero_row in (("shoulder_pan", "ref_shoulder_pan_arm1.csv", 0),
                                  ("wrist_roll", "ref_wrist_roll_arm1.csv", 0)):
    path = os.path.join(EVIDENCE, csv_name)
    if not os.path.exists(path):
        check(False, "%s: evidence %s exists" % (joint, csv_name))
        continue
    slope, n = slope_from_csv(path, zero_row)
    declared = table["joints"][joint]["scale"]
    check(abs(slope - declared) <= 0.02,
          "%s: the table matches its CSV" % joint,
          "csv says %.3f over %d points, table says %.3f" % (slope, n, declared))

check(os.path.isdir(os.path.join(REPO, table["evidence"].rstrip("/"))),
      "the evidence directory named in the table exists")

print("%d checks, %d failed" % (checks, len(fails)))
if fails:
    for f in fails:
        print("  - " + f)
    sys.exit(1)

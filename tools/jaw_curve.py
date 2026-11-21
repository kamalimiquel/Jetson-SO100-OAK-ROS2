#!/usr/bin/env python3
"""Sweep the jaw open and closed, recording ticks, SDK percent and a blank for the caliper.

The jaw is the one joint whose value is not an angle: the driver reports 0–100 % and `deg2rad`
turns it into radians with a constant that is explicitly a drawing, not a measurement. To replace
that constant you need the curve — several apertures in both directions, each one measured with the
jaws at the grasp plane — because the mechanism is not linear and its hysteresis is not zero.

This tool does the moving and the recording; the operator supplies the millimetres.

    python3 tools/jaw_curve.py --host 192.168.1.185 --port 8002 \
        --steps 0 10 20 30 40 50 60 70 80 90 100 90 80 70 60 50 40 30 20 10 0 \
        --settle 1.5 --yes --out jaw_arm1.csv

Safety: it refuses without --yes, it arms torque itself and always disarms in a finally block, it
moves only the jaw (the five body joints are commanded to where they already are), and it stops the
sweep on the first unexpected answer from the server.
"""

import argparse
import csv
import json
import sys
import time
import urllib.error
import urllib.request

BODY = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
NAMES = BODY + ["gripper"]
COLUMNS = ["step", "direction", "target_pct", "raw_ticks", "sdk_pct", "caliper_mm", "note"]


def call(host, port, path, body=None, timeout=4.0):
    req = urllib.request.Request("http://%s:%d%s" % (host, port, path), method="POST" if body is not None else "GET",
                                 data=json.dumps(body).encode() if body is not None else None,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")


def present(host, port):
    _, raw = call(host, port, "/read_raw")
    return raw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.1.185")
    ap.add_argument("--port", type=int, default=8002)
    ap.add_argument("--steps", type=float, nargs="+",
                    default=[0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0])
    ap.add_argument("--settle", type=float, default=1.5)
    ap.add_argument("--out", required=True)
    ap.add_argument("--yes", action="store_true", help="the operator is at the bench and the jaw is clear")
    args = ap.parse_args()

    if not args.yes:
        print("refusing to move the jaw without --yes (the operator must be at the bench)", file=sys.stderr)
        return 2

    code, before = call(args.host, args.port, "/read_raw")
    if code != 200:
        print("cannot read the arm: %s" % before, file=sys.stderr)
        return 1
    if before.get("torque"):
        print("torque is already ON; this tool arms and disarms itself, so release it first",
              file=sys.stderr)
        return 3

    rows = []
    armed = False
    try:
        code, answer = call(args.host, args.port, "/torque", {"on": True})
        if code != 200:
            print("arming failed: %s" % answer, file=sys.stderr)
            return 4
        armed = True
        # the five body joints stay where they are, sampled once per step from the arm itself
        direction = "open"
        for step, target in enumerate(args.steps):
            if step and target < args.steps[step - 1]:
                direction = "close"
            elif step and target > args.steps[step - 1]:
                direction = "open"
            now = present(args.host, args.port)
            angles = [round(now["normalized"][n], 2) for n in BODY] + [float(target)]
            code, answer = call(args.host, args.port, "/write", {"angles": angles})
            if code != 200:
                print("write refused at %s%%: %s — stopping the sweep" % (target, answer), file=sys.stderr)
                break
            time.sleep(args.settle)
            after = present(args.host, args.port)
            rows.append({"step": step, "direction": direction, "target_pct": target,
                         "raw_ticks": after["ticks"]["gripper"],
                         "sdk_pct": round(float(after["normalized"]["gripper"]), 2),
                         "caliper_mm": "", "note": ""})
            print("%3d  %-5s target %5.1f%%  ticks %5s  sdk %5.2f%%" %
                  (step, direction, target, after["ticks"]["gripper"],
                   float(after["normalized"]["gripper"])))
    finally:
        if armed:
            call(args.host, args.port, "/torque", {"on": False})

    with open(args.out, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    print("wrote %s (%d rows). Measure the aperture at each step in the same order and put the "
          "millimetres in caliper_mm — the curve, not the endpoint, is what calibrates the map."
          % (args.out, len(rows)))
    return 0


if __name__ == "__main__":
    sys.exit(main())

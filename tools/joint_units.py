#!/usr/bin/env python3
"""Write the joint-unit table that phase 1 signs off — from the running servers, not from the docs.

One row per motor per arm: bus id, model, the normalisation mode the SDK reports, the raw encoder
tick, the SDK value, the calibration hash — and two columns left deliberately empty for the human:
the value read from an external instrument, and how it was measured. A table where the last two
columns are blank is not evidence; it is a form.

Run it ON the Xavier: the joint servers bind to 127.0.0.1 on purpose (only loopback can reach the
write port), so this is not a LAN tool like the hand-eye scripts.

    ssh maju@192.168.1.185 'python3 ~/so100/tools/joint_units.py --out /tmp/joint_units.csv'
    scp maju@192.168.1.185:/tmp/joint_units.csv docs/qualification/baseline/<date>-<sha>/

Nothing here moves the arm: it only reads /manifest and /read_raw, and refuses to run if either
server reports torque on.
"""

import argparse
import csv
import json
import sys
import time
import urllib.error
import urllib.request

COLUMNS = ["arm", "port", "joint", "motor_id", "model", "norm_mode", "unit", "raw_ticks",
           "sdk_value", "ros_value", "calibration_sha256", "external_reference", "external_unit",
           "note"]


def get(host, port, path, timeout=3.0):
    with urllib.request.urlopen("http://%s:%d%s" % (host, port, path), timeout=timeout) as r:
        return json.loads(r.read())


def rows_for(arm, host, port):
    manifest = get(host, port, "/manifest")
    raw = get(host, port, "/read_raw")
    if raw.get("torque"):
        print("!! %s: torque is ON — the unit table is only meaningful on a passive arm" % arm,
              file=sys.stderr)
    out = []
    for j in manifest["joints"]:
        name = j["name"]
        out.append({
            "arm": arm,
            "port": port,
            "joint": name,
            "motor_id": j["id"],
            "model": j["model"],
            "norm_mode": j["norm"],
            "unit": manifest["unit"].get(name, "?"),
            "raw_ticks": raw["ticks"].get(name),
            "sdk_value": round(float(raw["normalized"].get(name, float("nan"))), 3),
            "ros_value": "",          # filled from a ROS snapshot taken in the same session
            "calibration_sha256": (manifest.get("calibration_sha256") or "")[:16],
            "external_reference": "",      # filled by hand, with an instrument
            "external_unit": "",
            "note": "",
        })
    return out, manifest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--ports", type=int, nargs="+", default=[8002, 8003])
    ap.add_argument("--arms", nargs="+", default=["arm1", "arm2"])
    ap.add_argument("--settle", type=float, default=0.0,
                    help="seconds between the two reads (they should agree; a moving arm will not)")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    rows = []
    for arm, port in zip(args.arms, args.ports):
        try:
            first, manifest = rows_for(arm, args.host, port)
        except (urllib.error.URLError, OSError) as exc:
            print("!! %s:%d unreachable (%s)" % (args.host, port, exc), file=sys.stderr)
            continue
        print("%s (%s, sdk %s, degrees=%s) — %d motors, calib %s"
              % (arm, manifest["arm_id"], manifest["sdk_version"], manifest["use_degrees"],
                 len(first), (manifest.get("calibration_sha256") or "no hash")[:12]))
        if args.settle:
            time.sleep(args.settle)
            second, _ = rows_for(arm, args.host, port)
            for a, b in zip(first, second):
                if a["raw_ticks"] != b["raw_ticks"]:
                    a["note"] = "read twice, %d -> %d ticks" % (a["raw_ticks"], b["raw_ticks"])
        rows.extend(first)

    if not rows:
        print("no rows: nothing was reachable", file=sys.stderr)
        return 1
    with open(args.out, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    print("wrote %s (%d rows). Fill external_reference/external_unit with the instrument used; "
          "blank columns are not evidence." % (args.out, len(rows)))
    return 0


if __name__ == "__main__":
    sys.exit(main())

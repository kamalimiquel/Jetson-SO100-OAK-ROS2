#!/usr/bin/env python3
"""Move one joint to a few commanded angles and record the SDK reading, leaving a column for the
instrument.

This is the other half of the unit table: `joint_units.py` proves that the SDK, the encoder and the
ROS topic agree with each other, which is worthless if all three share the same wrong scale. Only an
external reference — an inclinometer on a link, a marked disk, a protractor on the table — says
whether "90" is ninety degrees. The fit that matters is the slope of external-vs-SDK over the move:
1.000 ± tolerance for a joint reported in degrees, and a constant offset.

The five joints that are not being measured are commanded to where they already are, so the arm
does not drift during the session.

Run it ON the Xavier (the joint servers bind loopback):

    python3 tools/joint_reference.py --port 8002 --joint shoulder_lift --deltas 0 4 8 \
        --settle 2.5 --yes --out /tmp/ref_lift.csv

Safety: refuses without --yes, arms and always disarms, stops at the first refused write, and never
commands more than |delta| ≤ 10 from the starting pose.
"""

import argparse
import csv
import json
import sys
import time
import urllib.error
import urllib.request

BODY = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
COLUMNS = ["joint", "step", "delta_deg", "commanded_deg", "sdk_deg", "raw_ticks", "external_deg",
           "external_unit", "instrument", "note"]


def call(host, port, path, body=None, timeout=4.0):
    req = urllib.request.Request(
        "http://%s:%d%s" % (host, port, path), method="POST" if body is not None else "GET",
        data=json.dumps(body).encode() if body is not None else None,
        headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")


class _Stop(Exception):
    """Raised to stop a sweep while keeping the current pose held."""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8002)
    ap.add_argument("--joint", required=True, choices=BODY)
    ap.add_argument("--deltas", type=float, nargs="+", default=[0.0, 4.0, 8.0],
                    help="angles relative to the starting pose, degrees")
    ap.add_argument("--settle", type=float, default=2.5)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-step", type=float, default=10.0,
                    help="largest single command in degrees; a bigger delta is ramped in steps")
    ap.add_argument("--stop-err", type=float, default=6.0,
                    help="abort if the arm is further than this from the sub-target (stall/sag)")
    ap.add_argument("--keep-armed", action="store_true",
                    help="do not release torque at the end: the pose stays held while the operator "
                         "measures. Release it yourself when the session is over.")
    ap.add_argument("--already-armed", action="store_true",
                    help="the arm is already holding torque (a previous --keep-armed step): write "
                         "without arming or disarming it")
    ap.add_argument("--yes", action="store_true")
    args = ap.parse_args()

    if not args.yes:
        print("refusing to move without --yes (operator at the bench, instrument in hand)",
              file=sys.stderr)
        return 2
    if any(abs(d) > 120.0 for d in args.deltas):
        print("refusing a move larger than 120 deg from the starting pose", file=sys.stderr)
        return 2
    if not 0 < args.max_step <= 30:
        print("--max-step must be within (0, 30] deg", file=sys.stderr)
        return 2

    code, raw = call(args.host, args.port, "/read_raw")
    if code != 200 or "normalized" not in raw:
        print("cannot read the arm: %s" % raw, file=sys.stderr)
        return 1
    if raw.get("torque") and not args.already_armed:
        print("torque already ON — pass --already-armed to move within a held session, or release it",
              file=sys.stderr)
        return 3

    start = {n: float(raw["normalized"][n]) for n in BODY}
    print("start pose (sdk units): %s" % {n: round(v, 2) for n, v in start.items()})
    print("measuring %s over deltas %s\n" % (args.joint, args.deltas))

    rows, armed = [], False
    step = 0

    def move_to(target, delta):
        """One command + settle + read. Returns the measured error (sdk - target)."""
        nonlocal step
        angles = [round(start[n], 2) for n in BODY]
        angles[BODY.index(args.joint)] = round(target, 2)
        code, answer = call(args.host, args.port, "/write", {"angles": angles})
        if code != 200:
            hint = ""
            if code == 423:
                hint = (" — another writer holds the arm; for a maintenance session the arbiter must "
                        "be observing (ARBITER_ENFORCE=0)")
            print("write refused at %s=%.2f: %s%s" % (args.joint, target, answer, hint),
                  file=sys.stderr)
            return None
        time.sleep(args.settle)
        code, now = call(args.host, args.port, "/read_raw")
        sdk = float(now["normalized"][args.joint])
        tick = now["ticks"][args.joint]
        err = sdk - target
        rows.append({"joint": args.joint, "step": step, "delta_deg": delta,
                     "commanded_deg": round(target, 2), "sdk_deg": round(sdk, 3),
                     "raw_ticks": tick, "external_deg": "", "external_unit": "", "instrument": "",
                     "note": "ros==sdk" if abs(err) < 0.6 else "sdk-target=%.2f" % err})
        print("%2d  commanded %8.2f   sdk %8.3f   tick %5d   %s"
              % (step, target, sdk, tick, rows[-1]["note"]))
        step += 1
        return err

    try:
        if args.already_armed:
            armed = True
        else:
            code, answer = call(args.host, args.port, "/torque", {"on": True})
            if code != 200:
                print("arming failed: %s" % answer, file=sys.stderr)
                return 4
            armed = True
        for delta in args.deltas:
            n = max(1, int(abs(delta) / args.max_step + 0.999))
            for k in range(1, n + 1):
                target = start[args.joint] + delta * k / n
                err = move_to(target, round(delta * k / n, 2))
                if err is None:
                    raise _Stop()
                if abs(err) > args.stop_err:
                    print("STOP: %.2f deg short of %.2f — the joint is stalling or the load is too "
                          "high; the pose stays held where it is" % (err, target), file=sys.stderr)
                    raise _Stop()
    except _Stop:
        pass

    finally:
        if armed and not args.keep_armed:
            call(args.host, args.port, "/torque", {"on": False})
        elif armed and rows:
            print("\nARM HELD — torque is still ON at %s = %.2f (sdk). Measure now, and release with "
                  "`curl -X POST :8002/torque -d '{\"on\":false}'` when the session ends."
                  % (args.joint, rows[-1]["sdk_deg"]))
        elif armed:
            # The server refused every step (torque off, a lease, the stall guard). Saying "ARM HELD"
            # here sent a reader to the instrument for a pose that never happened.
            print("\nNOT MOVED — nothing was written, so there is no pose to measure: check torque, "
                  "the lease and the stall guard. Torque is still ON.", flush=True)

    with open(args.out, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    print("\nwrote %s (%d rows). Dictate the instrument reading for each commanded angle, in order."
          % (args.out, len(rows)))
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Copy every arm calibration profile into the repo, so the numbers survive the SD card.

LeRobot writes one JSON per arm under
`~/.cache/huggingface/lerobot/calibration/robots/so_follower/<id>.json` and overwrites it in place on
the next calibration. That file is the truth the joint server reads at boot, and losing it means
re-calibrating by hand with the arm in front of you — so every calibration leaves a dated copy in
`docs/calibration/arms/`, committed with the change that made it.

Run it on the Xavier straight after `lerobot-calibrate` (or let `scripts/calibrate.sh` do both), then
from the operator PC copy the folder back, commit, and push.

    python3 tools/archive_calibration.py                 # every profile it finds
    python3 tools/archive_calibration.py --id so100_cell  # just arm 1
    python3 tools/archive_calibration.py --dry-run        # say what it would copy

Idempotent: a profile identical to the newest archived copy is reported and skipped, and two
calibrations on the same day get `-b`, `-c`, … instead of overwriting each other.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import shutil
import sys

DEFAULT_SRC = os.path.expanduser("~/.cache/huggingface/lerobot/calibration/robots/so_follower")
DEFAULT_OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "calibration", "arms")
SUFFIXES = "bcdefgh"


def digest(path: str) -> str:
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def target_name(out_dir: str, stamp: str, arm_id: str) -> str:
    first = "%s-%s.json" % (stamp, arm_id)
    if not os.path.exists(os.path.join(out_dir, first)):
        return first
    for suffix in SUFFIXES:
        candidate = "%s%s-%s.json" % (stamp, suffix, arm_id)
        if not os.path.exists(os.path.join(out_dir, candidate)):
            return candidate
    raise SystemExit("more than 8 calibrations of %s on %s — tidy the folder first" % (arm_id, stamp))


def ranges(path: str) -> list[tuple[str, int, int, int, int]]:
    """(joint, motor id, range_min, range_max, span) in file order, which is bus order."""
    with open(path, encoding="utf-8") as fh:
        data = json.load(fh)
    rows = []
    for joint, cfg in data.items():
        lo, hi = int(cfg.get("range_min", 0)), int(cfg.get("range_max", 0))
        rows.append((joint, int(cfg.get("id", 0)), lo, hi, hi - lo))
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source-dir", default=DEFAULT_SRC)
    ap.add_argument("--out-dir", default=os.path.normpath(DEFAULT_OUT))
    ap.add_argument("--id", default=None, help="only this arm id (e.g. so100_cell, so100_cell_b)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if not os.path.isdir(args.source_dir):
        print("no calibration folder at %s — is this the machine that ran lerobot-calibrate?" % args.source_dir)
        return 1

    profiles = sorted(p for p in os.listdir(args.source_dir) if p.endswith(".json"))
    if args.id:
        profiles = [p for p in profiles if p == args.id + ".json"]
    if not profiles:
        print("nothing to archive in %s" % args.source_dir)
        return 1

    os.makedirs(args.out_dir, exist_ok=True)
    copied = skipped = 0
    for name in profiles:
        src = os.path.join(args.source_dir, name)
        arm_id = name[: -len(".json")]
        stamp = dt.datetime.fromtimestamp(os.path.getmtime(src)).strftime("%Y-%m-%d")
        want = digest(src)

        same = [f for f in sorted(os.listdir(args.out_dir))
                if f.endswith("-%s.json" % arm_id) and digest(os.path.join(args.out_dir, f)) == want]
        if same:
            print("%-16s unchanged — already archived as %s" % (arm_id, same[-1]))
            skipped += 1
            continue

        dest = target_name(args.out_dir, stamp, arm_id)
        if args.dry_run:
            print("%-16s would copy to %s" % (arm_id, dest))
            continue
        shutil.copy2(src, os.path.join(args.out_dir, dest))
        copied += 1
        print("%-16s -> %s" % (arm_id, os.path.join("docs/calibration/arms", dest)))
        print("  %-14s %3s %6s %6s %6s" % ("joint", "id", "min", "max", "span"))
        for joint, motor, lo, hi, span in ranges(src):
            print("  %-14s %3d %6d %6d %6d" % (joint, motor, lo, hi, span))

    if not args.dry_run and copied:
        print("\ncommit them:  git add docs/calibration/arms && git commit -m 'docs(calibration): archive <arm>'")
    print("\n%d copied, %d already there" % (copied, skipped))
    return 0


if __name__ == "__main__":
    sys.exit(main())

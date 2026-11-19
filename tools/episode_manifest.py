#!/usr/bin/env python3
"""Rebuild an episode's manifest from its own logs — for an episode that was killed.

An episode is `observations.jsonl` + `commands.jsonl` + `frames/`; the manifest is a summary of
those. When the summary is missing (a hard kill, a disk that filled, a copied directory), nothing is
lost: every number the manifest carries is derivable from the lines. This writes it back.

    python3 tools/episode_manifest.py /episodes/20261119-194107          # write manifest.json
    python3 tools/episode_manifest.py /episodes/20261119-194107 --show   # print, do not write
"""

import argparse
import json
import os
import sys

STALE_S = 0.5          # same window as episode_log


def rebuild(directory, stale_s=STALE_S):
    obs_path = os.path.join(directory, "observations.jsonl")
    cmd_path = os.path.join(directory, "commands.jsonl")
    observations, stale, max_age = {}, {}, {}
    first_mono = last_mono = None
    frames = 0
    with open(obs_path, encoding="utf-8") as fh:
        for line in fh:
            row = json.loads(line)
            topic = row["topic"]
            observations[topic] = observations.get(topic, 0) + 1
            if row.get("file"):
                frames += 1
            mono = row.get("mono")
            if mono is not None:
                first_mono = mono if first_mono is None else min(first_mono, mono)
                last_mono = mono if last_mono is None else max(last_mono, mono)
            age = row.get("age_s")
            if age is not None:
                if age > stale_s:
                    stale[topic] = stale.get(topic, 0) + 1
                if age > max_age.get(topic, -1.0):
                    max_age[topic] = age

    manifest = {
        "episode": os.path.basename(os.path.normpath(directory)),
        "rebuilt": True,
        "note": "manifest rebuilt from observations.jsonl: ended/duration are the log's span, and "
                "whatever the killed process never flushed is simply absent",
        "observations": observations,
        "stale": stale,
        "max_age_s": max_age,
        "frames": frames,
        "span_s": round(last_mono - first_mono, 3) if first_mono is not None else None,
        "commands": 0,
        "command_rows": 0,
    }
    if os.path.exists(cmd_path):
        with open(cmd_path, encoding="utf-8") as fh:
            for line in fh:
                row = json.loads(line)
                manifest["command_rows"] += 1
                if row.get("accepted"):
                    manifest["commands"] += 1
    return manifest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("directory")
    ap.add_argument("--show", action="store_true", help="print instead of writing manifest.json")
    args = ap.parse_args()
    if not os.path.exists(os.path.join(args.directory, "observations.jsonl")):
        print("no observations.jsonl in %s — is that an episode directory?" % args.directory)
        return 1
    manifest = rebuild(args.directory)
    text = json.dumps(manifest, indent=2, sort_keys=True)
    if args.show:
        print(text)
        return 0
    with open(os.path.join(args.directory, "manifest.json"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    print("wrote %s/manifest.json" % args.directory)
    return 0


if __name__ == "__main__":
    sys.exit(main())

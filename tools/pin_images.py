#!/usr/bin/env python3
"""Write the image lock: the digest of every image the cell runs, so a run can be reproduced by tag+digest.

Chapter 2 asks to "lock images by digest and record device mappings". The compose file names tags, and a
tag is a moveable pointer: `dustynv/ros:humble-ros-base-l4t-r35.4.1` today is not necessarily the bytes
that ran last week. The lock file records what was actually resolved, next to the device the container was
given.

    python3 tools/pin_images.py --host 192.168.1.185        # writes docs/qualification/images.lock
    python3 tools/pin_images.py --host ... --check         # fails if the live digests differ from the lock

On the Jetson (no docker python SDK needed): it shells out to `docker inspect`.
"""

import argparse
import json
import os
import subprocess
import sys

SERVICES = ("so100-arm", "so100-arm2", "so100-cams", "so100-perception", "so100-control",
            "so100-web", "so100-rosbridge")


def docker(host, *args):
    cmd = ["docker"] + list(args)
    if host:
        cmd = ["ssh", host] + cmd
    return subprocess.run(cmd, capture_output=True, text=True, check=True).stdout


def snapshot(host):
    out = {}
    for name in SERVICES:
        try:
            raw = docker(host, "inspect", name)
        except subprocess.CalledProcessError:
            out[name] = {"present": False}
            continue
        info = json.loads(raw)[0]
        devices = []
        for d in info.get("HostConfig", {}).get("Devices") or []:
            devices.append({"host": d.get("PathOnHost"), "container": d.get("PathInContainer")})
        out[name] = {
            "present": True,
            "image": info["Config"]["Image"],
            "image_id": info.get("Image"),                       # sha256:… of the resolved image
            "repo_digests": info.get("RepoDigests") or [],       # empty for locally built images
            "devices": sorted(devices, key=lambda x: x["container"] or ""),
            "created": info.get("Created"),
        }
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="", help="ssh host to ask (empty = this machine)")
    ap.add_argument("--out", default=os.path.join("docs", "qualification", "images.lock"))
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args(argv)

    live = snapshot(args.host)

    if args.check:
        if not os.path.exists(args.out):
            print("no lock at %s — write one with --host" % args.out)
            return 1
        locked = json.load(open(args.out))
        bad = []
        for name, entry in live.items():
            old = locked.get("services", {}).get(name, {})
            if entry.get("image_id") != old.get("image_id"):
                bad.append("%s: %s -> %s" % (name, str(old.get("image_id"))[:19],
                                             str(entry.get("image_id"))[:19]))
        for line in bad:
            print("  MOVED " + line)
        print("%d service(s) changed since the lock" % len(bad))
        return 1 if bad else 0

    payload = {"note": "Written by tools/pin_images.py: what was actually resolved, not what compose says",
               "services": live}
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True)
        fh.write("\n")
    missing = [n for n, e in live.items() if not e.get("present")]
    print("wrote %s (%d services, %d not running%s)"
          % (args.out, len(live), len(missing), ": " + ", ".join(missing) if missing else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())

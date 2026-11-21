#!/usr/bin/env python3
"""Every stored transform has to say *which* camera and *which* mount produced it.

Chapter 2 of the MCP spec asks for one thing among many: "version transforms with camera and mount IDs".
The reason is cheap to see in this repo — `docs/calibration/scene_cam_extrinsic.yaml` is a `base <- scene_cam`
matrix with no camera serial, no mount, no date and no dataset, so nothing in the file says whether it
belongs to the OAK that is bolted there today. A transform that cannot be tied to hardware is a drawing
with six decimals.

A file passes if it carries, at top level:

    parent, child, translation, rotation_row0..2
    camera_serial, camera_model, mount_id, method, date, solver

and, unless its first lines say `NOT USABLE`, a residual (`residual_px` or `residual_mm`). A file marked
NOT USABLE is allowed to be incomplete — that is what the marker is for — but it must say why, so nobody
mistakes silence for a measurement.

    python3 tools/check_transforms.py                 # check docs/calibration/*.yaml
    python3 tools/check_transforms.py path/to/dir

Exit 0 when every transform is either versioned or explicitly retired.
"""

import glob
import os
import re
import sys

REQUIRED = ("parent", "child", "translation", "rotation_row0", "rotation_row1", "rotation_row2",
            "camera_serial", "camera_model", "mount_id", "method", "date", "solver")
RESIDUAL = ("residual_px", "residual_mm", "holdout_mm")


def has_key(text, key):
    return re.search(r"(?m)^%s[ \t]*:" % re.escape(key), text) is not None


def value_of(text, key):
    m = re.search(r"(?m)^%s[ \t]*:[ \t]*(\S.*)$" % re.escape(key), text)
    return m.group(1).strip().strip('"\'') if m else ""


def retired(text):
    head = "\n".join(text.splitlines()[:12])
    return "NOT USABLE" in head


def check_file(path):
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    name = os.path.basename(path)
    if retired(text):
        # retired files still have to say why, or the marker means nothing
        head = "\n".join(text.splitlines()[:12])
        reason = len([l for l in head.splitlines() if l.strip().startswith("#")]) >= 2
        return ("skip", "%s: retired%s" % (name, "" if reason else " WITHOUT A REASON — say why"))
    if not (has_key(text, "parent") or has_key(text, "rotation_row0")):
        # an intrinsics or registration file, not a pose: it has no parent/child to version
        return ("skip", "%s: not a transform (no parent / rotation_row0)" % name)
    missing = [k for k in REQUIRED if not has_key(text, k)]
    if missing:
        return ("fail", "%s: no %s" % (name, ", ".join(missing)))
    empty = [k for k in ("camera_serial", "mount_id", "method", "date") if not value_of(text, k)]
    if empty:
        return ("fail", "%s: %s present but empty" % (name, ", ".join(empty)))
    if not any(has_key(text, k) for k in RESIDUAL):
        return ("fail", "%s: no residual (%s) — an unversioned accuracy is not evidence"
                % (name, " / ".join(RESIDUAL)))
    return ("ok", "%s: %s on mount %s (%s)" % (name, value_of(text, "camera_serial"),
                                               value_of(text, "mount_id"), value_of(text, "date")))


def main(argv):
    target = argv[1] if len(argv) > 1 else os.path.join("docs", "calibration")
    files = sorted(glob.glob(os.path.join(target, "*.yaml")))
    if not files:
        print("no .yaml under %s" % target)
        return 0 if target.startswith("docs/calibration") is False else 1
    bad = 0
    for path in files:
        status, line = check_file(path)
        print("  %-4s %s" % (status, line))
        if status == "fail":
            bad += 1
    print("%d transform(s), %d failing" % (len(files), bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

#!/usr/bin/env python3
"""Headless test of tools/check_transforms.py — chapter 2's "version transforms with camera and mount IDs".

Four cases, one per way this can go wrong: a transform that is versioned, one that looks complete but
cannot be tied to hardware, one retired on purpose, and a file that is not a transform at all.

    python3 tools/test_check_transforms.py
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_transforms as ct   # noqa: E402

fails = []


def check(name, got, want):
    ok = got == want
    print(("  ok   " if ok else "  FAIL ") + "%s: %s" % (name, got) + ("" if ok else " (expected %s)" % (want,)))
    if not ok:
        fails.append(name)


GOOD = """# base <- scene_cam
parent: base
child: scene_cam_link
translation: [0.1, -0.2, 0.3]
rotation_row0: [1.0, 0.0, 0.0]
rotation_row1: [0.0, 1.0, 0.0]
rotation_row2: [0.0, 0.0, 1.0]
camera_serial: OAK-1944301011B0C600
camera_model: OAK-D-W
mount_id: bracket-A-2026-11-21
method: eye-to-hand, board clamped to the jaw
date: 2026-11-21
solver: tools/handeye_bundle.py
residual_px: 0.41
"""

NO_SERIAL = GOOD.replace("camera_serial: OAK-1944301011B0C600\n", "")
EMPTY_MOUNT = GOOD.replace("mount_id: bracket-A-2026-11-21", "mount_id: ")
NO_RESIDUAL = "\n".join(l for l in GOOD.splitlines() if not l.startswith("residual_"))
RETIRED = """# !! NOT USABLE — kept only as a record !!
# the dataset does not close; re-solve with the board clamped
parent: base
child: scene_cam_link
translation: [0.1, -0.2, 0.3]
"""
INTRINSICS = """# camera matrix
fx: 700.0
fy: 700.0
cx: 320.0
cy: 240.0
"""


def run(text, name="x.yaml"):
    d = tempfile.mkdtemp()
    p = os.path.join(d, name)
    with open(p, "w") as fh:
        fh.write(text)
    return ct.check_file(p)


check("a versioned transform passes", run(GOOD)[0], "ok")
check("and names the camera and mount", "OAK-1944301011B0C600" in run(GOOD)[1], True)
check("no camera serial fails", run(NO_SERIAL)[0], "fail")
check("the message says what is missing", "camera_serial" in run(NO_SERIAL)[1], True)
check("an empty mount id fails", run(EMPTY_MOUNT)[0], "fail")
check("an empty field is not a version", "empty" in run(EMPTY_MOUNT)[1], True)
check("no residual fails", run(NO_RESIDUAL)[0], "fail")
check("and says why that matters", "not evidence" in run(NO_RESIDUAL)[1], True)
check("a retired transform is skipped", run(RETIRED)[0], "skip")
check("an intrinsics file is not a transform", run(INTRINSICS)[0], "skip")
check("and is named as such", "not a transform" in run(INTRINSICS)[1], True)

print("\n%s" % ("all checks pass" if not fails else "%d check(s) FAILED" % len(fails)))
sys.exit(1 if fails else 0)

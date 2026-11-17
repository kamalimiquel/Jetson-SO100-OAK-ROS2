#!/usr/bin/env python3
"""Headless test of --watch's decision rule — no arm, no camera, no rosbridge.

The mode exists to be used on a bench with a human in the loop, which is the worst
place to discover that "it saves while the arm is still moving" or "it saves the same
pose twelve times". `watch_step` is pure, so the whole rule can be pinned here:

    uv run --with roslibpy --with numpy python tools/test_handeye_watch.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import handeye_capture as hc  # noqa: E402

Q = [0.0, -25.0, 30.0, 0.0, 0.0, 12.0]
OTHER = [20.0, -45.0, 40.0, -10.0, 20.0, 12.0]


def corners(n=16, shift=0.0):
    """A 4x4 grid of inner corners, `n` of them, optionally moved by `shift` px."""
    g = np.array([[80 + 60 * (i % 4) + shift, 80 + 60 * (i // 4)] for i in range(16)],
                 dtype=np.float32)
    return g[:n]


def run(name, got, want):
    ok = got == want
    print("%-58s %s" % (name, "ok" if ok else "FAILED (got %r, want %r)" % (got, want)))
    return ok


def main():
    c = corners()
    kept = [{"joints_deg": Q, "corners": 16}]
    fails = 0

    fails += not run("board not visible -> forget the hold, keep watching",
                     hc.watch_step(10.0, Q, None, (9.0, Q, c), kept, 5.0)[0], "wait")
    action, ref = hc.watch_step(10.0, Q, corners(11), None, kept, 5.0)
    fails += not run("11 corners is below the floor of 12 -> wait, no reference",
                     (action, ref), ("wait", None))
    action, ref = hc.watch_step(10.0, Q, c, None, [], 5.0)
    fails += not run("first sighting -> settle and start the hold", (action, ref is not None),
                     ("settle", True))
    fails += not run("0.5 s later, still and same view -> wait (hold is 1.2 s)",
                     hc.watch_step(10.5, Q, c, (10.0, Q, c), [], 5.0)[0], "wait")
    fails += not run("1.3 s later, still and same view -> save",
                     hc.watch_step(11.3, Q, c, (10.0, Q, c), [], 5.0)[0], "save")
    fails += not run("same pose already kept -> duplicate, not a second copy",
                     hc.watch_step(11.3, Q, c, (10.0, Q, c), kept, 5.0)[0], "duplicate")
    moved = [q + 0.5 for q in Q]
    fails += not run("arm moved 0.5 deg during the hold -> settle again",
                     hc.watch_step(11.3, moved, c, (10.0, Q, c), [], 5.0)[0], "settle")
    fails += not run("board jumped 3 px in the image -> settle (stream catching up)",
                     hc.watch_step(11.3, Q, corners(shift=3.0), (10.0, Q, c), [], 5.0)[0],
                     "settle")
    fails += not run("a pose 20 deg away is new -> save",
                     hc.watch_step(11.3, OTHER, c, (10.0, OTHER, c), kept, 5.0)[0], "save")
    fails += not run("steady() tolerance: 0.29 deg is still, 0.31 is not",
                     (hc.steady(Q, [q + 0.29 for q in Q]), hc.steady(Q, [q + 0.31 for q in Q])),
                     (True, False))
    near = [Q[0] + 4.9] + Q[1:]
    far = [Q[0] + 5.0] + Q[1:]
    fails += not run("novelty is the largest single joint delta (4.9 deg is not new)",
                     (hc.novel(far, [{"joints_deg": Q}], 5.0),
                      hc.novel(near, [{"joints_deg": Q}], 5.0)),
                     (True, False))

    print("\n%s" % ("all checks pass" if not fails else "%d check(s) FAILED" % fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Headless test of the swept-volume gate (docker/ws/lib/sweep_gate.py)."""
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("sweep_gate", os.path.join(
    os.path.dirname(HERE), "docker", "ws", "lib", "sweep_gate.py"))
sg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sg)

fails = []


def check(name, got, want):
    ok = got == want
    print(("  ok   " if ok else "  FAIL ") + "%s: %r" % (name, got) + ("" if ok else " (expected %r)" % (want,)))
    if not ok:
        fails.append(name)


BOX = {"min": [-0.40, -0.25, 0.01], "max": [0.45, 0.55, 0.60]}
check("a point inside is inside", sg.inside((0.1, 0.1, 0.2), BOX), True)
check("the boundary counts as inside", sg.inside((-0.40, 0.0, 0.01), BOX), True)
check("below the table is outside", sg.inside((0.1, 0.1, -0.02), BOX), False)
check("past the far arm is outside", sg.inside((0.1, 0.60, 0.2), BOX), False)
check("describe() reads out the box", "x -0.400..0.450" in sg.describe(BOX), True)

qs = sg.sample_configs([0.0, 0.0], [1.0, -2.0], 5)
check("five samples, both ends included", len(qs), 5)
check("first sample is the present pose", qs[0], [0.0, 0.0])
check("last sample is the solution", qs[-1], [1.0, -2.0])
check("the middle is the middle", qs[2], [0.5, -1.0])
try:
    sg.sample_configs([0.0], [1.0], 1)
    check("one step is refused", "did not raise", "raises")
except ValueError:
    check("one step is refused", True, True)

check("a path that stays inside has no violation",
      sg.first_violation([(0.0, 0.0, 0.2), (0.1, 0.1, 0.25)], BOX), None)
viol = sg.first_violation([(0.0, 0.0, 0.2), (0.1, 0.1, -0.05), (0.0, 0.0, 0.2)], BOX)
check("the first offender is reported, with its axis",
      (viol[0], viol[2]), (1, "z"))
check("and includes the point", round(viol[1][2], 2), -0.05)

print("\n%s" % ("all checks pass" if not fails else "%d check(s) FAILED" % len(fails)))
sys.exit(1 if fails else 0)

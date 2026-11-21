"""The swept-volume gate: does the path to a solution stay inside a declared box?

IK answers "where the tool ends up". It says nothing about what the arm passes through on the way, and on a
bench with two arms 30 cm apart and a table underneath, that is the difference between a pose and a crash.
This module is the arithmetic only — no ROS, no chain — so it can be tested with numbers:

    interpolate the joint configuration from the present one to the solution (K steps, shortest way)
    put every link's origin through FK at each step
    reject at the first point outside the box, and name it

The box is declared in `docker/ws/cell_geometry.json` under `sweep_volume` (metres, arm 1's base frame).
**There is no box in that file today**: the bench has not been surveyed, and guessing one would produce a
gate that either refuses everything or blesses the crash. So the gate is implemented, tested, and switched
off — and `IK_SWEEP=1` without a box refuses to start rather than pretending.
"""


def inside(point, box):
    """Is (x, y, z) inside {min: [x,y,z], max: [x,y,z]}? Inclusive: the boundary is allowed."""
    lo, hi = box["min"], box["max"]
    return all(lo[i] <= point[i] <= hi[i] for i in range(3))


def sample_configs(q_from, q_to, steps):
    """Joint-space interpolation from one configuration to the other, both ends included.

    Straight lines in joint space, not in Cartesian space: it is what a joint-space controller will
    actually do between two positions, and a gate that models something else models the wrong thing.
    """
    if steps < 2:
        raise ValueError("need at least 2 steps (the two ends)")
    out = []
    for k in range(steps):
        t = k / (steps - 1.0)
        out.append([a + (b - a) * t for a, b in zip(q_from, q_to)])
    return out


def first_violation(points, box):
    """The first (index, point, axis) outside the box, or None. `points` is a list of (x, y, z)."""
    for i, p in enumerate(points):
        if not inside(p, box):
            lo, hi = box["min"], box["max"]
            axis = next(j for j in range(3) if not (lo[j] <= p[j] <= hi[j]))
            return i, p, "xyz"[axis]
    return None


def describe(box):
    lo, hi = box["min"], box["max"]
    return "x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f m" % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])

#!/usr/bin/env python3
"""A pose that is reachable by construction: the FK of a joint vector inside the limits.

The Phase 3 checks only ever saw unreachable poses, so solve_ik was proven in the negative. This
turns a joint vector into the pose the tool actually reaches, which is what the endpoint should be
fed to close the positive case. Default is the zero pose.

    python3 /opt/tools/ik_pose_fixture.py               # zero pose
    python3 /opt/tools/ik_pose_fixture.py 10,-20,30,0,0 # degrees, jaw appended at 0
"""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.environ.get("IK_DIR", "/opt/ws/control"))
from ik_node import Chain  # noqa: E402


def quat_from_R(R):
    """Rotation matrix -> (x, y, z, w). Shepperd's method, the numerically sane branch."""
    t = float(np.trace(R))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        qw, qx, qy, qz = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw, qx, qy, qz = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw, qx, qy, qz = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw, qx, qy, qz = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return [qx, qy, qz, qw]


def main():
    chain = Chain()
    deg = [float(v) for v in sys.argv[1].split(",")] if len(sys.argv) > 1 else []
    q = np.radians(deg + [0.0] * (chain.dof - len(deg))) if deg else np.zeros(chain.n)
    T = chain.fk(q, jaw=0.0)
    p = T[:3, 3]
    print(json.dumps({
        "joints_deg": [round(float(v), 3) for v in np.degrees(q)[:chain.dof]],
        "pose": {"x": round(float(p[0]), 5), "y": round(float(p[1]), 5), "z": round(float(p[2]), 5)},
        "quat": [round(float(v), 6) for v in quat_from_R(T[:3, :3])],
    }))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Hand-eye capture: drive the arm through poses, save image + gripper pose.

Eye-to-hand setup: the camera is fixed, the ChArUco board is mounted on the
gripper (ADR-0011 amended 2026-11-09). For every pose we store

    poses/pose_XX.jpg    the scene camera frame (from the cams container :8090)
    poses/poses.json     joint values (deg) + base->jaw transform from FK

so the solve step can be re-run from the stored data without touching the arm.

    python3 handeye_capture.py --host 192.168.1.185 --out ~/calib/scene
"""

import argparse
import json
import math
import os
import time
import urllib.request

import roslibpy

# ---------------------------------------------------------------- URDF chain
# (name, parent, xyz, rpy, axis) — kept identical to docker/ws/arm/urdf/so100.urdf,
# which is aligned frame-by-frame with the MuJoCo Menagerie model.
CHAIN = [
    ("shoulder_pan", (0.0, -0.0452, 0.0165), (1.5708, 0.0, 0.0), (0, 1, 0)),
    ("shoulder_lift", (0.0, 0.1025, 0.0306), (1.5708, 0.0, 0.0), (1, 0, 0)),
    ("elbow_flex", (0.0, 0.11257, 0.028), (-1.5708, 0.0, 0.0), (1, 0, 0)),
    ("wrist_flex", (0.0, 0.0052, 0.1349), (-1.5708, 0.0, 0.0), (1, 0, 0)),
    ("wrist_roll", (0.0, -0.0601, 0.0), (0.0, 1.5708, 0.0), (0, 1, 0)),
    ("gripper", (-0.0202, -0.0244, 0.0), (0.0, 3.14159, 0.0), (0, 0, 1)),
]


def _mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mv(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def _rpy(r, p, y):
    return _mm(_rz(y), _mm(_ry(p), _rx(r)))


def _axis_rot(axis, ang):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(ang), math.sin(ang)
    C = 1 - c
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def fk(q_deg):
    """base -> jaw, as a 4x4 (row-major nested lists)."""
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    p = [0.0, 0.0, 0.0]
    for (_, xyz, rpy_, axis), ang in zip(CHAIN, q_deg):
        p = [p[i] + _mv(R, list(xyz))[i] for i in range(3)]
        R = _mm(R, _rpy(*rpy_))
        R = _mm(R, _axis_rot(axis, math.radians(ang)))
    return [[R[0][0], R[0][1], R[0][2], p[0]],
            [R[1][0], R[1][1], R[1][2], p[1]],
            [R[2][0], R[2][1], R[2][2], p[2]],
            [0, 0, 0, 1]]


# ---------------------------------------------------------------- poses
# Deliberately varied: pan sweep, lift/elbow combos, wrist tilts and rolls.
# Every pose stays inside the servo travel and keeps the board facing the camera.
DEFAULT_POSES = [
    (0, -25, 30, 0, 0), (25, -25, 30, 0, 0), (-25, -25, 30, 0, 0),
    (0, -45, 45, 10, 0), (20, -45, 40, -10, 20), (-20, -40, 50, 15, -20),
    (15, -20, 20, 25, 40), (-15, -20, 25, -20, -40),
    (0, -35, 60, 0, 60), (0, -30, 55, 0, -60),
    (30, -35, 45, 20, 30), (-30, -35, 45, 20, -30),
    (30, -50, 60, -15, 45), (-30, -50, 60, -15, -45),
    (0, -55, 70, 25, 0), (10, -15, 35, -25, 0),
    (-10, -60, 65, 15, 90), (10, -60, 65, 15, -90),
    (0, -30, 40, 40, 120), (0, -30, 40, -35, -120),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.1.185")
    ap.add_argument("--port", type=int, default=9090)
    ap.add_argument("--view", type=int, default=8090, help="MJPEG view port (/snapshot)")
    ap.add_argument("--out", default=os.path.expanduser("~/calib/scene"))
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--gripper", type=float, default=45.0)
    ap.add_argument("--no-check", action="store_true",
                    help="skip the per-frame ChArUco check (needs opencv in the env)")
    args = ap.parse_args()

    # live board check: a sweep that only sees the board in a few poses is useless,
    # and finding out at solve time costs another full sweep (arm + operator time).
    board = det = None
    if not args.no_check:
        try:
            import cv2
            board = cv2.aruco.CharucoBoard((5, 5), 0.018, 0.0135,
                                           cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
            det = cv2.aruco.CharucoDetector(board)
        except ImportError:
            print("!! opencv not in this env — running without the board check")

    os.makedirs(args.out, exist_ok=True)
    client = roslibpy.Ros(host=args.host, port=args.port)
    client.run()
    state = {}
    roslibpy.Topic(client, "/joint_states", "sensor_msgs/JointState").subscribe(
        lambda m: state.update(p=[float(x) for x in m["position"][:6]]))
    pub = roslibpy.Topic(client, "/joint_targets", "std_msgs/Float64MultiArray")
    pub.advertise()
    t0 = time.time()
    while "p" not in state and time.time() - t0 < 15:
        time.sleep(0.2)
    if "p" not in state:
        raise SystemExit("no /joint_states — is the arm container up?")

    records = []
    for i, pose in enumerate(DEFAULT_POSES):
        tgt = list(pose) + [args.gripper]
        for _ in range(40):                       # ~1 s of 40 Hz targets
            pub.publish({"layout": {"dim": [], "data_offset": 0}, "data": tgt})
            time.sleep(0.025)
        time.sleep(args.settle)                   # let the servos settle
        q = list(state["p"])
        url = "http://%s:%d/snapshot" % (args.host, args.view)
        jpg = urllib.request.urlopen(url, timeout=10).read()
        name = "pose_%02d.jpg" % i
        with open(os.path.join(args.out, name), "wb") as f:
            f.write(jpg)
        seen = None
        if det is not None:
            import cv2
            import numpy as np
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
            c, _, _, _ = det.detectBoard(img)
            seen = 0 if c is None else len(c)
        records.append({"image": name, "target": tgt, "joints_deg": q,
                        "fk_base_jaw": fk(q), "corners": seen})
        print("pose %02d/%d  joints=%s  %d B%s"
              % (i + 1, len(DEFAULT_POSES), [round(v, 1) for v in q], len(jpg),
                 "" if seen is None else "  corners=%2d/16%s" % (seen, "" if seen >= 12 else "  <-- BAD")))

    with open(os.path.join(args.out, "poses.json"), "w") as f:
        json.dump({"board": {"squares": 5, "square_m": 0.018, "marker_m": 0.0135,
                             "dict": "DICT_4X4_50"}, "poses": records}, f, indent=1)
    print("saved", len(records), "poses to", args.out)
    pub.unadvertise()
    client.terminate()


if __name__ == "__main__":
    main()

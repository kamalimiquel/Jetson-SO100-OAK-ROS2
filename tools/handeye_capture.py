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
# FK straight from the URDF the robot actually publishes — never a hand-copied
# chain. The physical arm is an SO-101 (that is the model the MuJoCo twin loads);
# an SO-100 chain here silently produced a different robot and ruined the
# hand-eye solve.
import xml.etree.ElementTree as _ET

URDF_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         "docker", "ws", "arm", "urdf", "so101.urdf")
JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
               "wrist_roll", "gripper"]


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


def _axis_rot(axis, ang):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(ang), math.sin(ang)
    C = 1 - c
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def _rpy2m(r, p, y):
    return _mm(_rz(y), _mm(_ry(p), _rx(r)))


def _load_chain(path=None):
    """[(parent, child, xyz, rpy, axis)] ordered from the root link outwards."""
    root = _ET.parse(path or URDF_PATH).getroot()
    joints = {}
    for j in root.findall("joint"):
        if j.get("type") != "revolute":
            continue
        o = j.find("origin")
        a = j.find("axis")
        joints[j.find("child").get("link")] = (
            j.find("parent").get("link"),
            [float(v) for v in o.get("xyz").split()],
            [float(v) for v in o.get("rpy").split()],
            [float(v) for v in a.get("xyz").split()],
        )
    chain, link = [], None
    while joints:
        nxt = next((c for c, (p, *_rest) in joints.items() if link is None or p == link), None)
        if nxt is None:
            break
        chain.append(joints.pop(nxt))
        link = nxt
    return chain


_CHAIN = None


def fk(q_deg):
    """base -> jaw as a 4x4 nested list, q_deg in JOINT_ORDER (degrees)."""
    global _CHAIN
    if _CHAIN is None:
        _CHAIN = _load_chain()
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    p = [0.0, 0.0, 0.0]
    for (parent, xyz, rpy, axis), ang in zip(_CHAIN, q_deg):
        p = [p[i] + _mv(R, list(xyz))[i] for i in range(3)]
        R = _mm(R, _rpy2m(*rpy))
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

# Adaptive mode: small offsets around the pose where the board was last seen.
# The scene camera on this rig has a narrow field of view, so absolute poses are
# a lottery; walking away from a known-good pose and rejecting the ones that lose
# the board converges in a single pass without driving the arm for nothing.
# (pan, lift, elbow, wrist_flex) — wrist_roll is left alone: it turns the board
# edge-on to the camera and is not needed for rotation diversity.
ADAPTIVE_STEPS = [
    (0, 0, 0, 0),
    (25, 0, 0, 0), (-25, 0, 0, 0),
    (0, 18, 0, 0), (0, -18, 0, 0),
    (0, 0, 25, 0), (0, 0, -25, 0),
    (0, 0, 0, 25), (0, 0, 0, -25),
    (20, 15, 0, 0), (-20, -15, 0, 0),
    (0, 15, 20, 0), (0, -15, -20, 0),
    (15, 0, 0, 20), (-15, 0, 0, -20),
    (0, 0, 20, 20), (0, 0, -20, -20),
    (25, 18, 25, 0), (-25, -18, -25, 0),
    (0, 20, 0, 20), (0, -20, 0, -20),
    (25, 0, 20, -15), (-25, 0, -20, 15),
]

LIMITS = [(-80, 80), (-70, 15), (-5, 75), (-45, 45), (-180, 180), (0, 100)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.1.185")
    ap.add_argument("--port", type=int, default=9090)
    ap.add_argument("--view", type=int, default=8090, help="MJPEG view port (/snapshot)")
    ap.add_argument("--out", default=os.path.expanduser("~/calib/scene"))
    ap.add_argument("--settle", type=float, default=1.5)
    ap.add_argument("--gripper", type=float, default=None,
                    help="gripper target; default None = never send joint 6 at all")
    ap.add_argument("--deg-per-s", type=float, default=25.0,
                    help="ramp speed between poses (gentle)")
    ap.add_argument("--no-check", action="store_true",
                    help="skip the per-frame ChArUco check (needs opencv in the env)")
    ap.add_argument("--adaptive", action="store_true",
                    help="walk small offsets around the current pose instead of fixed poses")
    ap.add_argument("--watch", action="store_true",
                    help="user moves the arm by hand/pad; capture any stable pose that sees the board")
    ap.add_argument("--want", type=int, default=14, help="stop after this many accepted poses (--watch)")
    ap.add_argument("--watch-minutes", type=float, default=10.0, help="give up after this long (--watch)")
    ap.add_argument("--min-novelty", type=float, default=5.0,
                    help="a new pose must differ by at least this many degrees (any joint)")
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
    aborted = False
    adaptive = bool(args.adaptive)
    targets = []
    if adaptive:
        q0 = list(state["p"])
        for dp, dl, de, dw in ADAPTIVE_STEPS:
            cand = [q0[0] + dp, q0[1] + dl, q0[2] + de, q0[3] + dw, q0[4]]
            cand = [min(max(v, lo), hi) for v, (lo, hi) in zip(cand, LIMITS[:5])]
            targets.append((tuple(cand), None))
        print("adaptive sweep from the current pose: %s" % [round(v, 1) for v in q0])
    else:
        targets = [(tuple(p), None) for p in DEFAULT_POSES]

    last_good = list(state["p"])
    for i, (pose, _) in enumerate(targets):
        base = list(state["p"])
        # joint 6 (gripper) is left out of the message on purpose: the joint server
        # accepts 5 values and then never writes that servo. NaN over rosbridge is a
        # trap — JSON has no NaN, the whole message gets dropped and nothing moves.
        goal = list(pose[:5]) if args.gripper is None else list(pose[:5]) + [args.gripper]
        # gentle ramp: biggest joint delta decides how long the move takes
        delta = max(abs(g - b) for g, b in zip(goal[:5], base[:5]))
        ramp = max(0.8, min(4.0, delta / max(1.0, args.deg_per_s)))
        steps = max(8, int(ramp / 0.05))
        for k in range(1, steps + 1):
            a = k / float(steps)
            mid = [b + (g - b) * a for b, g in zip(base[:len(goal)], goal)]
            pub.publish({"layout": {"dim": [], "data_offset": 0}, "data": mid})
            time.sleep(0.05)
        time.sleep(args.settle)                   # let the servos settle
        q = list(state["p"])
        url = "http://%s:%d/snapshot" % (args.host, args.view)
        jpg = urllib.request.urlopen(url, timeout=10).read()
        seen = None
        if det is not None:
            import cv2
            import numpy as np
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
            c, _, _, _ = det.detectBoard(img)
            seen = 0 if c is None else len(c)
        good = seen is None or seen >= 12
        if adaptive and not good:
            # lost the board: come back to the last pose that saw it, drop the frame
            print("pose %02d/%d  joints=%s  corners=%s  <-- board lost, reverting"
                  % (i + 1, len(targets), [round(v, 1) for v in q], seen))
            delta = max(abs(g - b) for g, b in zip(last_good[:5], state["p"][:5]))
            steps = max(8, int(max(0.8, min(4.0, delta / max(1.0, args.deg_per_s))) / 0.05))
            base = list(state["p"])
            for k in range(1, steps + 1):
                a = k / float(steps)
                mid = [b + (g - b) * a for b, g in zip(base, last_good)]
                pub.publish({"layout": {"dim": [], "data_offset": 0}, "data": mid})
                time.sleep(0.05)
            time.sleep(args.settle)
            continue
        last_good = list(q)
        name = "pose_%02d.jpg" % len(records)
        with open(os.path.join(args.out, name), "wb") as f:
            f.write(jpg)
        records.append({"image": name, "target": goal, "joints_deg": q,
                        "fk_base_jaw": fk(q), "corners": seen})
        print("pose %02d/%d  joints=%s  corners=%s%s"
              % (i + 1, len(targets), [round(v, 1) for v in q], seen,
                 "" if good else "  <-- BAD"))
        if i == 0 and seen == 0:
            print("\n!! the board is not visible in the very first pose — aborting so the arm "
                  "does not keep moving for nothing. Fix the board/camera and re-run.")
            aborted = True
            break

    with open(os.path.join(args.out, "poses.json"), "w") as f:
        json.dump({"board": {"squares": 5, "square_m": 0.018, "marker_m": 0.0135,
                             "dict": "DICT_4X4_50"}, "poses": records}, f, indent=1)
    print("saved", len(records), "poses to", args.out)
    pub.unadvertise()
    client.terminate()


if __name__ == "__main__":
    main()

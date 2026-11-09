#!/usr/bin/env python3
"""Grab ONE hand-eye sample with a freshness check.

Failure this guards against: the MJPEG view serves the newest frame it has, which
can be up to ~1 s older than the joint readings. Pairing a stale image with a fresh
pose silently poisons the hand-eye solve, so a sample is accepted only when

  * the joints are stationary (two readings 1.2 s apart agree within 0.3 deg), and
  * two frames 1.2 s apart show the board in the SAME place (within 2 px),
    which proves the stream has caught up with the arm.

Nothing is ever commanded: joint 6 is not written, no joint is driven.

    python3 handeye_grab.py            # add a sample
    python3 handeye_grab.py --status   # what is collected so far
"""

import argparse
import json
import os
import sys
import time
import urllib.request

import roslibpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from handeye_capture import fk  # noqa: E402

BOARD = {"squares": 5, "square_m": 0.018, "marker_m": 0.0135, "dict": "DICT_4X4_50"}
MIN_CORNERS = 12


def load(path):
    return json.load(open(path)) if os.path.exists(path) else {"board": BOARD, "poses": []}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.expanduser("~/calib/scene3"))
    ap.add_argument("--host", default="192.168.1.185")
    ap.add_argument("--port", type=int, default=9090)
    ap.add_argument("--view", type=int, default=8090)
    ap.add_argument("--status", action="store_true")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    pj = os.path.join(args.out, "poses.json")
    data = load(pj)

    if args.status:
        print("%d samples in %s" % (len(data["poses"]), args.out))
        for p in data["poses"]:
            print("  %-14s corners=%-3s joints=%s"
                  % (p["image"], p.get("corners"), [round(v, 1) for v in p["joints_deg"]]))
        return

    import cv2
    import numpy as np
    import re
    board = cv2.aruco.CharucoBoard((5, 5), 0.018, 0.0135,
                                   cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    det = cv2.aruco.CharucoDetector(board)
    # camera matrix for the obliqueness/size checks (factory and charuco agree within 2 %)
    intr = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "docs", "calibration", "scene_cam_intrinsics.yaml")
    _y = open(intr).read()
    K = np.array([float(v) for v in re.search(
        r"camera_matrix:\n  rows: 3\n  cols: 3\n  data: \[(.*?)\]", _y, re.S).group(1).split(",")]).reshape(3, 3)
    DIST = np.array([float(v) for v in re.search(
        r"distortion_coefficients:\n  rows: 1\n  cols: 5\n  data: \[(.*?)\]", _y, re.S).group(1).split(",")])

    client = roslibpy.Ros(host=args.host, port=args.port)
    client.run()
    st = {"p": None}
    roslibpy.Topic(client, "/joint_states", "sensor_msgs/JointState").subscribe(
        lambda m: st.update(p=[float(x) for x in m["position"][:6]]))
    t0 = time.time()
    while st["p"] is None and time.time() - t0 < 10:
        time.sleep(0.2)
    if st["p"] is None:
        raise SystemExit("no /joint_states")

    def corners_in(jpg):
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
        c, _, _, _ = det.detectBoard(img)
        return None if c is None else c.reshape(-1, 2).astype(np.float32)

    url = "http://%s:%d/snapshot" % (args.host, args.view)
    q1 = list(st["p"])
    jpg1 = urllib.request.urlopen(url, timeout=10).read()
    time.sleep(1.2)
    q2 = list(st["p"])
    jpg2 = urllib.request.urlopen(url, timeout=10).read()

    jump = max(abs(a - b) for a, b in zip(q1, q2))
    if jump > 0.3:
        client.terminate()
        print("rejected: the arm is still moving (%.1f deg between readings) — let it settle"
              % jump)
        return

    c1, c2 = corners_in(jpg1), corners_in(jpg2)
    n1 = 0 if c1 is None else len(c1)
    n2 = 0 if c2 is None else len(c2)
    if n1 < MIN_CORNERS or n2 < MIN_CORNERS:
        client.terminate()
        print("rejected: board not fully visible (frames: %d / %d corners) — move it into view"
              % (n1, n2))
        return
    if len(c1) != len(c2):
        client.terminate()
        print("rejected: unstable detection (%d vs %d corners) — hold still and repeat" % (n1, n2))
        return
    px_move = float(np.abs(c1 - c2).max())
    if px_move > 2.0:
        client.terminate()
        print("rejected: the view is still catching up (board moved %.1f px between two frames) "
              "— hold still a second and repeat" % px_move)
        return

    # obliqueness: a board seen edge-on gives a pose whose normal and depth are almost
    # unconstrained, which is what poisons the hand-eye solve. Require the board to face
    # the camera within 45 degrees and to be big enough in the image.
    obj = board.getChessboardCorners()
    ok, rvec, tvec = cv2.solvePnP(obj, c2.reshape(-1, 1, 2).astype(np.float64), K, DIST,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        client.terminate()
        print("rejected: could not solve the board pose — repeat the shot")
        return
    R_cb = cv2.Rodrigues(rvec)[0]
    normal = R_cb[:, 2]
    to_board = tvec.ravel() / np.linalg.norm(tvec)
    tilt = float(np.degrees(np.arccos(np.clip(abs(np.dot(normal, to_board)), -1, 1))))
    span = float(np.linalg.norm(c2.max(axis=0) - c2.min(axis=0)))
    if tilt > 45.0:
        client.terminate()
        print("rejected: board is seen too edge-on (%.0f deg off facing the camera, max 45) "
              "— turn the gripper so the board faces the camera" % tilt)
        return
    if span < 180.0:
        client.terminate()
        print("rejected: board too small in the image (%.0f px across, need 180) — bring it "
              "closer to the camera" % span)
        return

    name = "pose_%02d.jpg" % len(data["poses"])
    with open(os.path.join(args.out, name), "wb") as f:
        f.write(jpg2)
    q = list(st["p"])
    data["poses"].append({"image": name, "joints_deg": q, "fk_base_jaw": fk(q),
                          "corners": int(n2)})
    json.dump(data, open(pj, "w"), indent=1)
    client.terminate()

    print("saved %s  corners=%d/16  joints=%s" % (name, n2, [round(v, 1) for v in q]))
    print("%d samples total" % len(data["poses"]))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Hand-eye solve for the eye-to-hand scene camera (ADR-0011 amended).

Input : the directory produced by handeye_capture.py (poses.json + pose_*.jpg)
Output: docs/calibration/scene_cam_intrinsics.yaml
        docs/calibration/scene_cam_extrinsic.yaml      (base -> scene_cam)

No ROS, no easy_handeye: OpenCV does both solves.

    python3 handeye_solve.py --dir ~/calib/scene
"""

import argparse
import glob
import json
import math
import os

import cv2
import numpy as np

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def board_from(spec):
    return cv2.aruco.CharucoBoard((spec["squares"], spec["squares"]),
                                  spec["square_m"], spec["marker_m"], DICT)


def detect(paths, board):
    det = cv2.aruco.CharucoDetector(board)
    corners, ids, imsize = [], [], None
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print("  !! unreadable:", p)
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        imsize = gray.shape[::-1]
        c, i, _, _ = det.detectBoard(gray)
        n = 0 if c is None else len(c)
        print("  %-28s %2d/16 corners" % (os.path.basename(p), n))
        if c is not None and n >= 8:
            corners.append(c)
            ids.append(i)
    return corners, ids, imsize


def inv(T):
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T.dot(t)
    return Ti


def spread(mats):
    """How much a set of 4x4 transforms disagree — the calibration residual."""
    ts = np.array([m[:3, 3] for m in mats])
    t_err = float(np.linalg.norm(ts - ts.mean(axis=0), axis=1).max() * 1000)
    ref = mats[0][:3, :3]
    r_err = 0.0
    for m in mats:
        dR = ref.T.dot(m[:3, :3])
        ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2))))
        r_err = max(r_err, ang)
    return t_err, r_err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.expanduser("~/calib/scene"))
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "calibration"))
    args = ap.parse_args()

    data = json.load(open(os.path.join(args.dir, "poses.json")))
    board = board_from(data["board"])
    poses = data["poses"]
    paths = [os.path.join(args.dir, p["image"]) for p in poses]
    print("detecting the board in %d frames" % len(paths))
    corners, ids, imsize = detect(paths, board)
    if len(corners) < 6:
        raise SystemExit("not enough usable frames (%d) — check lighting/occlusion" % len(corners))

    # ---------------------------------------------------------- intrinsics
    print("\nintrinsics (cv2.calibrateCameraCharuco) on %d views" % len(corners))
    rms, K, dist, _, _ = cv2.aruco.calibrateCameraCharuco(
        corners, ids, board, imsize, None, None)
    print("  rms %.2f px   fx=%.1f fy=%.1f cx=%.1f cy=%.1f"
          % (rms, K[0, 0], K[1, 1], K[0, 2], K[1, 2]))

    # ---------------------------------------------------- board pose per view
    R_t2c, t_t2c = [], []
    used = []
    for p, c, i in zip([q for q in poses], corners, ids):
        ok, rvec, tvec = cv2.solvePnP(board.getChessboardCorners(), c, K, dist,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        R_t2c.append(cv2.Rodrigues(rvec)[0])
        t_t2c.append(tvec.reshape(3, 1))
        used.append(p)
    print("board pose solved for %d/%d frames" % (len(used), len(poses)))

    R_g2b, t_g2b = [], []
    for p in used:
        T = np.array(p["fk_base_jaw"], dtype=float)
        R_g2b.append(T[:3, :3])
        t_g2b.append(T[:3, 3].reshape(3, 1))

    # ------------------------------------------------------------ hand-eye
    # eye-to-hand: the camera is bolted to the base and the board rides on the
    # gripper, so the pair fed to the solver is (base->gripper, target->cam).
    print("\nhand-eye (cv2.calibrateHandEye, PARK)")
    results = {}
    for label, (Rg, tg) in {
        "base2gripper": ([(np.array(np.array(R).T)) for R in R_g2b],
                         [(-np.array(R).T.dot(np.array(t))) for R, t in zip(R_g2b, t_g2b)]),
        "gripper2base": (R_g2b, t_g2b),
    }.items():
        R_cam, t_cam = cv2.calibrateHandEye([np.array(r) for r in Rg],
                                            [np.array(t) for t in tg],
                                            R_t2c, t_t2c,
                                            method=cv2.CALIB_HAND_EYE_PARK)
        T = np.eye(4)
        T[:3, :3] = R_cam
        T[:3, 3] = np.array(t_cam).reshape(3)
        # consistency: gripper->board must be the same in every pose
        consts = [inv(np.array(p["fk_base_jaw"], dtype=float)).dot(T).dot(
            np.block([[R_t2c[k], t_t2c[k]], [np.zeros(3), 1.0]])) for k, p in enumerate(used)]
        t_err, r_err = spread(consts)
        results[label] = (T, t_err, r_err)
        print("  %-13s residual: %.1f mm / %.2f deg" % (label, t_err, r_err))

    best = min(results.items(), key=lambda kv: kv[1][1] + kv[1][2] * 10)
    label, (T_bc, t_err, r_err) = best
    print("\nusing %s -> base<-cam = %s, residual %.1f mm / %.2f deg"
          % (label, np.round(T_bc[:3, 3], 4), t_err, r_err))

    # ------------------------------------------------------------------ save
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "scene_cam_intrinsics.yaml"), "w") as f:
        f.write("# scene OAK-D intrinsics — cv2.aruco.calibrateCameraCharuco, rms %.2f px\n"
                "# %d views, %s\n" % (rms, len(corners), data["board"]))
        f.write("image_width: %d\nimage_height: %d\ncamera_name: scene_cam\n"
                % (imsize[0], imsize[1]))
        f.write("camera_matrix:\n  rows: 3\n  cols: 3\n  data: [%s]\n"
                % ", ".join("%.6f" % v for v in K.reshape(-1)))
        f.write("distortion_model: plumb_bob\ndistortion_coefficients:\n  rows: 1\n  cols: 5\n"
                "  data: [%s]\n" % ", ".join("%.8f" % v for v in np.ravel(dist)))
        f.write("reprojection_rms_px: %.3f\n" % rms)
    with open(os.path.join(args.out, "scene_cam_extrinsic.yaml"), "w") as f:
        f.write("# base <- scene_cam  (eye-to-hand, board on the gripper)\n"
                "# solver: cv2.calibrateHandEye/%s — residual %.1f mm / %.2f deg over %d poses\n"
                % (label, t_err, r_err, len(used)))
        f.write("parent: base\nchild: scene_cam_link\n")
        f.write("translation: [%s]\n" % ", ".join("%.6f" % v for v in T_bc[:3, 3]))
        for r in range(3):
            f.write("rotation_row%d: [%s]\n" % (r, ", ".join("%.8f" % v for v in T_bc[r, :3])))
        f.write("residual_mm: %.2f\nresidual_deg: %.2f\n" % (t_err, r_err))
    print("wrote", args.out)


if __name__ == "__main__":
    main()

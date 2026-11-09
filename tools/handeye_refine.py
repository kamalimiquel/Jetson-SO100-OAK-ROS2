#!/usr/bin/env python3
"""Non-linear hand-eye refinement (bundle adjustment) for the scene camera.

Closed-form hand-eye (cv2.calibrateHandEye) amplifies small pose errors. This fits
both unknowns at once by minimising the reprojection error of the ChArUco corners
in every shot:

    T_base_cam        where the camera sits          (6 params)
    T_gripper_board   how the board is bolted on     (6 params)

residual = |project(K, dist, inv(T_base_cam) * T_base_gripper_i * T_gripper_board * X_board) - x_pixels|

If the minimum is still large, the kinematic model itself (not the calibration) is
wrong, and that is worth knowing before trusting any extrinsic.

    python3 handeye_refine.py --dir ~/calib/scene
"""

import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from handeye_capture import fk  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.expanduser("~/calib/scene"))
    ap.add_argument("--intrinsics", default=None)
    args = ap.parse_args()

    d = args.dir
    intr_path = args.intrinsics or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "docs", "calibration", "scene_cam_intrinsics.yaml")
    y = open(intr_path).read()
    import re
    K = np.array([float(v) for v in re.search(
        r"camera_matrix:\n  rows: 3\n  cols: 3\n  data: \[(.*?)\]", y, re.S).group(1).split(",")]).reshape(3, 3)
    dist = np.array([float(v) for v in re.search(
        r"distortion_coefficients:\n  rows: 1\n  cols: 5\n  data: \[(.*?)\]", y, re.S).group(1).split(",")])

    board = cv2.aruco.CharucoBoard((5, 5), 0.018, 0.0135,
                                   cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    det = cv2.aruco.CharucoDetector(board)
    obj_all = board.getChessboardCorners()          # (16,3) board-centred points

    shots = []
    for p in json.load(open(os.path.join(d, "poses.json")))["poses"]:
        img = cv2.imread(os.path.join(d, p["image"]), cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        c, i, _, _ = det.detectBoard(img)
        if c is None or len(c) < 10:
            continue
        c = cv2.cornerSubPix(img, c.astype(np.float32), (5, 5), (-1, -1),
                             (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3))
        o, pt = board.matchImagePoints(c, i)
        T = np.array(fk(list(p["joints_deg"][:6])), dtype=float)
        shots.append((p["image"], T, np.array(pt, dtype=float).reshape(-1, 2), np.array(o, dtype=float).reshape(-1, 3)))
    print("%d shots usable" % len(shots))

    def project(params, T_gb, obj, base_T):
        R = Rotation.from_rotvec(params[0:3]).as_matrix()
        t = params[3:6]
        Rb = Rotation.from_rotvec(params[6:9]).as_matrix()
        tb = params[9:12]
        T_cam_base = np.eye(4)
        T_cam_base[:3, :3] = R.T
        T_cam_base[:3, 3] = -R.T.dot(t)
        T = np.eye(4)
        T[:3, :3] = Rb
        T[:3, 3] = tb
        P = T_cam_base.dot(base_T).dot(T).dot(np.c_[obj, np.ones(len(obj))].T).T
        rv, _ = cv2.Rodrigues(np.eye(3))
        img, _ = cv2.projectPoints(P[:, :3].astype(float), rv, np.zeros(3), K, dist)
        return img.reshape(-1, 2)

    def resid(params):
        out = []
        for _, T_bg, px, obj in shots:
            pr = project(params, None, obj, T_bg)
            out.append((pr - px).ravel())
        return np.concatenate(out)

    # seeds: the closed-form hand-eye result first, then a small grid of plausible
    # camera placements. A single guess lands in a local minimum and reports a huge
    # rms that says nothing about the data.
    seeds = []
    try:
        Rg = [np.array(s[1], dtype=float)[:3, :3] for s in shots]
        tg = [np.array(s[1], dtype=float)[:3, 3].reshape(3, 1) for s in shots]
        Rc = []; tc = []
        for _, T_bg, px, obj in shots:
            ok, rv, tv = cv2.solvePnP(obj, px, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
            Rc.append(cv2.Rodrigues(rv)[0]); tc.append(tv.reshape(3, 1))
        R_h, t_h = cv2.calibrateHandEye(Rg, tg, Rc, tc, method=cv2.CALIB_HAND_EYE_PARK)
        T = np.eye(4); T[:3, :3] = R_h; T[:3, 3] = np.array(t_h).reshape(3)
        for inv_it in (False, True):
            M = np.linalg.inv(T) if inv_it else T
            seed = np.zeros(12)
            seed[0:3] = Rotation.from_matrix(M[:3, :3]).as_rotvec()
            seed[3:6] = M[:3, 3]
            seed[9:12] = [0.0, -0.02, 0.08]
            seeds.append(("hand-eye" + ("^-1" if inv_it else ""), seed))
    except Exception as e:  # noqa: BLE001
        print("closed-form seed failed:", e)
    for rx in (50, 70, 90):
        for ty in (-0.25, -0.45):
            for tz in (0.10, 0.30, 0.50):
                seed = np.zeros(12)
                seed[0:3] = Rotation.from_euler("x", rx, degrees=True).as_rotvec()
                seed[3:6] = [0.0, ty, tz]
                seed[9:12] = [0.0, -0.02, 0.08]
                seeds.append(("grid", seed))

    best = None
    for name, p0 in seeds:
        try:
            sol = least_squares(resid, p0, method="trf", xtol=1e-12, ftol=1e-12,
                                max_nfev=3000)
        except Exception:  # noqa: BLE001
            continue
        rms = float(np.sqrt(np.mean(resid(sol.x) ** 2)))
        if best is None or rms < best[0]:
            best = (rms, sol, name)
    rms, sol, name = best
    print("best seed: %s" % name)
    r = resid(sol.x)
    rms = float(np.sqrt(np.mean(r ** 2)))
    print("reprojection rms: %.2f px  (max %.2f px)" % (rms, float(np.abs(r).max())))

    R = Rotation.from_rotvec(sol.x[0:3]).as_matrix()
    t = sol.x[3:6]
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    print("base<-cam translation: [%s]  |t|=%.3f m" % (", ".join("%.4f" % v for v in t), np.linalg.norm(t)))
    Rb = Rotation.from_rotvec(sol.x[6:9]).as_matrix()
    print("board on the gripper: t=[%s]  (%.0f cm from the jaw frame)"
          % (", ".join("%.4f" % v for v in sol.x[9:12]), np.linalg.norm(sol.x[9:12]) * 100))

    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "docs", "calibration", "scene_cam_extrinsic.yaml")
    with open(out, "w") as f:
        f.write("# base <- scene_cam  (eye-to-hand, board mounted on the gripper)\n"
                "# solver: non-linear bundle adjustment over %d shots, reprojection rms %.2f px\n"
                % (len(shots), rms))
        f.write("parent: base\nchild: scene_cam_link\n")
        f.write("translation: [%s]\n" % ", ".join("%.6f" % v for v in t))
        for i in range(3):
            f.write("rotation_row%d: [%s]\n" % (i, ", ".join("%.8f" % v for v in R[i])))
        f.write("reprojection_rms_px: %.3f\n" % rms)
        f.write("board_on_gripper_translation: [%s]\n" % ", ".join("%.6f" % v for v in sol.x[9:12]))
        for i in range(3):
            f.write("board_on_gripper_rotation_row%d: [%s]\n" % (i, ", ".join("%.8f" % v for v in Rb[i])))
    print("wrote", out)


if __name__ == "__main__":
    main()

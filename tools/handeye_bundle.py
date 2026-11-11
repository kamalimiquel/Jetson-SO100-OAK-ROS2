#!/usr/bin/env python3
"""Hand-eye solve with a joint bundle adjustment, and a hold-out test that means something.

The closed-form solvers (cv2.calibrateHandEye) give an answer whose error nobody can see: the
"residual" they let you compute is the spread of the board-in-gripper pose, which stays small even
when the extrinsic is wrong. This script does the honest thing instead:

  1. detect the ChArUco board in every frame and solve its pose (PnP) with the camera's **measured**
     intrinsics — not intrinsics re-estimated from the same images;
  2. initialise the extrinsic with the closed form;
  3. refine **both** unknowns — the camera in `base` (X) and the board on the jaw (Y) — by
     minimising the pixel reprojection error of every corner in every view;
  4. report the leave-one-out error: solve on 11 views, predict the 12th, measure the miss. That
     number is what a grasp planner would actually suffer.

    uv run --with "opencv-contrib-python-headless" --with numpy python tools/handeye_bundle.py \
        --dir docs/calibration/datasets/2026-11-09-so101 \
        --intrinsics docs/calibration/scene_cam_intrinsics.yaml \
        --out docs/calibration/scene_cam_extrinsic.yaml
"""

import argparse
import glob
import json
import math
import os

import cv2
import numpy as np

DICT_NAME = "DICT_4X4_50"


def rodrigues(rvec):
    return cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))[0]


def inv(T):
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def load_intrinsics(path):
    """Read the intrinsics file we write ourselves (fx..cy + distortion)."""
    txt = open(path).read()
    block = txt.split("camera_matrix:", 1)[1].split("distortion_coefficients:", 1)[0]
    vals = [float(v) for v in block.split("data:", 1)[1].split("[", 1)[1].split("]", 1)[0].split(",")]
    K = np.array(vals, dtype=float).reshape(3, 3)
    dist = np.zeros(5)
    if "distortion_coefficients:" in txt:
        dblock = txt.split("distortion_coefficients:", 1)[1]
        if "data:" in dblock:
            dv = [float(v) for v in dblock.split("data:", 1)[1].split("[", 1)[1].split("]", 1)[0].split(",")]
            if len(dv) >= 5:
                dist = np.array(dv[:5], dtype=float).reshape(1, 5)
    return K, dist


def board_from(spec):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec.get("dict", DICT_NAME)))
    return cv2.aruco.CharucoBoard((spec["squares"], spec["squares"]),
                                 spec["square_m"], spec["marker_m"], d)


def detect(dirpath, poses, board):
    """[(pose_dict, obj_points Nx3, img_points Nx2)] for the frames where the board is found."""
    det = cv2.aruco.CharucoDetector(board)
    out = []
    for p in poses:
        path = os.path.join(dirpath, p["image"])
        img = cv2.imread(path)
        if img is None:
            print("  !! unreadable:", path)
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _, _ = det.detectBoard(gray)
        if corners is None or ids is None or len(corners) < 6:
            print("  !! %s: %d corners — skipped" % (p["image"], 0 if corners is None else len(corners)))
            continue
        obj, imgp = board.matchImagePoints(corners, ids)
        if obj is None or len(obj) < 6:
            continue
        out.append((p, obj.reshape(-1, 3), imgp.reshape(-1, 2), (corners, ids)))
    return out


def project(K, dist, T_cam_obj, pts_obj):
    rvec = cv2.Rodrigues(T_cam_obj[:3, :3])[0]
    tvec = T_cam_obj[:3, 3].reshape(3, 1)
    p, _ = cv2.projectPoints(pts_obj, rvec, tvec, K, dist)
    return p.reshape(-1, 2)


def residuals(params, views, K, dist):
    X = np.eye(4)
    X[:3, :3] = rodrigues(params[:3])
    X[:3, 3] = params[3:6]
    Y = np.eye(4)
    Y[:3, :3] = rodrigues(params[6:9])
    Y[:3, 3] = params[9:12]
    Xi = inv(X)
    err = []
    for A, obj, imgp, _bi in views:
        T = Xi @ A @ Y
        err.append((project(K, dist, T, obj) - imgp).reshape(-1))
    return np.concatenate(err)


def levmar(views, K, dist, p0, iters=120, verbose=False):
    """Levenberg-Marquardt on a 12-parameter vector; numeric Jacobian is plenty at this size."""
    p = np.array(p0, dtype=float)
    r = residuals(p, views, K, dist)
    cost = float(r @ r)
    lam = 1e-3
    for it in range(iters):
        J = np.zeros((len(r), 12))
        for k in range(12):
            d = 1e-6 if k >= 3 else 1e-7
            p2 = p.copy()
            p2[k] += d
            J[:, k] = (residuals(p2, views, K, dist) - r) / d
        g = J.T @ r
        H = J.T @ J
        try:
            step = np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-12), -g)
        except np.linalg.LinAlgError:
            lam *= 10.0
            continue
        p_new = p + step
        r_new = residuals(p_new, views, K, dist)
        cost_new = float(r_new @ r_new)
        if cost_new < cost:
            p, r, cost = p_new, r_new, cost_new
            lam = max(lam * 0.3, 1e-9)
            if verbose and it % 10 == 0:
                print("    it %3d  rms %.3f px" % (it, math.sqrt(cost / len(r))))
        else:
            lam *= 10.0
            if lam > 1e6:
                break
    return p, math.sqrt(cost / len(r))


def solve_one(views, K, dist, board_centre):
    """Closed-form init + BA. Returns (X, Y, rms_px)."""
    R_g2b = [v[0][:3, :3] for v in views]
    t_g2b = [v[0][:3, 3].reshape(3, 1) for v in views]
    R_t2c = [v[3][0] for v in views]
    t_t2c = [v[3][1] for v in views]

    cands = []
    for label, args in (
        ("gripper2base/target2cam", (R_g2b, t_g2b, R_t2c, t_t2c)),
    ):
        try:
            R_c2b, t_c2b = cv2.calibrateHandEye(args[0], args[1], args[2], args[3],
                                                method=cv2.CALIB_HAND_EYE_PARK)
            X = np.eye(4)
            X[:3, :3] = R_c2b
            X[:3, 3] = np.array(t_c2b).reshape(3)
            cands.append((label, X))
        except cv2.error as exc:
            print("  closed form %s failed: %s" % (label, exc))
    # eye-to-hand: the same call with the pair swapped returns cam->base; try its inverse too
    try:
        R_g2c, t_g2c = cv2.calibrateHandEye(R_t2c, t_t2c, R_g2b, t_g2b,
                                            method=cv2.CALIB_HAND_EYE_PARK)
        X = np.eye(4)
        X[:3, :3] = R_g2c
        X[:3, 3] = np.array(t_g2c).reshape(3)
        cands.append(("swapped->inv", inv(X)))
    except cv2.error:
        pass
    if not cands:
        raise SystemExit("closed form gave nothing")

    best = None
    for label, X in cands:
        # Y from the first view: Y = A^-1 X B  (B = board pose in camera)
        A = views[0][0]
        B = np.eye(4)
        B[:3, :3], B[:3, 3] = views[0][3][0], views[0][3][1].reshape(3)
        Y = inv(A) @ X @ B
        p0 = np.concatenate([cv2.Rodrigues(X[:3, :3])[0].reshape(3), X[:3, 3],
                             cv2.Rodrigues(Y[:3, :3])[0].reshape(3), Y[:3, 3]])
        p, rms = levmar(views, K, dist, p0)
        if best is None or rms < best[0]:
            best = (rms, p, label)
    rms, p, label = best
    X = np.eye(4)
    X[:3, :3] = rodrigues(p[:3])
    X[:3, 3] = p[3:6]
    Y = np.eye(4)
    Y[:3, :3] = rodrigues(p[6:9])
    Y[:3, 3] = p[9:12]
    return X, Y, rms, label


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="docs/calibration/datasets/2026-11-09-so101")
    ap.add_argument("--intrinsics", default="docs/calibration/scene_cam_intrinsics.yaml")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    data = json.load(open(os.path.join(args.dir, "poses.json")))
    board = board_from(data["board"])
    K, dist = load_intrinsics(args.intrinsics)
    print("intrinsics: fx=%.2f fy=%.2f cx=%.2f cy=%.2f" % (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))

    found = detect(args.dir, data["poses"], board)
    print("board found in %d/%d frames" % (len(found), len(data["poses"])))

    views = []
    for pose, obj, imgp, (corners, ids) in found:
        ok, rvec, tvec = cv2.solvePnP(obj, imgp, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        R = cv2.Rodrigues(rvec)[0]
        views.append((np.array(pose["fk_base_jaw"], dtype=float), obj, imgp, (R, tvec.reshape(3, 1))))
    print("PnP ok for %d frames" % len(views))

    board_centre = np.mean(views[0][1], axis=0)
    X, Y, rms, label = solve_one(views, K, dist, board_centre)
    print("\njoint bundle adjustment (%s): reprojection rms %.3f px over %d corners"
          % (label, rms, sum(len(v[2]) for v in views)))
    print("X = base <- cam: t = [%.1f, %.1f, %.1f] mm"
          % tuple(X[:3, 3] * 1000.0))
    z_cam_in_base = X[:3, 2]          # the camera's optical axis, expressed in base
    print("  camera +z (viewing axis) in base: [%.2f, %.2f, %.2f]  (should point DOWN, z<0)"
          % tuple(z_cam_in_base))

    # ---------------------------------------------------------- leave one out
    print("\nleave-one-out (solve on %d views, predict the one left out):" % (len(views) - 1))
    px, mm = [], []
    for i in range(len(views)):
        train = views[:i] + views[i + 1:]
        try:
            Xi, Yi, _, _ = solve_one(train, K, dist, board_centre)
        except SystemExit:
            continue
        A, obj, imgp, (R_b, t_b) = views[i]
        T = inv(Xi) @ A @ Yi
        pred = project(K, dist, T, obj)
        err_px = float(np.sqrt(np.mean(np.sum((pred - imgp) ** 2, axis=1))))
        px.append(err_px)
        # board centre in base, from the prediction vs from the measurements
        c_base_pred = (Xi @ T)[:3, :3] @ board_centre + (Xi @ T)[:3, 3]
        R_meas = A @ Y     # what the arm says the board is doing
        c_base_meas = R_meas[:3, :3] @ board_centre + R_meas[:3, 3]
        mm.append(float(np.linalg.norm(c_base_pred - c_base_meas)) * 1000.0)
    if px:
        print("  reprojection: median %.2f px  max %.2f px" % (np.median(px), max(px)))
        print("  board centre in base: median %.1f mm  max %.1f mm"
              % (np.median(mm), max(mm)))
    print("\nSUMMARY  rms %.3f px | hold-out %.2f px / %.1f mm (median) | %d views"
          % (rms, np.median(px) if px else -1, np.median(mm) if mm else -1, len(views)))

    if args.out:
        R, t = X[:3, :3], X[:3, 3]
        with open(args.out, "w") as fh:
            fh.write("# base <- scene_cam  (eye-to-hand; board on the gripper)\n")
            fh.write("# solver: tools/handeye_bundle.py — closed form + joint bundle adjustment\n")
            fh.write("# reprojection rms %.3f px over %d views; leave-one-out %.2f px / %.1f mm\n"
                     % (rms, len(views), np.median(px) if px else -1, np.median(mm) if mm else -1))
            fh.write("parent: base\nchild: scene_cam_link\n")
            fh.write("translation: [%.6f, %.6f, %.6f]\n" % (t[0], t[1], t[2]))
            for i in range(3):
                fh.write("rotation_row%d: [%.8f, %.8f, %.8f]\n" % (i, R[i, 0], R[i, 1], R[i, 2]))
            fh.write("residual_px: %.3f\n" % rms)
            fh.write("holdout_px: %.3f\nholdout_mm: %.3f\n"
                     % (np.median(px) if px else -1, np.median(mm) if mm else -1))
        print("wrote", args.out)


if __name__ == "__main__":
    main()

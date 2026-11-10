"""Solve the colour -> depth-view homography from the ChArUco board.

The two cameras are rigidly mounted and both see a plane (the board), so a single
homography relates their pixel grids — no intrinsics, no rectification model, nothing
to assume. The board's 12 markers give 48 correspondences, which is plenty.

Writes docs/calibration/color_to_depth.yaml and prints the residual.
"""

import json
import os
import sys

import cv2
import numpy as np

SRC = sys.argv[1] if len(sys.argv) > 1 else "/tmp/oak_reg"
OUT = sys.argv[2] if len(sys.argv) > 2 else None

board = cv2.aruco.CharucoBoard((5, 5), 0.018, 0.0135,
                               cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
params = cv2.aruco.DetectorParameters()
params.adaptiveThreshWinSizeMin, params.adaptiveThreshWinSizeMax = 5, 35
params.adaptiveThreshWinSizeStep, params.adaptiveThreshConstant = 5, 7
params.minMarkerPerimeterRate = 0.02
params.polygonalApproxAccuracyRate = 0.05
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
adet = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)


def markers(path):
    img = cv2.imread(path)
    if img is None:
        sys.exit("cannot read " + path)
    gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mk, ids, _ = adet.detectMarkers(gray)
    return img, mk, (None if ids is None else ids.ravel())


col, mk_c, ids_c = markers(os.path.join(SRC, "color.png"))
mono, mk_r, ids_r = markers(os.path.join(SRC, "right.png"))
print("colour %dx%d: %d markers | right %dx%d: %d markers"
      % (col.shape[1], col.shape[0], 0 if mk_c is None else len(mk_c),
         mono.shape[1], mono.shape[0], 0 if mk_r is None else len(mk_r)))

if mk_c is None or mk_r is None:
    sys.exit("the board has to be found in both images")

c_lut = {int(i): m.reshape(-1, 2) for m, i in zip(mk_c, ids_c)}
r_lut = {int(i): m.reshape(-1, 2) for m, i in zip(mk_r, ids_r)}
common = sorted(set(c_lut) & set(r_lut))
if len(common) < 3:
    sys.exit("only %d markers in common — the board must be visible in both" % len(common))

src = np.vstack([c_lut[i] for i in common]).astype(np.float32)
dst = np.vstack([r_lut[i] for i in common]).astype(np.float32)
print("common markers %d -> %d point pairs" % (len(common), len(src)))

H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0, maxIters=5000, confidence=0.999)
if H is None:
    sys.exit("homography failed")
inl = mask.ravel().astype(bool)
proj = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
err = np.linalg.norm(proj - dst, axis=1)
print("inliers %d/%d | residual rms %.2f px | max %.2f px"
      % (inl.sum(), len(inl), float(np.sqrt(np.mean(err[inl] ** 2))), float(err[inl].max())))

# sanity: the board must land in the same relative place and be smaller (wider FOV)
cu = src[:, 0].mean(), src[:, 1].mean()
ru = dst[:, 0].mean(), dst[:, 1].mean()
print("board centre: colour (%.0f, %.0f) -> right (%.0f, %.0f)" % (cu[0], cu[1], ru[0], ru[1]))
print("board width: colour %.0f px -> right %.0f px"
      % (np.ptp(src[:, 0]), np.ptp(dst[:, 0])))

if OUT:
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    def fmt(m):
        return "\n".join("  - [%s]" % ", ".join("%.10f" % v for v in row) for row in m)
    with open(OUT, "w") as fh:
        fh.write("# colour(1920x1080) -> depth-view(1280x720) homography, from the ChArUco board\n")
        fh.write("# solved %s with %d markers, residual rms %.2f px\n" % (
            __import__("datetime").date.today(), len(common), float(np.sqrt(np.mean(err[inl] ** 2)))))
        fh.write("# apply as: [u v 1]_depth ~ H @ [u v 1]_colour\n")
        fh.write("source: color\n")
        fh.write("target: depth\n")
        fh.write("matrix:\n%s\n" % fmt(H))
    print("wrote", OUT)

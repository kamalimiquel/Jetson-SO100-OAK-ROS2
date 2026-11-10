"""Depth scale check without any known object.

The board's pose in the colour camera comes from PnP (sub-pixel), which gives the
distance to every board pixel. The depth map gives its own answer at those same pixels,
reached through the colour->depth homography. Same surface, two independent
measurements: if they agree, the depth map is metric and needs no scale correction.

    ratio z_depth / z_pnp ~= 1.00  ->  scale is right
    constant ratio elsewhere       ->  that is OAK_SCENE_DEPTH_SCALE
"""

import json
import sys

import cv2
import numpy as np

OUR_K = np.array([[1571.5, 0, 940.8], [0, 1556.9, 539.6], [0, 0, 1]])
DIR = sys.argv[1] if len(sys.argv) > 1 else "/tmp/oak_capture"
H_PATH = sys.argv[2] if len(sys.argv) > 2 else "docs/calibration/color_to_depth.yaml"


def load_h(path):
    rows, inside = [], False
    for line in open(path):
        if line.strip().startswith("matrix:"):
            inside = True
            continue
        if inside:
            s = line.strip()
            if not s.startswith("- ["):
                break
            vals = [float(x) for x in s.split("[")[1].split("]")[0].split(",")]
            if len(vals) == 3:
                rows.append(vals)
    return np.array(rows, dtype=float)


H = load_h(H_PATH)
color = cv2.imread(DIR + "/color.png")
depth = np.load(DIR + "/depth.npy").astype(np.float32) / 1000.0
meta = json.load(open(DIR + "/calib.json"))
print("depth %dx%d" % (depth.shape[1], depth.shape[0]))

board = cv2.aruco.CharucoBoard((5, 5), 0.018, 0.0135,
                               cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
params = cv2.aruco.DetectorParameters()
params.adaptiveThreshWinSizeMin, params.adaptiveThreshWinSizeMax = 5, 35
params.adaptiveThreshWinSizeStep, params.adaptiveThreshConstant = 5, 7
params.minMarkerPerimeterRate = 0.02
params.polygonalApproxAccuracyRate = 0.05
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
adet = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)

gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
mk, ids, _ = adet.detectMarkers(gray)
if mk is None or len(mk) < 4:
    sys.exit("board not found in the colour image")

obj_all = board.getObjPoints()
lut = {int(mid): k for k, mid in enumerate(board.getIds().ravel())}
O, I = [], []
for corners, mid in zip(mk, ids.ravel()):
    k = lut.get(int(mid))
    if k is None:
        continue
    O.append(np.asarray(obj_all[k], dtype=np.float32).reshape(-1, 3))
    I.append(np.asarray(corners, dtype=np.float32).reshape(-1, 2))
obj_pts, img_pts = np.vstack(O), np.vstack(I)
ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, OUR_K, None, flags=cv2.SOLVEPNP_ITERATIVE)
R, _ = cv2.Rodrigues(rvec)
proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, OUR_K, None)
rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img_pts) ** 2, axis=1))))
print("%d corners | PnP centre %.3f m | rms %.2f px" % (len(img_pts), tvec[2, 0], rms))

# every board pixel, through the homography, gives a predicted depth from the plane
corners3d = (R @ obj_pts.T + tvec).T
n = R @ np.array([0.0, 0.0, 1.0])               # plane normal in the colour camera
d = float(n @ corners3d.mean(axis=0))
poly = cv2.convexHull(img_pts.astype(np.int32))
mask = np.zeros(gray.shape, dtype=np.uint8)
cv2.fillConvexPoly(mask, poly, 255)
mask = cv2.erode(mask, np.ones((7, 7), np.uint8))
vv, uu = np.nonzero(mask)
print("board pixels in colour: %d" % len(uu))

pts = np.stack([uu, vv, np.ones(len(uu))], axis=1).astype(np.float64)
mapped = (H @ pts.T).T
mapped = mapped[:, :2] / mapped[:, 2:3]
du = np.round(mapped[:, 0]).astype(int)
dv = np.round(mapped[:, 1]).astype(int)
inside = ((du >= 1) & (du < depth.shape[1] - 1) & (dv >= 1) & (dv < depth.shape[0] - 1))
du, dv = du[inside], dv[inside]
sel = (uu[inside], vv[inside])
print("mapped inside the depth frame: %d" % len(du))

zd = depth[dv, du]
valid = (zd > 0.05) & (zd < 2.0)
print("with valid depth: %d (%.0f%%)" % (valid.sum(), 100 * valid.sum() / max(1, len(zd))))
if valid.sum() < 200:
    sys.exit("not enough depth on the board — move it a little farther or nearer and retry")

# predicted depth for each pixel from the plane equation: n . X = d, with the pixel's
# own ray direction in the colour camera
fx, fy, cx, cy = OUR_K[0, 0], OUR_K[1, 1], OUR_K[0, 2], OUR_K[1, 2]
u = uu[inside][valid]
v = vv[inside][valid]
ray = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones(len(u))], axis=1)
z_pred = d / (ray @ n)                          # X = z * ray, n . (z ray) = d
z_meas = zd[valid]

ratio = z_meas / z_pred
keep = np.abs(ratio - np.median(ratio)) < 0.15    # drop the odd bad pixel
ratio, z_meas, z_pred = ratio[keep], z_meas[keep], z_pred[keep]
print("\nmedian depth (measured) %.1f mm | from PnP %.1f mm" % (1000 * np.median(z_meas), 1000 * np.median(z_pred)))
print("ratio z_depth / z_pnp : median %.4f  (p10 %.4f / p90 %.4f)  over %d px"
      % (np.median(ratio), np.percentile(ratio, 10), np.percentile(ratio, 90), len(ratio)))
print("error: median %+.0f mm, p10-p90 %+.0f..%+.0f mm"
      % (1000 * np.median(z_meas - z_pred),
         1000 * np.percentile(z_meas - z_pred, 10), 1000 * np.percentile(z_meas - z_pred, 90)))
print("\n=> 1.00 +- 2%%: metric, no correction. Otherwise OAK_SCENE_DEPTH_SCALE = %.4f" % np.median(ratio))

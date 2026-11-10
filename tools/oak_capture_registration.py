"""Capture for the colour <-> depth registration.

The production pipeline (colour 1080p + depth + rectified mono) exceeds what the USB
link carries at High Speed, which is why asking for the rectified stream failed with
X_LINK_ERROR. Here the depth XLinkOut is dropped — the registration only needs the two
IMAGES — and the rectified right view is streamed at the same 1280x720 grid the depth
map uses in production.

Writes /tmp/oak_reg/{color.png, right.png, calib.json}.
"""

import json
import os
import time

import cv2
import depthai as dai

OUT = os.environ.get("OAK_REG_DIR", "/tmp/oak_reg")


def main():
    os.makedirs(OUT, exist_ok=True)
    dev = dai.Device()
    calib = dev.readCalibration()
    kd = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, 1280, 720)
    meta = {
        "device": dev.getDeviceName(),
        "k_depth_cam_c": [[kd[0][0], 0.0, kd[0][2]], [0.0, kd[1][1], kd[1][2]], [0.0, 0.0, 1.0]],
        "baseline_cm": float(calib.getBaselineDistance()),
    }
    dev.close()

    p = dai.Pipeline()
    color = p.createColorCamera()
    color.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    color.setInterleaved(False)
    color.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    xc = p.createXLinkOut()
    xc.setStreamName("color")
    color.video.link(xc.input)

    left = p.createMonoCamera()
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    right = p.createMonoCamera()
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    sd = p.createStereoDepth()
    sd.setExtendedDisparity(True)
    sd.setLeftRightCheck(True)
    left.out.link(sd.left)
    right.out.link(sd.right)
    sd.setOutputRectified(True)          # off by default in 2.33
    xr = p.createXLinkOut()
    xr.setStreamName("right")
    sd.rectifiedRight.link(xr.input)

    dev2 = dai.Device(p)
    qc = dev2.getOutputQueue("color", 8, False)
    qr = dev2.getOutputQueue("right", 8, False)

    time.sleep(3.0)
    cols, rights = [], []
    t0 = time.time()
    while time.time() - t0 < 8:
        fc = qc.tryGet()
        if fc is not None:
            im = fc.getCvFrame()
            shar = float(cv2.Laplacian(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var())
            cols.append((fc.getTimestampDevice().total_seconds(), im, shar))
        fr = qr.tryGet()
        if fr is not None:
            m = fr.getCvFrame()
            if m is not None:
                rights.append((fr.getTimestampDevice().total_seconds(), m))
        time.sleep(0.01)
    dev2.close()

    if not cols or not rights:
        print("capture failed: %d colour, %d right" % (len(cols), len(rights)))
        return

    best = None
    for tc, im, shar in cols:
        for tr, m in rights:
            score = (abs(tc - tr), -shar)
            if best is None or score < best[0]:
                best = (score, im, m, shar, abs(tc - tr))
    _, img, mono, sharp, gap = best

    cv2.imwrite(os.path.join(OUT, "color.png"), img)
    cv2.imwrite(os.path.join(OUT, "right.png"), mono)
    with open(os.path.join(OUT, "calib.json"), "w") as fh:
        json.dump(meta, fh, indent=2)
    print("gap %.0f ms | sharpness %.0f | colour %dx%d | right %dx%d"
          % (gap * 1000, sharp, img.shape[1], img.shape[0], mono.shape[1], mono.shape[0]))


if __name__ == "__main__":
    main()

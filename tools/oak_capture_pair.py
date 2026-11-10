"""Capture one colour + one depth frame and the device calibration.

The cams image ships an old OpenCV whose aruco module has no CharucoBoard, so the
detection half runs on the host. This writes /tmp/oak_capture/:

    color.png      1920x1080 BGR
    depth.npy      1280x720 uint16, millimetres, CAM_C frame
    calib.json     K of CAM_C at the depth size, RGB->CAM_C extrinsic, stereo baseline
"""

import json
import os
import time

import cv2
import depthai as dai
import numpy as np

OUT = os.environ.get("OAK_CAPTURE_DIR", "/tmp/oak_capture")
W, H = 1280, 720


def main():
    os.makedirs(OUT, exist_ok=True)
    dev = dai.Device()
    calib = dev.readCalibration()
    kd = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, W, H)
    ext = np.array(calib.getCameraExtrinsics(dai.CameraBoardSocket.CAM_A,
                                            dai.CameraBoardSocket.CAM_C), dtype=float).reshape(4, 4)
    meta = {
        "device": dev.getDeviceName(),
        "mxid": dev.getMxId(),
        "usb_speed": str(dev.getUsbSpeed()),
        "depth_size": [W, H],
        "k_depth_cam_c": [[kd[0][0], 0.0, kd[0][2]], [0.0, kd[1][1], kd[1][2]], [0.0, 0.0, 1.0]],
        "rgb_to_cam_c": ext.tolist(),
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
    sd.setExtendedDisparity(True)   # same settings as the node, or we measure another camera
    sd.setLeftRightCheck(True)
    left.out.link(sd.left)
    right.out.link(sd.right)
    xd = p.createXLinkOut()
    xd.setStreamName("depth")
    sd.depth.link(xd.input)

    dev2 = dai.Device(p)
    qc = dev2.getOutputQueue("color", 2, False)
    qd = dev2.getOutputQueue("depth", 2, False)

    # The first frames after the pipeline starts are dark and motion-blurred:
    # auto-exposure has not settled. Skip a couple of seconds, then keep the
    # sharpest frame of a short burst — a blurry board simply does not detect.
    time.sleep(3.0)                    # let auto-exposure settle
    cols, deps = [], []
    t0 = time.time()
    while time.time() - t0 < 8:
        fc = qc.tryGet()
        if fc is not None:
            im = fc.getCvFrame()
            shar = float(cv2.Laplacian(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var())
            cols.append((fc.getTimestampDevice().total_seconds(), im, shar))
        fd = qd.tryGet()
        if fd is not None:
            d = fd.getFrame()
            if d is not None and d.max() > 0:
                deps.append((fd.getTimestampDevice().total_seconds(), d))
        time.sleep(0.01)
    dev2.close()

    if not cols or not deps:
        print("capture failed")
        return
    best = None
    for tc, im, shar in cols:
        for td, d in deps:
            score = (abs(tc - td), -shar)
            if best is None or score < best[0]:
                best = (score, im, d, shar, abs(tc - td))
    _, img, depth, sharp, gap = best
    print("colour/depth gap %.0f ms | sharpness %.0f" % (gap * 1000, sharp))

    cv2.imwrite(os.path.join(OUT, "color.png"), img)
    np.save(os.path.join(OUT, "depth.npy"), depth)
    with open(os.path.join(OUT, "calib.json"), "w") as fh:
        json.dump(meta, fh, indent=2)

    valid = depth[depth > 0]
    print("saved color %dx%d, depth %dx%d, valid depth px %d (%.0f%%)"
          % (img.shape[1], img.shape[0], depth.shape[1], depth.shape[0],
             valid.size, 100.0 * valid.size / depth.size))
    if valid.size:
        print("depth p05/p50/p95: %.0f / %.0f / %.0f mm"
              % tuple(np.percentile(valid.astype(np.float32), [5, 50, 95])))


if __name__ == "__main__":
    main()

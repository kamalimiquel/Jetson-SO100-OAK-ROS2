#!/usr/bin/env python
"""Two-camera OAK probe (depthai 2.x API — works with the pip wheel AND with
bindings built against depthai-core v2.33 from source).

1. Lists every depthai device with its MXID, name, firmware state and USB
   protocol (USB3 = X_LINK_USB_VSC, USB2 = X_LINK_USB2).
2. Opens each device and captures one 1080p color frame + one 720p stereo
   depth frame, printing resolution, fps and depth range. Saves frames to
   ~/oak_caps/<mxid>_color.jpg / <mxid>_depth.png.

Run on the Jetson:  <venv>/bin/python tools/oak_cams_probe.py
Exit 0 if every available device opened and captured.
"""

import os
import time

import cv2
import depthai as dai

OUT = os.path.expanduser("~/oak_caps")
os.makedirs(OUT, exist_ok=True)

PROTO = {v: k for k, v in dai.XLinkProtocol.__members__.items()}


def build_pipeline() -> dai.Pipeline:
    p = dai.Pipeline()
    c = p.createColorCamera()
    c.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    xc = p.createXLinkOut()
    xc.setStreamName("color")
    c.video.link(xc.input)

    l = p.createMonoCamera()
    l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    l.setBoardSocket(dai.CameraBoardSocket.LEFT)
    r = p.createMonoCamera()
    r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    r.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    sd = p.createStereoDepth()
    l.out.link(sd.left)
    r.out.link(sd.right)
    xd = p.createXLinkOut()
    xd.setStreamName("depth")
    sd.depth.link(xd.input)
    return p


def main() -> int:
    devices = dai.Device.getAllAvailableDevices()
    if not devices:
        print("NO DEVICES FOUND — check USB cabling and power")
        return 1
    print(f"found {len(devices)} device(s):")
    for info in devices:
        proto = PROTO.get(info.protocol, str(info.protocol))
        print(f"  {info.name} mxid={info.getMxId()} proto={proto} state={info.state.name}")

    ok = 0
    for info in devices:
        mxid = info.getMxId()
        print(f"\n== {info.name} ({mxid}) ==")
        try:
            t0 = time.time()
            n = 0
            last_c = None
            last_d = None
            with dai.Device(build_pipeline(), info) as dev:
                qc = dev.getOutputQueue("color")
                qd = dev.getOutputQueue("depth")
                while time.time() - t0 < 5:
                    cf = qc.tryGet()
                    df = qd.tryGet()
                    if cf is not None:
                        n += 1
                        last_c = cf.getCvFrame()
                    if df is not None:
                        last_d = df.getFrame()
            fps = n / (time.time() - t0)
            cv2.imwrite(f"{OUT}/{mxid}_color.jpg", last_c)
            dmax = int(last_d.max())
            d8 = (last_d * 255 // dmax).astype("uint8") if dmax > 0 else last_d
            cv2.imwrite(f"{OUT}/{mxid}_depth.png", d8)
            print(f"  color {last_c.shape[1]}x{last_c.shape[0]} | "
                  f"depth {last_d.shape[1]}x{last_d.shape[0]} | ~{fps:.1f} fps | "
                  f"depth {int(last_d.min())}-{dmax} (raw)")
            ok += 1
        except Exception as exc:  # noqa: BLE001
            print(f"  ERROR {type(exc).__name__}: {exc}")

    print(f"\nOK {ok}/{len(devices)}")
    return 0 if ok == len(devices) else 1


if __name__ == "__main__":
    raise SystemExit(main())

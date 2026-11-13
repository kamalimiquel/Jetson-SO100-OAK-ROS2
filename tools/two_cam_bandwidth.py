#!/usr/bin/env python3
"""Bandwidth test for two USB webcams sharing one hub.

Opens every given device at the same time, asks V4L2 for MJPG at the requested size and rate,
reads frames in one thread per camera for a fixed time and reports what each one actually got.
Run it on the Xavier; it needs opencv (both the system python and ~/lerobot/.venv have it).

    python3 tools/two_cam_bandwidth.py /dev/video0 /dev/video1 --w 640 --h 480 --fps 10 --secs 10
"""

import argparse
import threading
import time

import cv2

FOURCC = cv2.VideoWriter_fourcc(*"MJPG")


def run(dev, w, h, fps, secs, out, idx):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        out[idx] = {"dev": dev, "error": "cannot open"}
        return
    cap.set(cv2.CAP_PROP_FOURCC, FOURCC)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    real_fps = cap.get(cv2.CAP_PROP_FPS)
    got = fails = 0
    t0 = time.time()
    while time.time() - t0 < secs:
        ok, frame = cap.read()
        if ok and frame is not None:
            got += 1
        else:
            fails += 1
    dt = time.time() - t0
    cap.release()
    out[idx] = {"dev": dev, "size": "%dx%d" % (real_w, real_h), "asked_fps": real_fps,
                "frames": got, "failed_reads": fails, "secs": round(dt, 1),
                "fps": round(got / dt, 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("devices", nargs="+")
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=480)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--secs", type=float, default=10)
    args = ap.parse_args()

    out = {}
    threads = [threading.Thread(target=run, args=(d, args.w, args.h, args.fps, args.secs, out, i))
               for i, d in enumerate(args.devices)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    print("requested: MJPG %dx%d @ %d fps, %.0f s, all cameras at once"
          % (args.w, args.h, args.fps, args.secs))
    total = 0.0
    for i in range(len(args.devices)):
        r = out.get(i, {})
        if "error" in r:
            print("  %-14s %s" % (r["dev"], r["error"]))
            continue
        total += r["fps"] * (args.w * args.h * 0.10) / 8 / 1000 / 1000   # rough MJPG bitrate, Mbit/s
        print("  %-14s %s  fps asked %.0f got %.2f  frames %d  failed reads %d"
              % (r["dev"], r["size"], r["asked_fps"], r["fps"], r["frames"], r["failed_reads"]))
    print("  rough aggregate: %.1f Mbit/s of the 480 Mbit/s the hub shares" % total)


if __name__ == "__main__":
    main()

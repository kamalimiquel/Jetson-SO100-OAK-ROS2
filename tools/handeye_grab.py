#!/usr/bin/env python3
"""Grab ONE hand-eye sample now: current pose + camera frame + board check.

The operator moves the arm/board by hand, says "ok", and this runs. Nothing is
commanded: no joint is written, joint 6 is never touched.

    python3 handeye_grab.py            # add a sample
    python3 handeye_grab.py --status   # what is collected so far
"""

import argparse
import json
import os
import sys
import urllib.request

import roslibpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from handeye_capture import fk  # noqa: E402

BOARD = {"squares": 5, "square_m": 0.018, "marker_m": 0.0135, "dict": "DICT_4X4_50"}


def load(path):
    if os.path.exists(path):
        return json.load(open(path))
    return {"board": BOARD, "poses": []}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.expanduser("~/calib/scene"))
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
    det = cv2.aruco.CharucoDetector(
        cv2.aruco.CharucoBoard((5, 5), 0.018, 0.0135,
                               cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)))

    client = roslibpy.Ros(host=args.host, port=args.port)
    client.run()
    st = {"p": None}
    roslibpy.Topic(client, "/joint_states", "sensor_msgs/JointState").subscribe(
        lambda m: st.update(p=[float(x) for x in m["position"][:6]]))
    t0 = __import__("time").time()
    while st["p"] is None and __import__("time").time() - t0 < 10:
        __import__("time").sleep(0.2)
    if st["p"] is None:
        raise SystemExit("no /joint_states")
    q = list(st["p"])

    url = "http://%s:%d/snapshot" % (args.host, args.view)
    jpg = urllib.request.urlopen(url, timeout=10).read()
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
    c, _, _, _ = det.detectBoard(img)
    seen = 0 if c is None else len(c)

    name = "pose_%02d.jpg" % len(data["poses"])
    with open(os.path.join(args.out, name), "wb") as f:
        f.write(jpg)
    data["poses"].append({"image": name, "joints_deg": q, "fk_base_jaw": fk(q), "corners": seen})
    json.dump(data, open(pj, "w"), indent=1)
    client.terminate()

    verdict = "OK" if seen >= 12 else "weak — move the board so it is fully visible"
    print("saved %s  corners=%d/16  joints=%s  -> %s"
          % (name, seen, [round(v, 1) for v in q], verdict))
    print("%d samples total" % len(data["poses"]))


if __name__ == "__main__":
    main()

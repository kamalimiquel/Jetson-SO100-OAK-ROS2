#!/usr/bin/env python3
"""The owner's read surface for the agent: one snapshot, one detection answer.

Phase 3 of docs/design/mcp-endpoint-plan.md. Images never leave the robot: this process keeps the
freshest frame per camera, writes a snapshot under SNAPSHOT_DIR and returns its camera, stamp,
sha256 and observation id. The MCP endpoint only ever sees that JSON.

    GET  /cameras
    POST /snapshot {"camera": "scene"|"arm1_wrist"|"arm2_wrist"}
    POST /detect   {"camera": ...}

An unknown camera is refused with the list. A frame older than MAX_AGE_S is refused as STALE — a
picture of the past is worse than no picture. With no detector installed, /detect says MISSING
instead of inventing boxes.
"""
from __future__ import annotations

import hashlib
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

CAMERAS = {
    "scene": "/oak/color/image_raw/compressed",
    "arm1_wrist": "/arm1/gripper/image/compressed",
    "arm2_wrist": "/arm2/gripper/image/compressed",
}
PORT = int(os.environ.get("OWNER_PORT", "8098"))
SNAP_DIR = os.environ.get("SNAPSHOT_DIR", "/state/snapshots")
MAX_AGE_S = float(os.environ.get("SNAPSHOT_MAX_AGE_S", "2.0"))


class Freshest(Node):
    def __init__(self):
        super().__init__("owner_rpc")
        self.lock = threading.Lock()
        self.latest = {}
        self.counts = {c: 0 for c in CAMERAS}
        for cam, topic in CAMERAS.items():
            self.create_subscription(CompressedImage, topic,
                                     lambda m, c=cam: self._keep(c, m), 10)
        self.get_logger().info("owner_rpc: %s" % ", ".join(CAMERAS))

    def _keep(self, cam, msg):
        with self.lock:
            self.latest[cam] = msg

    def _dir(self):
        try:
            os.makedirs(SNAP_DIR, exist_ok=True)
            return SNAP_DIR
        except OSError:
            os.makedirs("/tmp/snapshots", exist_ok=True)
            return "/tmp/snapshots"

    def snapshot(self, cam):
        with self.lock:
            msg = self.latest.get(cam)
        if msg is None:
            return {"error": "MISSING: no frame on %s yet; is that camera up?" % cam}
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = time.time() - stamp if stamp else 0.0
        if age > MAX_AGE_S:
            return {"error": "STALE: %s last frame is %.1f s old (limit %.1f s)"
                             % (cam, age, MAX_AGE_S)}
        raw = bytes(msg.data)
        sha = hashlib.sha256(raw).hexdigest()
        with self.lock:
            self.counts[cam] += 1
            obs = "%s-%d" % (cam, self.counts[cam])
        path = os.path.join(self._dir(), "%s.jpg" % obs)
        with open(path, "wb") as fh:
            fh.write(raw)
        return {"camera": cam, "stamp": round(stamp, 6), "age_s": round(age, 3),
                "sha256": sha, "bytes": len(raw), "path": path, "format": msg.format or "",
                "observation_id": obs}

    def detect(self, cam):
        # No tracker is installed on this cell yet. Saying so is the honest answer.
        return {"camera": cam, "objects": [], "status": "MISSING",
                "observation_id": "%s-%d" % (cam, self.counts.get(cam, 0)),
                "detail": "no detector is configured on this cell yet — the grasp pipeline is not installed"}


def make_handler(node):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, payload):
            raw = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            if self.path.rstrip("/") in ("/cameras", ""):
                return self._send(200, {"cameras": sorted(CAMERAS)})
            return self._send(404, {"error": "GET /cameras, POST /snapshot or /detect"})

        def do_POST(self):
            path = self.path.rstrip("/")
            if path not in ("/snapshot", "/detect"):
                return self._send(404, {"error": "POST /snapshot or /detect"})
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n).decode() or "{}")
            except ValueError:
                return self._send(400, {"error": "body is not JSON"})
            cam = str((body or {}).get("camera") or "")
            if cam not in CAMERAS:
                return self._send(404, {"error": "unknown camera %r" % cam,
                                        "cameras": sorted(CAMERAS)})
            answer = node.snapshot(cam) if path == "/snapshot" else node.detect(cam)
            return self._send(200 if "error" not in answer else 409, answer)

    return Handler


def main():
    rclpy.init()
    node = Freshest()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), make_handler(node))
    print("owner_rpc: http://0.0.0.0:%d/cameras" % PORT, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

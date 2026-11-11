#!/usr/bin/env python3
"""Wrist-camera focus test (ADR-0022) — the bench tool, in the browser, on demand.

The question this answers is narrow: is the camera on the moving jaw sharp at the distance it
is going to work at (10-15 cm)? Cheap USB cameras are fixed focus and "fixed focus" says
nothing about where the plane really is, so the only way to know is to measure — put something
with texture at a known distance and read the sharpness.

It reuses what the cell already streams: it scores the frames that `wrist_cam` publishes on
/gripper/image/compressed, so the camera is opened once, by its own node, and this tool can
come and go without touching the device.

Sharpness is the variance of the Laplacian over the greyscale frame, scaled to 720 px wide so
the number means the same thing at any capture resolution. Blurry or flat scenes score near
zero; printed text at the right distance scores in the hundreds.

Cost discipline, because this runs on the same Xavier: the subscription exists only between
/start and /stop, and it stops itself when the page has not asked for a frame for 30 s — so
the tab costs nothing while it is closed, and nothing if the browser is killed.

Endpoints
    GET  /                the page
    GET  /stream          MJPEG from the newest scored frame
    GET  /status          {"score":…, "best":…, "brightness":…, "frames":…, "running":…}
    GET  /mark?d=10       append a row for that distance to the CSV, return the rows
    GET  /marks           the rows written so far
    POST /start           subscribe and begin scoring
    POST /stop            unsubscribe
"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import numpy as np

try:
    import cv2
except ImportError:                                        # pragma: no cover
    cv2 = None

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage

PORT = int(os.environ.get("WRIST_FOCUS_PORT", "8093"))
TOPIC = "/gripper/image/compressed"
IDLE_S = 30.0                     # stop scoring after this long with no client
CSV_PATH = os.environ.get("WRIST_FOCUS_CSV", "/opt/ws/cams/focus/wrist-focus.csv")
SCORE_WIDTH = 720                 # normalise the score across resolutions


class FocusState:
    """Everything the HTTP side needs, written by the ROS thread."""

    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None           # newest frame, as published
        self.score = 0.0
        self.best = 0.0
        self.brightness = 0.0
        self.frames = 0
        self.stamp = 0.0           # when the newest frame arrived
        self.last_client = 0.0     # when the page last asked for something
        self.running = False

    def snapshot(self):
        with self.lock:
            return {
                "score": round(self.score, 1),
                "best": round(self.best, 1),
                "brightness": round(self.brightness, 1),
                "frames": self.frames,
                "running": self.running,
                "age": round(time.time() - self.stamp, 2) if self.stamp else None,
            }


STATE = FocusState()


def sharpness(jpeg):
    """Variance of the Laplacian, grey, scaled to SCORE_WIDTH. Returns (score, brightness)."""
    if cv2 is None or not jpeg:
        return 0.0, 0.0
    buf = np.frombuffer(jpeg, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return 0.0, 0.0
    h, w = img.shape[:2]
    if w != SCORE_WIDTH:
        img = cv2.resize(img, (SCORE_WIDTH, max(1, int(round(h * SCORE_WIDTH / w)))),
                         interpolation=cv2.INTER_AREA)
    return float(cv2.Laplacian(img, cv2.CV_64F).var()), float(img.mean())


class FocusNode(Node):
    """Subscribes on demand — nothing is subscribed until /start."""

    def __init__(self):
        super().__init__("wrist_focus")
        qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._sub = None
        self._qos = qos

    def start(self):
        if self._sub is None:
            self._sub = self.create_subscription(CompressedImage, TOPIC, self._on_frame, self._qos)
            with STATE.lock:
                STATE.running = True
            self.get_logger().info("focus test: subscribed to %s" % TOPIC)

    def stop(self):
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
            with STATE.lock:
                STATE.running = False
            self.get_logger().info("focus test: unsubscribed")

    def _on_frame(self, msg):
        jpeg = bytes(msg.data)
        score, bright = sharpness(jpeg)
        with STATE.lock:
            STATE.jpeg = jpeg
            STATE.score = score
            STATE.brightness = bright
            STATE.best = max(STATE.best, score)
            STATE.frames += 1
            STATE.stamp = time.time()


NODE = None


def mark(distance_cm):
    """Write one row: the sharpness you are looking at, at the distance you say it is."""
    with STATE.lock:
        score, bright, frames = STATE.score, STATE.brightness, STATE.frames
    if frames == 0:
        return None
    os.makedirs(os.path.dirname(CSV_PATH), exist_ok=True)
    new = not os.path.exists(CSV_PATH)
    with open(CSV_PATH, "a") as fh:
        if new:
            fh.write("time,distance_cm,sharpness,brightness,frames\n")
        fh.write("%s,%.1f,%.2f,%.1f,%d\n"
                 % (time.strftime("%Y-%m-%d %H:%M:%S"), distance_cm, score, bright, frames))
    return {"time": time.strftime("%H:%M:%S"), "distance_cm": distance_cm,
            "sharpness": round(score, 1), "brightness": round(bright, 1)}


def rows():
    out = []
    try:
        with open(CSV_PATH) as fh:
            for line in fh.readlines()[1:]:
                parts = line.strip().split(",")
                if len(parts) >= 4:
                    out.append({"time": parts[0][11:], "distance_cm": float(parts[1]),
                                "sharpness": float(parts[2]), "brightness": float(parts[3])})
    except FileNotFoundError:
        pass
    return out[-24:]


PAGE = """<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>Wristcam focus test</title>
<style>
 :root { color-scheme: dark; }
 body { margin: 0; background: #10151c; color: #d8e1ec; font-family: system-ui, sans-serif;
        display: flex; gap: 18px; padding: 16px; height: 100vh; box-sizing: border-box; }
 .cam { flex: 1; display: flex; flex-direction: column; gap: 10px; min-width: 0; }
 img { width: 100%; height: 100%; object-fit: contain; background: #000; border-radius: 6px; }
 .side { width: 300px; display: flex; flex-direction: column; gap: 14px; }
 h1 { font-size: 15px; margin: 0; color: #9fb0c8; letter-spacing: .04em; text-transform: uppercase; }
 .score { font-size: 56px; font-weight: 700; line-height: 1; margin: 0; }
 .score small { font-size: 14px; font-weight: 400; color: #8d99ad; }
 .row { display: flex; justify-content: space-between; font-size: 13px; color: #9fb0c8; }
 button { padding: 9px; border: 0; border-radius: 6px; background: #2e3d4e; color: #d8e1ec;
          font-size: 14px; cursor: pointer; }
 button:hover { background: #3b4d61; }
 button.mark { background: #2b5d8a; }
 table { width: 100%; border-collapse: collapse; font-size: 12px; }
 th, td { text-align: right; padding: 3px 4px; border-bottom: 1px solid #22303f; }
 th:first-child, td:first-child { text-align: left; }
 .hint { font-size: 11.5px; color: #73808f; line-height: 1.45; }
</style></head><body>
<div class="cam"><img id="feed" src="/stream" alt="wrist camera"></div>
<div class="side">
  <div><h1>Sharpness</h1><p class="score" id="score">—<small> best <span id="best">—</span></small></p></div>
  <div class="row"><span>brightness</span><span id="bright">—</span></div>
  <div class="row"><span>frames scored</span><span id="frames">—</span></div>
  <button class="mark" onclick="mark(10)">Mark at 10 cm</button>
  <button class="mark" onclick="mark(12.5)">Mark at 12.5 cm</button>
  <button class="mark" onclick="mark(15)">Mark at 15 cm</button>
  <table><thead><tr><th>time</th><th>cm</th><th>sharp</th></tr></thead><tbody id="rows"></tbody></table>
  <p class="hint">Hold the textured target at the distance you press, wait for the number to
  settle, then mark. Sharp is hundreds; a flat or dark scene sits near zero. Only the newest
  frame is scored — this page is what starts the work, and closing it stops it.</p>
</div>
<script>
async function tick() {
  try {
    const s = await (await fetch("/status", {cache: "no-store"})).json();
    document.getElementById("score").firstChild.nodeValue = s.score.toFixed(1);
    document.getElementById("best").textContent = s.best.toFixed(0);
    document.getElementById("bright").textContent = s.brightness.toFixed(0);
    document.getElementById("frames").textContent = s.frames;
  } catch (e) {}
}
async function mark(d) {
  const r = await fetch("/mark?d=" + d);
  if (r.ok) drawRows(await r.json());
}
function drawRows(list) {
  document.getElementById("rows").innerHTML = list.slice().reverse().map(r =>
    `<tr><td>${r.time}</td><td>${r.distance_cm}</td><td>${r.sharpness.toFixed(1)}</td></tr>`).join("");
}
tick(); setInterval(tick, 700);
fetch("/marks").then(r => r.json()).then(drawRows).catch(() => {});
addEventListener("pagehide", () => { try { navigator.sendBeacon("/stop"); } catch (e) {} });
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):           # the tab polls twice a second: keep the log quiet
        pass

    # The tabs page lives on :80 and calls /start and /stop here on :8093, so the browser
    # treats those as cross-origin: without this it blocks the response (and the page cannot
    # tell whether the test actually started).
    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.send_header("Content-Length", "0")
        self.end_headers()

    def _send(self, code, body, ctype="application/json"):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        path = urlparse(self.path).path
        if path == "/start":
            STATE.last_client = time.time()
            if NODE:
                NODE.start()
            self._send(200, json.dumps({"running": True}))
        elif path == "/stop":
            if NODE:
                NODE.stop()
            self._send(200, json.dumps({"running": False}))
        else:
            self._send(404, json.dumps({"error": "no such endpoint"}))

    def do_GET(self):
        parsed = urlparse(self.path)
        path, qs = parsed.path, parse_qs(parsed.query)
        STATE.last_client = time.time()

        if path == "/":
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif path == "/status":
            self._send(200, json.dumps(STATE.snapshot()))
        elif path == "/marks":
            self._send(200, json.dumps(rows()))
        elif path == "/mark":
            try:
                d = float(qs.get("d", ["0"])[0])
            except ValueError:
                return self._send(400, json.dumps({"error": "d must be a number"}))
            row = mark(d)
            if row is None:
                return self._send(409, json.dumps({"error": "no frames yet"}))
            self._send(200, json.dumps(rows()))
        elif path == "/stream":
            self._stream()
        else:
            self._send(404, json.dumps({"error": "no such endpoint"}))

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self._cors()
        self.end_headers()
        last = 0.0
        while time.time() - STATE.last_client < IDLE_S:
            with STATE.lock:
                jpeg = STATE.jpeg
                stamp = STATE.stamp
            if jpeg is None:
                time.sleep(0.1)
                continue
            if stamp != last:
                last = stamp
                try:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    return
            time.sleep(0.02)


def idle_watchdog():
    """Nothing is left running because a tab was closed with the browser."""
    while True:
        time.sleep(5)
        if NODE and STATE.running and STATE.last_client:
            if time.time() - STATE.last_client > IDLE_S:
                NODE.stop()


def main():
    global NODE
    rclpy.init()
    NODE = FocusNode()
    threading.Thread(target=lambda: rclpy.spin(NODE), daemon=True).start()
    threading.Thread(target=idle_watchdog, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    NODE.get_logger().info("focus test page on http://0.0.0.0:%d/ (starts on /start)" % PORT)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        NODE.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

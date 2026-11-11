#!/usr/bin/env python3
"""Wristcam focus test — the "Wristcam focus test" tab, on demand (:8093).

The idea is the same as RViz: nothing runs until the tab is opened. The page calls
POST /start when it is shown and POST /stop when it is left (plus a sendBeacon on
pagehide), and an idle watchdog stops the work if the browser disappears without saying
goodbye — a forgotten tab must not keep the Xavier busy.

What runs while it is on: a subscriber to `/gripper/image/compressed`. It does NOT open
`/dev/so100_wrist`: the wrist node in `so100-cams` owns the device, and a second opener
would fight it. Reading the published topic also means the number measured here is the
sharpness of exactly what ROS and RViz see.

The score is the variance of the Laplacian of the grey image, computed on the newest
frame a few times a second — the same arithmetic as `tools/camfocus.py`, so the numbers
from the bench CSVs stay comparable. It is a *relative* number: compare it between
distances in one sitting, never against another day or another lamp.

Endpoints
    GET  /               the page (live image, score, distance buttons, marks)
    GET  /status         {running, score, best, brightness, frames, marks, ...}
    GET  /cam.mjpg       the live MJPEG stream
    GET  /mark?d=12.5    append one row to the CSV and return it
    GET  /health         liveness
    POST /start          begin subscribing + scoring (idempotent)
    POST /stop           stop (idempotent)

Usage
    python3 wrist_focus.py                    # for real, inside so100-web
    python3 wrist_focus.py --demo             # synthetic frames, no ROS: tests the page
    python3 wrist_focus.py --port 8093 --csv /srv/state/wristcam-focus.csv
"""

import argparse
import csv
import io
import json
import os
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import numpy as np
    from PIL import Image
    HAVE_SCORER = True
except Exception:                                    # noqa: BLE001 — reported in /status
    HAVE_SCORER = False

TOPIC = "/gripper/image/compressed"
CSV_HEADER = ["time", "distance_cm", "sharpness", "brightness", "frames", "source"]
MIN_SCORE_INTERVAL = 0.30            # score at ~3 Hz: 10 Hz of JPEG decode buys nothing
IDLE_TIMEOUT = 45.0                  # seconds without a page poll before we stop ourselves

lock = threading.Lock()
state = {
    "run": False,
    "worker": None,
    "score": 0.0,
    "best": 0.0,
    "brightness": 0.0,
    "frames": 0,                     # frames received
    "scored": 0,                     # frames scored
    "last_score_t": 0.0,
    "last_frame_t": 0.0,
    "frame": b"",                    # newest complete JPEG
    "source": "-",                   # "ros", "demo" or "-"
    "error": "",
    "last_seen": time.time(),        # last time the page asked for something
    "marks": [],
    "recent": [],                    # last scores, so a mark is a median and not one frame
}


# --------------------------------------------------------------------------- scoring
def score_jpeg(jpg):
    """(sharpness, brightness) of one JPEG, or (None, None) if it cannot be decoded.

    Variance of the 5-point Laplacian on the grey image, resized to at most 720 px — the
    same measure as tools/camfocus.py, so the numbers are comparable between the two.
    """
    if not HAVE_SCORER:
        return None, None
    try:
        img = Image.open(io.BytesIO(jpg)).convert("L")
        img.thumbnail((720, 720))
        a = np.asarray(img, dtype=np.float32)
        if a.shape[0] < 3 or a.shape[1] < 3:
            return None, None
        lap = (-4.0 * a[1:-1, 1:-1] + a[:-2, 1:-1] + a[2:, 1:-1]
               + a[1:-1, :-2] + a[1:-1, 2:])
        return float(lap.var()), float(a.mean())
    except Exception:                                # noqa: BLE001
        return None, None


def ingest(jpg):
    """Take one frame: keep it for the stream, score it now and then."""
    now = time.time()
    with lock:
        state["frame"] = jpg
        state["frames"] += 1
        state["last_frame_t"] = now
        due = now - state["last_score_t"] >= MIN_SCORE_INTERVAL
    if not due:
        return
    v, bright = score_jpeg(jpg)
    with lock:
        state["last_score_t"] = now
        if v is None:
            return
        state["score"] = v
        state["brightness"] = bright
        state["scored"] += 1
        state["recent"].append(v)
        del state["recent"][:-10]
        if v > state["best"]:
            state["best"] = v


# --------------------------------------------------------------------------- sources
def ros_worker():
    """Subscribe to the wrist camera until asked to stop. This is the real source."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage

    rclpy.init()
    node = Node("wristcam_focus")
    node.create_subscription(CompressedImage, TOPIC,
                             lambda msg: ingest(bytes(msg.data)), 5)
    with lock:
        state["source"] = "ros"
        state["error"] = ""
    node.get_logger().info("wristcam focus: listening on %s" % TOPIC)
    try:
        while True:
            with lock:
                if not state["run"]:
                    break
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def demo_worker():
    """Synthetic frames sharp for a second, blurred the next — no ROS, no camera.

    Enough to exercise the page, the scoring and the CSV on a machine that has neither,
    which is how the tab was tested before the Xavier was reachable.
    """
    with lock:
        state["source"] = "demo"
        state["error"] = ""
    n = 0
    while True:
        with lock:
            if not state["run"]:
                return
        n += 1
        from PIL import ImageDraw, ImageFilter
        img = Image.new("L", (640, 480), 40)
        d = ImageDraw.Draw(img)
        for i, line in enumerate(["SO-101  12.5 cm  focus", "sharpness target",
                                  "0123456789  abcdef", "/dev/so100_wrist"]):
            d.text((40, 90 + i * 60), line, fill=220)
        if (n // 10) % 2:                             # two seconds sharp, two blurred
            img = img.filter(ImageFilter.GaussianBlur(3.5))
        buf = io.BytesIO()
        img.convert("RGB").save(buf, format="JPEG", quality=85)
        ingest(buf.getvalue())
        time.sleep(0.10)


def start(kind):
    with lock:
        if state["run"]:
            state["last_seen"] = time.time()
            return False
        state["run"] = True
        state["frames"] = state["scored"] = 0
        state["best"] = 0.0
        state["score"] = state["brightness"] = 0.0
        state["last_seen"] = time.time()
        target = demo_worker if kind == "demo" else ros_worker
        t = threading.Thread(target=target, name="focus-worker", daemon=True)
        state["worker"] = t
    t.start()
    return True


def stop():
    with lock:
        if not state["run"]:
            return False
        state["run"] = False
        worker = state["worker"]
        state["worker"] = None
        state["frame"] = b""
        state["source"] = "-"
    # the ROS worker notices within a spin_once timeout; do not block the HTTP thread
    if worker is not None and worker.is_alive():
        threading.Thread(target=worker.join, kwargs={"timeout": 3.0}, daemon=True).start()
    return True


def watchdog():
    while True:
        time.sleep(5)
        with lock:
            running = state["run"]
            idle = time.time() - state["last_seen"]
        if running and idle > IDLE_TIMEOUT:
            stop()


# --------------------------------------------------------------------------- csv
def mark(csv_path, distance, source):
    """Append one measurement and return the row."""
    # the median of the last scores, not the instant one: a single blurred frame must not
    # become the number that goes in the CSV
    recent = sorted(state["recent"]) or [state["score"]]
    sharp = recent[len(recent) // 2] if len(recent) % 2 else \
        0.5 * (recent[len(recent) // 2 - 1] + recent[len(recent) // 2])
    row = [time.strftime("%Y-%m-%d %H:%M:%S"), distance,
           round(sharp, 2), round(state["brightness"], 1), state["scored"], source]
    new = not os.path.exists(csv_path)
    os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
    with open(csv_path, "a", newline="") as fh:
        w = csv.writer(fh)
        if new:
            w.writerow(CSV_HEADER)
        w.writerow(row)
    with lock:
        state["marks"].append(row)
        del state["marks"][:-12]
    return row


# --------------------------------------------------------------------------- page
PAGE = """<!doctype html><meta charset=utf-8><title>wristcam focus test</title>
<style>
 body{margin:0;background:#07090d;color:#c9d3e0;font:13px/1.45 ui-monospace,monospace}
 header{padding:8px 12px;background:#0b1220;border-bottom:1px solid #1b2735}
 b{color:#3fb950}
 .wrap{display:flex;height:calc(100vh - 44px)}
 .view{flex:1;min-width:0;background:#000;display:grid;place-items:center}
 img{max-width:100%%;max-height:100%%}
 aside{width:320px;border-left:1px solid #1b2735;padding:12px;overflow:auto}
 .score{font-size:44px;color:#e6edf3;line-height:1}
 .lbl{color:#8d99ad;font-size:11px;letter-spacing:.06em;text-transform:uppercase}
 .row{margin:14px 0}
 button{width:100%%;margin:4px 0;padding:8px;background:#16202e;color:#c9d3e0;
   border:1px solid #26364a;border-radius:5px;font:inherit;cursor:pointer}
 button:hover{border-color:#3fb950;color:#fff}
 table{width:100%%;border-collapse:collapse;margin-top:6px;font-size:12px}
 td{padding:3px 4px;border-bottom:1px solid #1b2735}
 .hint{color:#8d99ad;font-size:11.5px;margin-top:10px}
 .off{color:#c0392b}
</style>
<header>wristcam focus test — <span id="src">starting…</span>
 &nbsp;·&nbsp; <b id="state">on demand</b></header>
<div class="wrap">
 <div class="view"><img id="vid" src="/cam.mjpg" alt="wrist camera"></div>
 <aside>
  <div class="lbl">sharpness (variance of Laplacian)</div>
  <div class="score" id="score">—</div>
  <div class="hint">best <b id="best">—</b> · brightness <span id="bright">—</span>
   · <span id="frames">0</span> frames</div>
  <div class="row">
   <div class="lbl">mark the distance you are measuring at</div>
   <button onclick="mark(10)">10 cm</button>
   <button onclick="mark(12.5)">12.5 cm</button>
   <button onclick="mark(15)">15 cm</button>
   <button onclick="mark(0)">other — type it</button>
  </div>
  <div class="row"><div class="lbl">marks this session</div>
   <table id="marks"><tr><td>—</td></tr></table></div>
  <div class="hint">Compare the numbers <em>within one sitting</em>: the score depends on
   the light and on what is in front of the lens. Move the arm, not the camera, to change
   distance. Nothing runs unless this tab is open.</div>
 </aside>
</div>
<script>
 let last = 0;
 function mark(d) {
   if (!d) { const v = prompt("distance in cm?"); if (!v) return; d = v; }
   fetch(`/mark?d=${encodeURIComponent(d)}`).then(r => r.json()).then(refresh).catch(()=>{});
 }
 function refresh(j) {
   if (!j) return;
   document.getElementById("score").textContent =
     j.running ? j.score.toFixed(1) : "—";
   document.getElementById("best").textContent =
     j.running ? j.best.toFixed(1) : "—";
   document.getElementById("bright").textContent =
     j.running ? j.brightness.toFixed(0) : "—";
   document.getElementById("frames").textContent = j.frames;
   document.getElementById("src").textContent =
     j.running ? (j.source === "ros" ? "/gripper/image/compressed" : "demo frames") : "idle";
   const st = document.getElementById("state");
   st.textContent = j.running ? "measuring" : "idle";
   st.className = j.running ? "" : "off";
   if (j.error) document.getElementById("src").textContent = j.error;
   const t = document.getElementById("marks");
   t.innerHTML = (j.marks && j.marks.length)
     ? j.marks.slice().reverse().map(m =>
         `<tr><td>${m[1]} cm</td><td>${m[2]}</td><td>${m[0].slice(11)}</td></tr>`).join("")
     : "<tr><td>—</td></tr>";
 }
 function poll() {
   fetch("/status", {cache:"no-store"}).then(r => r.json())
     .then(j => { last = Date.now(); refresh(j); }).catch(()=>{});
 }
 setInterval(poll, 1000); poll();
</script>
"""


class Handler(BaseHTTPRequestHandler):
    server_version = "wristcam-focus/1.0"
    demo = False
    csv_path = "/srv/state/wristcam-focus.csv"

    def log_message(self, fmt, *args):               # keep the container log readable
        pass

    def _send(self, code, body, ctype="application/json", extra=None):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _status(self):
        with lock:
            return {
                "running": state["run"],
                "score": round(state["score"], 2),
                "best": round(state["best"], 2),
                "brightness": round(state["brightness"], 1),
                "frames": state["frames"],
                "scored": state["scored"],
                "source": state["source"],
                "error": state["error"],
                "have_scorer": HAVE_SCORER,
                "marks": list(state["marks"]),
            }

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        path = self.path.split("?")[0]
        q = urllib.parse.parse_qs(self.path.split("?")[1]) if "?" in self.path else {}
        with lock:
            state["last_seen"] = time.time()         # any poll keeps the watchdog quiet
        if path in ("/", "/index.html"):
            return self._send(200, PAGE, "text/html; charset=utf-8")
        if path == "/status":
            return self._send(200, json.dumps(self._status()))
        if path == "/health":
            return self._send(200, json.dumps({"ok": True}))
        if path == "/mark":
            d = (q.get("d") or ["0"])[0]
            with lock:
                src = state["source"]
                running = state["run"]
            if not running:
                return self._send(409, json.dumps({"error": "not running"}))
            row = mark(self.csv_path, d, src)
            return self._send(200, json.dumps({"row": row}))
        if path == "/cam.mjpg":
            return self._stream()
        return self._send(404, json.dumps({"error": "not found"}))

    def do_POST(self):
        path = self.path.split("?")[0]
        with lock:
            state["last_seen"] = time.time()
        if path == "/start":
            started = start("demo" if self.demo else "ros")
            return self._send(200, json.dumps({"running": True, "started": started}))
        if path == "/stop":
            stopped = stop()
            return self._send(200, json.dumps({"running": False, "stopped": stopped}))
        return self._send(404, json.dumps({"error": "not found"}))

    def _stream(self):
        """multipart/x-mixed-replace of the newest frame, with a boundary the browser finds."""
        boundary = "frame"
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=%s" % boundary)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        sent = b""
        try:
            while True:
                with lock:
                    jpg = state["frame"]
                    running = state["run"]
                    state["last_seen"] = time.time()
                if jpg and jpg is not sent:
                    self.wfile.write(b"--" + boundary.encode() + b"\r\n"
                                     b"Content-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n")
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    sent = jpg
                time.sleep(0.05 if running else 0.4)
        except (BrokenPipeError, ConnectionResetError):
            return


def main():
    ap = argparse.ArgumentParser(description="wristcam focus test (:8093, on demand)")
    ap.add_argument("--port", type=int, default=8093)
    ap.add_argument("--csv", default="/srv/state/wristcam-focus.csv")
    ap.add_argument("--demo", action="store_true",
                    help="synthetic frames instead of ROS — tests the page with no cell")
    args = ap.parse_args()

    Handler.demo = args.demo
    Handler.csv_path = args.csv
    if not HAVE_SCORER:
        with lock:
            state["error"] = "numpy/PIL missing: the score cannot be computed"

    threading.Thread(target=watchdog, name="focus-watchdog", daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print("wristcam focus test on http://:%d/  csv=%s  demo=%s" % (args.port, args.csv, args.demo),
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        stop()


if __name__ == "__main__":
    main()

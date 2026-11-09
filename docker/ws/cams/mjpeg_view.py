"""Live MJPEG view of the scene OAK-D inside the browser.

Subscribes to /oak/scene/color/compressed and re-serves the latest frame as a
multipart MJPEG stream, so the framing can be checked/aimed from any browser on
the LAN without touching ROS tooling:

    http://<jetson-ip>:8090/          page with the live image + resolution/age
    http://<jetson-ip>:8090/stream    the MJPEG stream itself
    http://<jetson-ip>:8090/snapshot  one JPEG, for grabbing stills

Python 3.8 (the image's own interpreter) — no external dependencies.
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

PORT = 8090
TOPIC = "/oak/scene/color/compressed"

_lock = threading.Lock()
_latest = {"jpeg": None, "stamp": 0.0, "w": 0, "h": 0, "fps": 0.0}
_count = {"n": 0, "t0": time.time()}


class Viewer(Node):
    def __init__(self):
        super().__init__("scene_cam_mjpeg")
        self.create_subscription(CompressedImage, TOPIC, self.on_img, 10)
        self.get_logger().info("serving %s on :%d" % (TOPIC, PORT))

    def on_img(self, msg):
        data = msg.data
        if not isinstance(data, (bytes, bytearray)):
            data = bytes(data)
        w = h = 0
        # JPEG SOF0/SOF2 carries the size; cheaper than decoding
        i = data.find(b"\xff\xc0")
        if i < 0:
            i = data.find(b"\xff\xc2")
        if i >= 0 and len(data) > i + 9:
            h = (data[i + 5] << 8) | data[i + 6]
            w = (data[i + 7] << 8) | data[i + 8]
        with _lock:
            _latest.update(jpeg=bytes(data), stamp=time.time(), w=w, h=h)
            _count["n"] += 1
            dt = time.time() - _count["t0"]
            if dt > 2.0:
                _latest["fps"] = _count["n"] / dt
                _count["n"], _count["t0"] = 0, time.time()


PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>scene OAK-D</title>
<style>
 body{margin:0;background:#0b1220;color:#e8edf5;font:14px system-ui,sans-serif}
 header{padding:10px 14px;background:#141b2a;border-bottom:1px solid #263042;
        display:flex;gap:16px;align-items:center}
 .stage{position:relative}
 img{display:block;width:100%;height:auto}
 .grid{position:absolute;inset:0;pointer-events:none}
 .grid .v,.grid .h{position:absolute;background:rgba(63,185,80,.45)}
 .grid .v{top:0;bottom:0;width:1px}
 .grid .h{left:0;right:0;height:1px}
 .grid .c{position:absolute;left:50%;top:50%;width:26px;height:26px;
          transform:translate(-50%,-50%);border:1px solid rgba(217,144,54,.9);border-radius:50%}
 .grid .box{position:absolute;left:20%;top:20%;width:60%;height:60%;
            border:1px dashed rgba(217,144,54,.55)}
 #meta{color:#8d99ad;font-variant-numeric:tabular-nums}
 b{color:#3fb950}
</style></head><body>
<header><b>scene OAK-D</b><span id="meta">waiting for frames…</span>
  <span style="color:#8d99ad">aim so the arm base and the workspace sit inside the dashed box</span>
  <span style="margin-left:auto"><a href="/snapshot" style="color:#d99036">snapshot</a></span></header>
<div class="stage">
  <img id="v" src="/stream">
  <div class="grid">
    <div class="v" style="left:33.3%"></div><div class="v" style="left:66.6%"></div>
    <div class="h" style="top:33.3%"></div><div class="h" style="top:66.6%"></div>
    <div class="box"></div><div class="c"></div>
  </div>
</div>
<script>
setInterval(()=>{fetch('/state').then(r=>r.json()).then(s=>{
  document.getElementById('meta').textContent =
    s.w+'x'+s.h+'  ·  '+s.fps.toFixed(1)+' fps  ·  frame '+s.age.toFixed(1)+' s old';
}).catch(()=>{});}, 1000);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/state"):
            with _lock:
                body = ('{"w":%d,"h":%d,"fps":%.2f,"age":%.2f}'
                        % (_latest["w"], _latest["h"], _latest["fps"],
                           time.time() - _latest["stamp"] if _latest["stamp"] else 0)).encode()
            self._send(200, "application/json", body)
        elif self.path.startswith("/snapshot"):
            with _lock:
                jpeg = _latest["jpeg"]
            if not jpeg:
                self._send(503, "text/plain", b"no frame yet")
            else:
                self._send(200, "image/jpeg", jpeg)
        elif self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last = None
            try:
                while True:
                    with _lock:
                        jpeg, stamp = _latest["jpeg"], _latest["stamp"]
                    if jpeg is None or stamp == last:
                        time.sleep(0.05)
                        continue
                    last = stamp
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n")
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self._send(200, "text/html; charset=utf-8", PAGE.encode())

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)


def main():
    rclpy.init()
    node = Viewer()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()

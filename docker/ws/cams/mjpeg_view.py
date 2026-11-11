"""Live MJPEG views of the scene OAK-D inside the browser.

Re-serves the latest JPEG of three streams so the framing can be checked from any
browser on the LAN without ROS tooling:

    http://<jetson-ip>:8090/               colour page, with the aiming grid
    http://<jetson-ip>:8090/stereo         the two rectified views plus the depth map
    http://<jetson-ip>:8090/<name>/stream  MJPEG stream (name = color | left | right)
    http://<jetson-ip>:8090/<name>/snapshot  one JPEG
    http://<jetson-ip>:8090/<name>/state     size, fps, age

The bare /stream, /snapshot and /state paths keep working and mean the colour camera.

Python 3.8 (the image's own interpreter) — no external dependencies.
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

PORT = 8090
STREAMS = {
    "color": "/oak/scene/color/compressed",
    "left": "/oak/scene/left/compressed",
    "right": "/oak/scene/right/compressed",
    # the colourised depth view published by oak_views — 5 Hz is plenty for a map
    "depth": "/oak/scene/depth/color/compressed",
}
DEFAULT = "color"

_lock = threading.Lock()
_frames = {name: {"jpeg": None, "stamp": 0.0, "w": 0, "h": 0, "fps": 0.0, "n": 0, "t0": time.time()}
           for name in STREAMS}


def _jpeg_size(data):
    """Read the width/height out of the JPEG SOF marker — cheaper than decoding."""
    i = data.find(b"\xff\xc0")
    if i < 0:
        i = data.find(b"\xff\xc2")
    if i < 0 or len(data) <= i + 9:
        return 0, 0
    h = (data[i + 5] << 8) | data[i + 6]
    w = (data[i + 7] << 8) | data[i + 8]
    return w, h


class Viewer(Node):
    def __init__(self):
        super().__init__("scene_cam_mjpeg")
        for name, topic in STREAMS.items():
            self.create_subscription(CompressedImage, topic,
                                     lambda msg, n=name: self.on_img(n, msg), 10)
        self.get_logger().info("serving %s on :%d" % (", ".join(STREAMS.values()), PORT))

    def on_img(self, name, msg):
        data = msg.data
        if not isinstance(data, (bytes, bytearray)):
            data = bytes(data)
        w, h = _jpeg_size(data)
        with _lock:
            f = _frames[name]
            f.update(jpeg=bytes(data), stamp=time.time(), w=w or f["w"], h=h or f["h"])
            f["n"] += 1
            dt = time.time() - f["t0"]
            if dt > 2.0:
                f["fps"] = f["n"] / dt
                f["n"], f["t0"] = 0, time.time()


COLOR_PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>scene OAK-D</title>
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
 a{color:#d99036}
</style></head><body>
<header><b>scene OAK-D</b><span id="meta">waiting for frames…</span>
  <span style="color:#8d99ad">aim so the arm base and the workspace sit inside the dashed box</span>
  <span style="margin-left:auto"><a href="/stereo">stereo</a> · <a href="/color/snapshot">snapshot</a></span></header>
<div class="stage">
  <img id="v" src="/color/stream">
  <div class="grid">
    <div class="v" style="left:33.3%"></div><div class="v" style="left:66.6%"></div>
    <div class="h" style="top:33.3%"></div><div class="h" style="top:66.6%"></div>
    <div class="box"></div><div class="c"></div>
  </div>
</div>
<script>
setInterval(()=>{fetch('/color/state').then(r=>r.json()).then(s=>{
  document.getElementById('meta').textContent =
    s.w+'x'+s.h+'  ·  '+s.fps.toFixed(1)+' fps  ·  frame '+s.age.toFixed(1)+' s old';
}).catch(()=>{});}, 1000);
</script></body></html>"""

STEREO_PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>OAK stereo pair</title>
<style>
 body{margin:0;background:#0b1220;color:#e8edf5;font:13px system-ui,sans-serif}
 header{padding:9px 14px;background:#141b2a;border-bottom:1px solid #263042;
        display:flex;gap:18px;align-items:center}
 .pair{display:grid;grid-template-columns:1fr 1fr;gap:6px;padding:6px}
 .pair .wide{grid-column:1 / span 2}
 .cam{position:relative;background:#0f1828;border:1px solid #263042;border-radius:6px;overflow:hidden}
 .cam img{display:block;width:100%;height:auto}
 .tag{position:absolute;left:8px;top:6px;padding:2px 7px;border-radius:4px;
      background:#0b1220cc;color:#3fb950;font-weight:700;letter-spacing:.04em}
 .tag.depth{color:#fbbf24}
 .base{position:absolute;right:8px;top:6px;padding:2px 7px;border-radius:4px;
       background:#0b1220cc;color:#8d99ad;font-variant-numeric:tabular-nums}
 b{color:#3fb950} a{color:#d99036}
</style></head><body>
<header><b>stereo pair &amp; depth</b>
  <span style="color:#8d99ad">the two rectified views the depth is computed from, and the depth map itself</span>
  <span style="margin-left:auto"><a href="/">colour</a></span></header>
<div class="pair">
  <div class="cam"><img src="/left/stream"><div class="tag">LEFT · CAM_B</div><div class="base" id="ml"></div></div>
  <div class="cam"><img src="/right/stream"><div class="tag">RIGHT · CAM_C</div><div class="base" id="mr"></div></div>
  <div class="cam wide"><img src="/depth/stream"><div class="tag depth">DEPTH · 0–2 m</div><div class="base" id="md"></div></div>
</div>
<script>
function poll(name, el){
  setInterval(()=>{fetch('/'+name+'/state').then(r=>r.json()).then(s=>{
    document.getElementById(el).textContent =
      s.w+'x'+s.h+' · '+s.fps.toFixed(1)+' fps'+(s.age > 3 ? ' · stale' : '');
  }).catch(()=>{});}, 1000);
}
poll('left', 'ml'); poll('right', 'mr'); poll('depth', 'md');
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _stream_and_verb(self):
        parts = [p for p in self.path.split("?")[0].split("/") if p]
        if not parts:
            return DEFAULT, "page"
        if parts[0] in STREAMS and len(parts) > 1:
            return parts[0], parts[1]
        if parts[0] in ("stream", "snapshot", "state"):
            return DEFAULT, parts[0]
        return DEFAULT, "page"

    def do_GET(self):
        path = self.path.split("?")[0]
        if path.rstrip("/") == "/stereo":
            self._send(200, "text/html; charset=utf-8", STEREO_PAGE.encode())
            return

        name, verb = self._stream_and_verb()
        with _lock:
            f = dict(_frames[name])

        if verb == "state":
            body = ('{"w":%d,"h":%d,"fps":%.2f,"age":%.2f}'
                    % (f["w"], f["h"], f["fps"],
                       time.time() - f["stamp"] if f["stamp"] else 0)).encode()
            self._send(200, "application/json", body)
        elif verb == "snapshot":
            if not f["jpeg"]:
                self._send(503, "text/plain", b"no frame yet")
            else:
                self._send(200, "image/jpeg", f["jpeg"])
        elif verb == "stream":
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
                        jpeg, stamp = _frames[name]["jpeg"], _frames[name]["stamp"]
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
            self._send(200, "text/html; charset=utf-8", COLOR_PAGE.encode())

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

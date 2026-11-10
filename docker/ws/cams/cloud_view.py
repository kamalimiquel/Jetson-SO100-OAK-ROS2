"""Browser viewer for the OAK point cloud.

RViz can draw a PointCloud2, but it is heavy on llvmpipe and awkward to check from
a laptop. This serves the newest cloud as a compact binary over plain HTTP and a
three.js page that renders it:

    http://<jetson-ip>:8091/          interactive 3D view
    http://<jetson-ip>:8091/cloud.bin  uint32 count + float32 x,y,z per point

The cloud is downsampled with a stride (default: keep every 3rd point) so a
720p cloud stays interactive over WiFi. Colour is derived from distance in the
viewer, so nothing extra is sent.
"""

import os
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

PORT = 8091
TOPIC = "/oak/scene/points"
STRIDE = 3          # keep every Nth point

_lock = threading.Lock()
_state = {"bin": b"", "n": 0, "stamp": 0.0, "fps": 0.0}
_count = {"n": 0, "t0": time.time()}


class CloudViewer(Node):
    def __init__(self):
        super().__init__("oak_cloud_view")
        self.create_subscription(PointCloud2, TOPIC, self.on_cloud, 3)
        self.get_logger().info("serving %s on :%d" % (TOPIC, PORT))

    def on_cloud(self, msg: PointCloud2):
        if msg.height * msg.width == 0:
            return
        fields = {f.name: f.offset for f in msg.fields}
        if not all(k in fields for k in ("x", "y", "z")):
            return
        buf = bytes(msg.data)
        step = msg.point_step
        # fast path: x,y,z are the first three float32s (oak_ros_node builds it that way)
        raw = np.frombuffer(buf, dtype=np.uint8)
        usable = (len(raw) // step) * step
        pts = np.frombuffer(raw[:usable].tobytes(), dtype=np.float32)
        per = step // 4
        pts = pts.reshape(-1, per)[:, :3]
        pts = pts[::STRIDE]
        pts = pts[np.isfinite(pts).all(axis=1)]
        # keep the working volume only: the OAK frame is optical (z forward), and the
        # far/noisy points stretch the bounding box until the cloud renders as dust
        pts = pts[(pts[:, 2] > 0.12) & (pts[:, 2] < 2.5)
                  & (np.abs(pts[:, 0]) < 1.5) & (np.abs(pts[:, 1]) < 1.5)]
        if len(pts) == 0:
            return
        payload = struct.pack("<I", len(pts)) + pts.astype("<f4").tobytes()
        with _lock:
            _state.update(bin=payload, n=len(pts), stamp=time.time())
            _count["n"] += 1
            dt = time.time() - _count["t0"]
            if dt > 2.0:
                _state["fps"] = _count["n"] / dt
                _count["n"], _count["t0"] = 0, time.time()


PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>OAK point cloud</title>
<style>
 html,body{margin:0;height:100%;background:#0b1220;overflow:hidden;font:13px system-ui,sans-serif;color:#e8edf5}
 #hud{position:fixed;top:10px;left:12px;padding:8px 12px;background:#141b2aee;border:1px solid #263042;border-radius:8px}
 #hud b{color:#3fb950} #hud span{color:#8d99ad}
</style></head><body>
<div id="hud"><b>OAK point cloud</b> <span id="info">cargando…</span></div>
<script src="/three.min.js"></script>
<script>
const scene = new THREE.Scene();
scene.fog = new THREE.FogExp2(0x0b1220, 0.9);
const cam = new THREE.PerspectiveCamera(55, innerWidth/innerHeight, 0.01, 20);
cam.position.set(0, -0.9, 0.55);
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(1);
document.body.appendChild(renderer.domElement);
const target = new THREE.Vector3(0, 0, 0.4);
let sph = {r:1.05, th:-1.2, ph:0.9};
function placeCam(){
  cam.position.set(
    target.x + sph.r*Math.sin(sph.ph)*Math.cos(sph.th),
    target.y + sph.r*Math.sin(sph.ph)*Math.sin(sph.th),
    target.z + sph.r*Math.cos(sph.ph));
  cam.lookAt(target);
}
placeCam();
let drag=false, px=0, py=0;
renderer.domElement.addEventListener('mousedown', e=>{drag=true; px=e.clientX; py=e.clientY;});
addEventListener('mouseup', ()=>drag=false);
addEventListener('mousemove', e=>{
  if(!drag) return;
  sph.th -= (e.clientX-px)*0.008; sph.ph -= (e.clientY-py)*0.008;
  sph.ph = Math.max(0.15, Math.min(3.0, sph.ph)); px=e.clientX; py=e.clientY; placeCam();
});
renderer.domElement.addEventListener('wheel', e=>{
  sph.r *= (1 + Math.sign(e.deltaY)*0.1); sph.r = Math.max(0.25, Math.min(4, sph.r)); placeCam(); e.preventDefault();
}, {passive:false});
scene.add(new THREE.GridHelper(2, 20, 0x263042, 0x1b2434));
scene.add(new THREE.AxesHelper(0.2));

let pts = null, geom = null;
function makeGeom(n){
  geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(n*3), 3));
  geom.setAttribute('color', new THREE.BufferAttribute(new Float32Array(n*3), 3));
  const mat = new THREE.PointsMaterial({size:0.012, vertexColors:true, sizeAttenuation:true});
  if (pts) scene.remove(pts);
  pts = new THREE.Points(geom, mat);
  scene.add(pts);
}
async function tick(){
  try{
    const r = await fetch('/cloud.bin', {cache:'no-store'});
    const buf = await r.arrayBuffer();
    if (buf.byteLength > 4){
      const dv = new DataView(buf);
      const n = dv.getUint32(0, true);
      if (!geom || geom.attributes.position.count !== n) makeGeom(n);
      const pos = geom.attributes.position.array, col = geom.attributes.color.array;
      // two passes: read positions, then colour by distance from the cloud centre
      // normalised to this frame, so contrast does not depend on scale
      let cx=0, cy=0, cz=0;
      for (let i=0;i<n;i++){
        const x = dv.getFloat32(4+i*12, true), y = dv.getFloat32(8+i*12, true), z = dv.getFloat32(12+i*12, true);
        pos[i*3]=x; pos[i*3+1]=y; pos[i*3+2]=z; cx+=x; cy+=y; cz+=z;
      }
      cx/=n; cy/=n; cz/=n;
      let dmin=1e9, dmax=1e-9;
      for (let i=0;i<n;i++){
        const d = Math.hypot(pos[i*3]-cx, pos[i*3+1]-cy, pos[i*3+2]-cz);
        if (d<dmin) dmin=d; if (d>dmax) dmax=d;
      }
      const span = Math.max(1e-3, dmax-dmin);
      for (let i=0;i<n;i++){
        const t = (Math.hypot(pos[i*3]-cx, pos[i*3+1]-cy, pos[i*3+2]-cz) - dmin)/span;
        col[i*3]   = Math.min(1, 0.10 + 1.75*t);
        col[i*3+1] = Math.min(1, 0.95 - 0.55*Math.abs(t-0.45)*2);
        col[i*3+2] = Math.max(0.10, 0.95 - 1.6*t);
      }
      geom.attributes.position.needsUpdate = true;
      geom.attributes.color.needsUpdate = true;
      geom.computeBoundingSphere();
      if (!window.__fitted && geom.boundingSphere){
        const bs = geom.boundingSphere;                 // frame the cloud once
        target.copy(bs.center);
        sph.r = Math.max(0.35, bs.radius * 1.25);
        placeCam(); window.__fitted = true;
      }
      document.getElementById('info').textContent = n.toLocaleString() + ' puntos';
    }
  }catch(e){ document.getElementById('info').textContent = 'sin datos'; }
  setTimeout(tick, 400);
}
tick();
addEventListener('resize', ()=>{ cam.aspect=innerWidth/innerHeight; cam.updateProjectionMatrix(); renderer.setSize(innerWidth,innerHeight); });
(function loop(){ requestAnimationFrame(loop); renderer.render(scene, cam); })();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/cloud.bin"):
            with _lock:
                payload = _state["bin"]
            if not payload:
                self.send_response(503); self.send_header("Content-Length", "0"); self.end_headers(); return
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)
        elif self.path.startswith("/three.min.js"):
            path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static", "three.min.js")
            try:
                body = open(path, "rb").read()
            except OSError:
                body = b""
            self.send_response(200 if body else 404)
            self.send_header("Content-Type", "application/javascript")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/state"):
            with _lock:
                body = ('{"n":%d,"fps":%.2f,"age":%.2f}'
                        % (_state["n"], _state["fps"],
                           time.time() - _state["stamp"] if _state["stamp"] else 0)).encode()
            self._send(body, "application/json")
        else:
            self._send(PAGE.encode(), "text/html; charset=utf-8")

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)


def main():
    rclpy.init()
    node = CloudViewer()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()

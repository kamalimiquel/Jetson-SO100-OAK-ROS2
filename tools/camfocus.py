#!/usr/bin/env python3
"""camfocus — check that a camera is actually in focus at a working distance (10-15 cm).

Live preview with a sharpness score, buttons that record the score at 10 / 12.5 / 15 cm into
a CSV, and (when the camera exposes it) manual focus control for a sweep.

  camfocus                          # /dev/video4 on http://127.0.0.1:8190/
  camfocus /dev/video2 --size 1920x1080
  camfocus --csv ~/focus.csv --port 8190

The score is the variance of the Laplacian of the grey image, computed on the newest frame
about twice a second. It is a *relative* number: compare it between distances or focus values
in one sitting, never against a number from another day or another lamp.

Only one process can hold the device, so stop anything else using it first.
"""

import argparse
import csv
import io
import json
import os
import queue
import re
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from PIL import Image

LOG = "/tmp/camfocus_ffmpeg.log"


def resolve_device():
    """The device node to open. With --follow, find it by card name every time, so a camera
    that comes back as /dev/video6 instead of /dev/video4 still works."""
    if not args.follow:
        return args.device
    import glob
    want = args.follow.lower()
    names = []
    for sysdir in sorted(glob.glob("/sys/class/video4linux/video*")):
        try:
            name = open(os.path.join(sysdir, "name")).read().strip()
        except OSError:
            continue
        node = "/dev/" + os.path.basename(sysdir)
        if not any(n == name for n, _ in names):      # one entry per card, not per node
            names.append((name, node))
    exact = [d for n, d in names if n.lower() == want]
    if exact:
        return exact[0]
    partial = [(n, d) for n, d in names if want in n.lower()]
    if len(partial) > 1:
        print("warning: %r matches several cameras %s — take the first; use a fuller name or "
              "the device path to be sure" % (args.follow, [n for n, _ in partial]), flush=True)
    return partial[0][1] if partial else None

args = None
clients = set()
lock = threading.Lock()
newest = {"jpg": None, "t": 0.0}
ring = {"buf": b""}
score_state = {"score": 0.0, "best": 0.0, "bright": 0.0, "n": 0, "t": 0.0}
history = deque(maxlen=90)


def card_name(dev):
    """The camera's card name, so rows from different cameras are never ambiguous."""
    try:
        return open("/sys/class/video4linux/%s/name" % os.path.basename(dev)).read().strip()
    except OSError:
        return "?"


def best_size(dev):
    """Largest size of the camera's own compressed format — MJPG or the older JPEG fourcc."""
    try:
        out = subprocess.run(["v4l2-ctl", "-d", dev, "--list-formats-ext"],
                             capture_output=True, text=True, timeout=6).stdout
    except Exception:  # noqa: BLE001
        return None
    groups, cur = {}, None
    for line in out.splitlines():
        m = re.search(r"\[\d+\]:\s+'(\w+)'", line)
        if m:
            cur = m.group(1)
            groups.setdefault(cur, [])
            continue
        m = re.search(r"Size: Discrete (\d+)x(\d+)", line)
        if m and cur:
            groups[cur].append((int(m.group(1)), int(m.group(2))))
    for fourcc in ("MJPG", "JPEG"):
        if groups.get(fourcc):
            w, h = max(groups[fourcc], key=lambda s: s[0] * s[1])
            return "%dx%d" % (w, h)
    return None


def device_focus_range(dev):
    """(min, max, value) of focus_absolute, or None when the camera is fixed-focus."""
    try:
        out = subprocess.run(["v4l2-ctl", "-d", dev, "--list-ctrls"],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:  # noqa: BLE001
        return None
    for line in out.splitlines():
        if "focus_absolute" in line:
            nums = {}
            for tok in line.split():
                if tok.startswith(("min=", "max=", "value=")):
                    k, v = tok.split("=")
                    nums[k] = int(v)
            if {"min", "max", "value"} <= nums.keys():
                return nums
    return None


def pump():
    while True:
        dev = resolve_device()
        if dev:
            break
        with open(LOG, "a") as fh:
            fh.write("%s waiting for a camera matching %r\n" % (time.strftime("%H:%M:%S"), args.follow))
        time.sleep(1.5)
    args.device = dev
    if args.size == "auto":
        args.size = best_size(dev) or "640x480"
    print("streaming from %s %s" % (dev, args.size), flush=True)
    cmd = [
        "ffmpeg", "-hide_banner", "-loglevel", "warning",
        "-f", "v4l2", "-input_format", "mjpeg", "-video_size", args.size,
    ] + (["-framerate", str(args.fps)] if args.fps else []) + [
        "-i", args.device,
        "-c:v", "copy", "-f", "mpjpeg", "-boundary_tag", "frame", "-",
    ]
    while True:
        with open(LOG, "a") as fh:
            fh.write("%s start %s %s\n" % (time.strftime("%H:%M:%S"), args.device, args.size))
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)

        def drain():
            for line in proc.stderr:
                with open(LOG, "a") as fh:
                    fh.write("%s   ffmpeg: %s\n" % (time.strftime("%H:%M:%S"),
                                                    line.decode("utf-8", "replace").strip()))

        threading.Thread(target=drain, daemon=True).start()
        while True:
            chunk = proc.stdout.read(16384)
            if not chunk:
                break
            with lock:
                ring["buf"] = (ring["buf"] + chunk)[-1_500_000:]
                dead = []
                for q in clients:
                    try:
                        q.put_nowait(chunk)
                    except queue.Full:
                        dead.append(q)
                for q in dead:
                    clients.discard(q)
        proc.wait()
        time.sleep(1.0)
        if not args.follow:
            continue
        while True:                      # re-resolve: it may come back on another node
            dev = resolve_device()
            if dev:
                args.device = dev
                break
            time.sleep(1.5)


def scorer():
    """Keep the newest complete JPEG and measure its sharpness a couple of times a second."""
    buf = b""
    while True:
        time.sleep(0.45)
        with lock:
            data = ring["buf"]
            # last *complete* frame: find the last end-of-image, then the start that belongs
            # to it. Searching the last SOI on its own usually lands in the half-received
            # frame at the tail of the buffer, which then looks like "no frame".
            e0 = data.rfind(b"\xff\xd9")
            s0 = data.rfind(b"\xff\xd8\xff", 0, e0) if e0 > 0 else -1
            if s0 < 0 or e0 <= s0:
                continue
            jpg = data[s0:e0 + 2]
            newest["jpg"] = jpg
            newest["t"] = time.time()
        if not jpg:
            continue
        try:
            img = Image.open(io.BytesIO(jpg)).convert("L")
            img.thumbnail((720, 720))
            a = np.asarray(img, dtype=np.float32)
        except Exception:  # noqa: BLE001
            continue
        lap = (-4.0 * a[1:-1, 1:-1] + a[:-2, 1:-1] + a[2:, 1:-1]
               + a[1:-1, :-2] + a[1:-1, 2:])
        v = float(lap.var())
        bright = float(a.mean())
        score_state["score"] = v
        score_state["bright"] = bright
        score_state["n"] += 1
        score_state["t"] = time.time()
        if v > score_state["best"]:
            score_state["best"] = v
        history.append((round(time.time(), 2), round(v, 1)))


def set_focus(value):
    subprocess.run(["v4l2-ctl", "-d", args.device, "-c", "focus_automatic_continuous=0"],
                   capture_output=True)
    return subprocess.run(["v4l2-ctl", "-d", args.device, "-c", "focus_absolute=%d" % value],
                          capture_output=True, text=True)


PAGE = """<!doctype html><meta charset=utf-8><title>camfocus %(dev)s</title>
<style>
 body{margin:0;background:#07090d;color:#c9d3e0;font:13px/1.45 ui-monospace,monospace}
 header{padding:8px 12px;background:#0b1220;border-bottom:1px solid #1b2735}
 b{color:#3fb950}
 .wrap{display:flex;height:calc(100vh - 44px)}
 .view{flex:1;min-width:0;background:#000;display:grid;place-items:center}
 img{max-width:100%%;max-height:100%%}
 aside{width:290px;border-left:1px solid #1b2735;padding:12px}
 .score{font-size:42px;color:#e6edf3;line-height:1}
 .lbl{color:#8d99ad;font-size:11px;letter-spacing:.06em;text-transform:uppercase}
 .row{margin:14px 0}
 button{width:100%%;margin:4px 0;padding:8px;background:#16202e;color:#c9d3e0;
   border:1px solid #26364a;border-radius:5px;font:inherit;cursor:pointer}
 button:hover{border-color:#3fb950;color:#fff}
 .bars{display:flex;align-items:flex-end;gap:2px;height:60px;margin-top:6px}
 .bars i{flex:1;background:#1f6feb;min-height:1px}
 .warn{color:#f0883e}
 @media (max-width:820px){            /* narrow tiled window: stack instead of squeeze */
   .wrap{flex-direction:column;height:auto}
   .view{height:54vh}
   aside{width:auto;border-left:0;border-top:1px solid #1b2735}
   .score{font-size:34px}
 }
</style>
<header><b>camfocus</b> · %(dev)s · %(size)s · %(focus)s</header>
<div class=wrap>
 <div class=view><img id=v src="/cam.mjpg"></div>
 <aside>
  <div class=lbl>sharpness (lap. variance)</div>
  <div class=score id=s>—</div>
  <div class=lbl style="margin-top:6px">best this session · <span id=b>—</span></div>
  <div class=bars id=h></div>
  <div class=row>
   <div class=lbl>mark the target distance</div>
   <button onclick="mark(10)">10 cm</button>
   <button onclick="mark(12.5)">12.5 cm</button>
   <button onclick="mark(15)">15 cm</button>
  </div>
  <div class=row>
   <div class=lbl>brightness <span id=br></span></div>
   <div id=msg style="color:#8d99ad;font-size:11px;margin-top:6px"></div>
  </div>
 </aside>
</div>
<script>
const S=v=>document.getElementById(v);
async function tick(){
  try{
    const s = await (await fetch('/status',{cache:'no-store'})).json();
    S('s').textContent = s.score ? s.score.toFixed(1) : '—';
    S('b').textContent = s.best ? s.best.toFixed(1) : '—';
    S('br').textContent = s.brightness ? s.brightness.toFixed(0) : '—';
    const mx = Math.max(...s.history.map(h=>h[1]), 1);
    S('h').innerHTML = s.history.map(h=>`<i style="height:${Math.round(h[1]/mx*58)}px"></i>`).join('');
  }catch(e){}
}
async function mark(d){
  const r = await (await fetch('/mark?d='+d,{cache:'no-store'})).json();
  S('msg').textContent = `marked ${d} cm → score ${r.score.toFixed(1)} (saved)`;
}
setInterval(tick, 700); tick();
</script>
"""


class H(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def json(self, obj, code=200):
        body = (obj if isinstance(obj, str) else json.dumps(obj)).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path
        if path.startswith("/status"):
            self.json({
                "score": score_state["score"], "best": score_state["best"],
                "brightness": score_state["bright"], "frames_scored": score_state["n"],
                "history": list(history)[-60:],
            })
            return
        if path.startswith("/mark"):
            dist = 0.0
            if "d=" in path:
                try:
                    dist = float(path.split("d=")[1].split("&")[0])
                except ValueError:
                    pass
            focus = device_focus_range(args.device)
            row = [time.strftime("%Y-%m-%d %H:%M:%S"), card_name(args.device), args.device,
                   args.size, dist, round(score_state["score"], 2),
                   round(score_state["bright"], 1), focus["value"] if focus else "fixed"]
            write_header = not os.path.exists(args.csv) or os.path.getsize(args.csv) == 0
            with open(args.csv, "a", newline="") as fh:
                w = csv.writer(fh)
                if write_header:
                    w.writerow(["time", "card", "device", "resolution", "distance_cm",
                                "sharpness", "brightness", "focus"])
                w.writerow(row)
            print("mark %4.1f cm  sharpness %7.1f  -> %s" % (dist, score_state["score"], args.csv),
                  flush=True)
            self.json({"distance_cm": dist, "score": score_state["score"]})
            return
        if path.startswith("/focus"):
            focus = device_focus_range(args.device)
            if not focus:
                self.json({"error": "camera has no focus_absolute control"}, 409)
                return
            q = path.split("?")[1] if "?" in path else ""
            val = focus["value"]
            if "v=" in q:
                try:
                    val = max(focus["min"], min(focus["max"], int(q.split("v=")[1].split("&")[0])))
                except ValueError:
                    pass
            r = set_focus(val)
            self.json({"focus": val, "ok": r.returncode == 0, "stderr": r.stderr[-200:]})
            return
        if path.startswith("/cam.mjpg"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            q = queue.Queue(maxsize=120)
            with lock:
                clients.add(q)
            started = False
            try:
                while True:
                    try:
                        chunk = q.get(timeout=15)
                    except queue.Empty:
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                        continue
                    if not started:
                        # a client that joins mid-frame gets a stream whose first bytes are
                        # the middle of a JPEG: Chromium shows a broken image. Wait for the
                        # next boundary and start from there.
                        cut = chunk.find(b"--frame")
                        if cut < 0:
                            continue
                        chunk = chunk[cut:]
                        started = True
                    self.wfile.write(chunk)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with lock:
                    clients.discard(q)
            return
        focus = device_focus_range(args.device)
        focus_txt = ("focus_absolute %d…%d, now %d" % (focus["min"], focus["max"], focus["value"])
                     if focus else "fixed focus (no focus_absolute control)")
        body = (PAGE % {"dev": args.device, "size": args.size, "focus": focus_txt}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    global args
    ap = argparse.ArgumentParser(description="Live focus check for a UVC camera")
    ap.add_argument("device", nargs="?", default="/dev/video4")
    ap.add_argument("--size", default="auto",
                    help="WxH, or auto (default) for the largest MJPG/JPEG size the camera offers")
    ap.add_argument("--fps", type=int, default=0,
                    help="force a frame rate; by default the camera's own is used")
    ap.add_argument("--port", type=int, default=8190)
    ap.add_argument("--csv", default=os.path.expanduser("~/camfocus.csv"))
    ap.add_argument("--follow", default="",
                    help='camera card name to track when it re-enumerates, e.g. --follow "Webcam"')
    args = ap.parse_args()

    focus = device_focus_range(args.device)
    where = ("waiting for a camera named %r" % args.follow if args.follow
             else "on %s (%s)" % (args.device,
                                  "focus %d…%d" % (focus["min"], focus["max"]) if focus
                                  else "fixed focus"))
    print("camfocus %s — http://127.0.0.1:%d/  csv=%s" % (where, args.port, args.csv), flush=True)
    threading.Thread(target=pump, daemon=True).start()
    threading.Thread(target=scorer, daemon=True).start()
    ThreadingHTTPServer(("127.0.0.1", args.port), H).serve_forever()


if __name__ == "__main__":
    main()

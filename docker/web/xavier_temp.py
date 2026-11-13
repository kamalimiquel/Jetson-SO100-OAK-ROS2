#!/usr/bin/env python3
"""Xavier NX Temp — the temperatures of the board, in a tab (:8094).

Reads `/sys/class/thermal/thermal_zone*` (that sysfs is the host kernel's, even from inside a
container) once a second and keeps the last few minutes so the page can draw a trace. No ROS, no
tegrastats, no device access: thirteen small file reads a second, which is nothing next to the
cameras — so unlike the focus tab this one simply always runs.

Zones on a Xavier NX are typically `CPU-therm`, `GPU-therm`, `SOC0-therm`, `SOC1-therm`,
`SOC2-therm`, `tj-therm` (the junction, the one that throttles), `CV0-therm`, `CV1-therm` and a
handful of board sensors. They are listed hottest first; the names are whatever the kernel calls
them, so nothing is hardcoded — a different Jetson just shows its own list.

Endpoints
    GET /            the page
    GET /temp        {"zones": [{"name", "temp", "peak"}...], "load": [...], "uptime": s, "ts": s}
    GET /history     {"series": [{"name", "points": [...]}...]} — the last N samples per zone
    GET /health      liveness

Usage
    python3 xavier_temp.py [--port 8094] [--interval 1.0] [--history 240]
"""

import argparse
import glob
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

STATE = {
    "zones": {},          # name -> {"temp": C, "peak": C, "points": [C, ...]}
    "ts": 0.0,
    "error": "",
}
LOCK = threading.Lock()

WARN_C = 60.0             # amber from here
HOT_C = 80.0              # red from here; the NX throttles around 95
HISTORY = 240             # samples kept per zone (4 min at 1 s)


def read_zones():
    """[(name, celsius)] for every thermal zone, ignoring the ones the kernel reports as 0."""
    out = []
    for path in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        try:
            with open(os.path.join(path, "type")) as fh:
                name = fh.read().strip()
            with open(os.path.join(path, "temp")) as fh:
                raw = int(fh.read().strip())
        except (OSError, ValueError):
            continue
        c = raw / 1000.0 if abs(raw) > 1000 else float(raw)     # millidegrees on Linux
        if c <= 0 or c > 150:
            continue
        out.append((name, c))
    return out


def sampler(interval, keep):
    while True:
        zones = read_zones()
        now = time.time()
        with LOCK:
            seen = set()
            for name, c in zones:
                # a duplicate type (two SOC sensors share a name on some boards) keeps both rows
                key = name
                n = 2
                while key in seen:
                    key = "%s #%d" % (name, n)
                    n += 1
                seen.add(key)
                z = STATE["zones"].setdefault(key, {"temp": c, "peak": c, "points": []})
                z["temp"] = c
                z["peak"] = max(z["peak"], c)
                z["points"].append(round(c, 1))
                del z["points"][:-keep]
            STATE["ts"] = now
            STATE["error"] = "" if zones else "no thermal zones in /sys/class/thermal"
        time.sleep(interval)


def loadavg():
    try:
        with open("/proc/loadavg") as fh:
            return [float(x) for x in fh.read().split()[:3]]
    except OSError:
        return []


def uptime():
    try:
        with open("/proc/uptime") as fh:
            return float(fh.read().split()[0])
    except OSError:
        return 0.0


PAGE = """<!doctype html><meta charset=utf-8><title>xavier nx temp</title>
<style>
 body{margin:0;background:#07090d;color:#c9d3e0;font:13px/1.45 ui-monospace,monospace}
 header{padding:10px 14px;background:#0b1220;border-bottom:1px solid #1b2735;display:flex;gap:18px;align-items:baseline}
 header b{color:#e6edf3}
 .hot{color:#f0883e}.crit{color:#f85149}.ok{color:#3fb950}
 .wrap{padding:14px;display:grid;grid-template-columns:1fr 420px;gap:14px}
 table{width:100%;border-collapse:collapse}
 td{padding:6px 8px;border-bottom:1px solid #16202e;vertical-align:middle}
 td.name{color:#c9d3e0;width:170px}
 td.val{text-align:right;font-size:17px;width:90px}
 .bar{height:9px;background:#16202e;border-radius:5px;overflow:hidden}
 .bar i{display:block;height:100%;background:#3fb950}
 .peak{color:#8d99ad;font-size:11px;text-align:right;width:80px}
 aside{background:#0b1220;border:1px solid #1b2735;border-radius:8px;padding:12px}
 canvas{width:100%;height:220px;display:block}
 .lbl{color:#8d99ad;font-size:11px;letter-spacing:.06em;text-transform:uppercase;margin-bottom:8px}
 .legend{color:#8d99ad;font-size:11.5px;margin-top:8px}
</style>
<header>
 <b>Xavier NX — temperatures</b>
 <span id="sum">—</span>
 <span class="legend" id="meta"></span>
</header>
<div class="wrap">
 <div><table id="tbl"><tr><td class="name">waiting…</td></tr></table></div>
 <aside>
  <div class="lbl">hottest zone, last 4 min</div>
  <canvas id="chart" width="820" height="440"></canvas>
  <div class="legend">green &lt; 60 °C · amber ≥ 60 °C · red ≥ 80 °C · the NX throttles near 95 °C.
   Values are the kernel's own zones, straight from /sys/class/thermal.</div>
 </aside>
</div>
<script>
 const WARN = 60, HOT = 80;
 let history = [], lastPts = [];
 function cls(t) { return t >= HOT ? "crit" : t >= WARN ? "hot" : "ok"; }
 function col(t) { return t >= HOT ? "#f85149" : t >= WARN ? "#f0883e" : "#3fb950"; }
 function draw() {
   const c = document.getElementById("chart"), g = c.getContext("2d");
   const w = c.width, h = c.height;
   g.clearRect(0, 0, w, h);
   g.strokeStyle = "#16202e"; g.lineWidth = 1;
   for (let t = 20; t <= 100; t += 20) {
     const y = h - (t - 20) / 80 * h;
     g.beginPath(); g.moveTo(0, y); g.lineTo(w, y); g.stroke();
     g.fillStyle = "#3d4a5c"; g.font = "18px monospace"; g.fillText(t + "°", 4, y - 4);
   }
   if (!lastPts.length) return;
   const n = lastPts.length;
   for (const s of history) {
     g.beginPath(); g.strokeStyle = s.colour; g.lineWidth = 2.5;
     s.points.forEach((v, i) => {
       const x = i / Math.max(1, n - 1) * (w - 10);
       const y = h - Math.max(0, Math.min(1, (v - 20) / 80)) * h;
       i ? g.lineTo(x, y) : g.moveTo(x, y);
     });
     g.stroke();
   }
 }
 async function tick() {
   try {
     const r = await fetch("/temp", {cache: "no-store"});
     const j = await r.json();
     const zones = j.zones || [];
     const tbl = document.getElementById("tbl");
     tbl.innerHTML = zones.map(z => {
       const pct = Math.max(2, Math.min(100, (z.temp - 20) / 80 * 100));
       return `<tr><td class="name">${z.name}</td>
         <td><div class="bar"><i style="width:${pct}%;background:${col(z.temp)}"></i></div></td>
         <td class="val ${cls(z.temp)}">${z.temp.toFixed(1)}°</td>
         <td class="peak">max ${z.peak.toFixed(1)}°</td></tr>`;
     }).join("") || `<tr><td class="name">${j.error || "no zones"}</td></tr>`;
     const hottest = zones[0];
     document.getElementById("sum").innerHTML = hottest
       ? `hottest: <span class="${cls(hottest.temp)}">${hottest.name} ${hottest.temp.toFixed(1)} °C</span>`
       : "—";
     const up = j.uptime || 0;
     document.getElementById("meta").textContent =
       `uptime ${Math.floor(up / 3600)}h ${String(Math.floor(up / 60) % 60).padStart(2, "0")}m`
       + ` · load ${(j.load || []).map(v => v.toFixed(2)).join(" ")}`;
     history = zones.slice(0, 3).map((z, i) => ({
       colour: ["#4b8bd6", "#f0883e", "#a374e0"][i] || "#8d99ad",
       points: z.points || [] }));
     lastPts = (zones[0] && zones[0].points) || [];
     draw();
   } catch (e) { /* the cell can be unreachable for a second; keep the last picture */ }
 }
 setInterval(tick, 1500); tick();
</script>
"""


class Handler(BaseHTTPRequestHandler):
    server_version = "xavier-temp/1.0"

    def log_message(self, fmt, *args):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/health":
            return self._json({"ok": True})
        with LOCK:
            zones = [{"name": k, "temp": v["temp"], "peak": v["peak"], "points": list(v["points"][-60:])}
                     for k, v in STATE["zones"].items()]
            zones.sort(key=lambda z: z["temp"], reverse=True)
            ts, err = STATE["ts"], STATE["error"]
        if path == "/temp":
            return self._json({"zones": zones, "ts": ts, "error": err,
                               "load": loadavg(), "uptime": round(uptime(), 0)})
        if path == "/history":
            with LOCK:
                series = [{"name": k, "points": list(v["points"])} for k, v in STATE["zones"].items()]
            return self._json({"series": series})
        return self._json({"error": "not found"}, 404)


def main():
    ap = argparse.ArgumentParser(description="Xavier NX temperatures (:8094)")
    ap.add_argument("--port", type=int, default=8094)
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--history", type=int, default=HISTORY)
    args = ap.parse_args()

    threading.Thread(target=sampler, args=(args.interval, args.history), daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print("xavier temp on http://:%d/  (%d zones)" % (args.port, len(read_zones())), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

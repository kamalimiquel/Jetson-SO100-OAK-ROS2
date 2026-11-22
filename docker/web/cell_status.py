#!/usr/bin/env python3
"""The Cell tab: who owns each arm, torque, fault, and which scale table is loaded.

The browser cannot read 127.0.0.1:8002 or :8003 — those are the container's loopback, not the operator's.
So this runs inside so100-web, asks, and serves the answer two ways:

    GET /          a small self-refreshing page (loaded in an iframe, like the focus and temp tabs)
    GET /status    the same data as JSON, for a script or a log

It never invents a value: an arm that does not answer is reported as unreachable with its error, and the
page shows it in red. Silence from the supervisor is normal — it is not started by entry.sh.

    python3 cell_status.py --port 8095
"""
from __future__ import annotations

import argparse
import json
import socket
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ARMS = [("so100_cell", 8002), ("so100_cell_b", 8003)]
SKILL_PORT = 8097        # supervisor HTTP status (read-only); see supervisor.py
TIMEOUT_S = 1.5


def get(port, path):
    try:
        with urllib.request.urlopen("http://127.0.0.1:%d%s" % (port, path), timeout=TIMEOUT_S) as r:
            return json.loads(r.read().decode("utf-8")), None
    except urllib.error.HTTPError as exc:
        return None, "HTTP %d: %s" % (exc.code, exc.read().decode("utf-8", "replace").strip()[:120])
    except (urllib.error.URLError, socket.timeout, ValueError) as exc:
        return None, "unreachable (%s)" % getattr(exc, "reason", exc)


def collect():
    arms = []
    for name, port in ARMS:
        row = {"arm": name, "port": port, "reachable": False}
        owner, err = get(port, "/owner")
        if err:
            row["error"] = err
            arms.append(row)
            continue
        row["reachable"] = True
        row["owner"] = (owner or {}).get("owner")
        row["lease_s"] = (owner or {}).get("seconds_left")
        tw, err = get(port, "/torque")
        row["torque"] = bool((tw or {}).get("torque")) if not err else None
        st, err = get(port, "/status")
        if err:
            row["error"] = err
        else:
            row["connected"] = (st or {}).get("connected")
            row["calibrated"] = (st or {}).get("calibrated")
        sc, err = get(port, "/scale")
        if not err and sc:
            row["scale"] = {"arm": sc.get("arm"), "applied": sc.get("applied"),
                            "sha256": (sc.get("sha256") or "")[:8], "session": sc.get("session")}
        arms.append(row)
    sup, err = get(SKILL_PORT, "/")
    return {"arms": arms,
            "supervisor": {"listening": err is None, "detail": sup if err is None else None,
                           "error": err},
            "all_reachable": all(a["reachable"] for a in arms)}


PAGE = """<!doctype html><meta charset="utf-8">
<title>Cell</title>
<style>
 :root { color-scheme: dark; }
 body { margin:0; padding:14px 16px; font:14px/1.45 system-ui, sans-serif; background:#14161a; color:#e8eaed; }
 h1 { font-size:15px; font-weight:600; margin:0 0 10px; letter-spacing:.02em; color:#c9ced6; }
 table { border-collapse:collapse; width:100%; }
 th, td { text-align:left; padding:7px 10px; border-bottom:1px solid #24272e; font-variant-numeric:tabular-nums; }
 th { font-weight:600; color:#98a2b3; font-size:12px; text-transform:uppercase; letter-spacing:.04em; }
 .no { color:#ff7b72; } .on { color:#7ee787; } .off { color:#98a2b3; }
 .note { margin-top:12px; color:#98a2b3; font-size:12.5px; }
 code { color:#d2a8ff; }
 .sup { margin-top:14px; border-top:1px solid #24272e; padding-top:10px; }
 .sup h2 { font-size:12px; text-transform:uppercase; letter-spacing:.04em; color:#98a2b3; margin:0 0 6px; font-weight:600; }
 .sup dl { display:grid; grid-template-columns:max-content 1fr; gap:3px 14px; margin:0; }
 .sup dt { color:#98a2b3; }
 .sup dd { margin:0; font-variant-numeric:tabular-nums; }
</style>
<h1>Cell — arms, ownership and scale</h1>
<table id="t"><thead><tr>
 <th>arm</th><th>port</th><th>state</th><th>lease</th><th>torque</th><th>scale</th><th>session</th>
</tr></thead><tbody></tbody></table>
<div class="sup">
 <h2>Supervisor</h2>
 <dl id="supdl"></dl>
</div>
<script>
const fmt = (d) => {
  const rows = d.arms.map((a) => {
    const state = a.reachable ? `<span class="on">up</span>` : `<span class="no">unreachable</span><br><small>${a.error||""}</small>`;
    const torque = (a.torque === null || a.torque === undefined) ? "—"
                 : a.torque ? `<span class="on">energised</span>` : `<span class="off">free</span>`;
    const sc = a.scale ? `<code>${a.scale.sha256}</code>${a.scale.applied ? "" : ` <span class="off">(not applied)</span>`}` : "—";
    return `<tr><td>${a.arm}</td><td>${a.port}</td><td>${state}</td><td>${a.owner ? `${a.owner} <span class="off">${Math.round(a.lease_s||0)}s</span>` : "—"}</td>
            <td>${torque}</td><td>${sc}</td><td>${a.scale ? a.scale.session : "—"}</td></tr>`;
  }).join("");
  document.querySelector("#t tbody").innerHTML = rows;
  const sup = d.supervisor;
  const dl = document.querySelector("#supdl");
  if (!sup.listening) {
    dl.innerHTML = `<dt>state</dt><dd><span class="off">not running</span> — normal unless a mission is up</dd>`
      + (sup.error ? `<dt>why</dt><dd class="off">${sup.error}</dd>` : "");
  } else {
    const x = sup.detail;
    const last = x.last && x.last.skill
      ? `${x.last.skill}${x.last.mission ? " · " + x.last.mission : ""}`
        + (x.last.at ? ` <span class="off">(${Math.round(Date.now()/1000 - x.last.at)}s ago)</span>` : "")
      : "—";
    dl.innerHTML = [
      ["state", `<span class="on">up</span>`],
      ["node", `<code>${x.node}</code> (${x.node_name})`],
      ["holds", (x.owned || []).join(", ") || "<span class='off'>nothing</span>"],
      ["lease ttl", `${x.ttl_s} s per arm`],
      ["mission", x.mission || "—"],
      ["skills served", x.served],
      ["last skill", last],
      ["uptime", `${Math.round(x.uptime_s)} s`],
    ].map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join("");
  }
};
const tick = () => fetch("/status", {cache:"no-store"}).then(r => r.json()).then(fmt).catch(() => {});
tick(); setInterval(tick, 2000);
</script>
"""


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        body = json.dumps(collect()).encode() if self.path.startswith("/status") else PAGE.encode()
        ctype = "application/json" if self.path.startswith("/status") else "text/html; charset=utf-8"
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8095)
    args = ap.parse_args()
    ThreadingHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""LAN-facing torque switch for the arm — the red button in the web page.

The joint server binds loopback only on purpose (127.0.0.1:8002), so the browser
cannot reach it. This tiny proxy exposes exactly one capability, on :8092:

    GET  /torque            -> {"torque": false}
    POST /torque {"on":true|false}  -> flips the servos' torque and reports the state

Nothing else is proxied: the page can switch torque, it cannot move the arm.

CORS is open (`*`) because the page is served from :80 and talks to :8092 — same
Jetson, different port, so the browser treats it as another origin.

Env (two-arm plan, Phase 1): TORQUE_PORT (default 8092; arm 2 uses 8095),
JOINT_PORT (default 8002; arm 2 uses 8003) — the loopback joint server to proxy.
"""

import json
import os
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HOST = "0.0.0.0"
PORT = int(os.environ.get("TORQUE_PORT", "8092"))
JOINT_PORT = os.environ.get("JOINT_PORT", "8002")
UPSTREAM = f"http://127.0.0.1:{JOINT_PORT}/torque"


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        raw = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def do_OPTIONS(self):
        self._send(204, b"")

    def do_GET(self):
        if self.path.split("?")[0] != "/torque":
            self._send(404, {"error": "not found"})
            return
        try:
            with urllib.request.urlopen(UPSTREAM, timeout=3.0) as r:
                self._send(200, r.read(), "application/json")
        except Exception as exc:  # noqa: BLE001
            self._send(503, {"error": f"joint server unreachable: {exc}"})

    def do_POST(self):
        if self.path.split("?")[0] != "/torque":
            self._send(404, {"error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            payload = self.rfile.read(length) or b"{}"
            json.loads(payload)                       # reject anything that is not JSON
            req = urllib.request.Request(UPSTREAM, data=payload, method="POST",
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=5.0) as r:
                self._send(200, r.read(), "application/json")
        except urllib.error.HTTPError as exc:
            self._send(exc.code, exc.read() or b"{}", "application/json")
        except Exception as exc:  # noqa: BLE001
            self._send(503, {"error": f"joint server unreachable: {exc}"})


if __name__ == "__main__":
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()

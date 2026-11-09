#!/usr/bin/env python3
"""Shutdown endpoint for the tabs page (port 8099, host network = LAN-visible).

POST /shutdown -> SSH to the Xavier host (127.0.0.1 = the host, since the web
container runs on host network) and run `sudo shutdown -h now`. The maju
password comes from /run/secrets/shutdown-secret (600-perm file mounted from
the host, never in the repo). GET /health returns 200 without doing anything.
"""

import json
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SECRET = "/run/secrets/shutdown-secret"


def read_password() -> str:
    with open(SECRET) as fh:
        return fh.read().strip()


def fire_shutdown():
    """Runs on a timer, after the HTTP response has been sent."""
    try:
        password = read_password()
        cmd = (
            f"sshpass -p '{password}' ssh -o StrictHostKeyChecking=no "
            f"-o UserKnownHostsFile=/dev/null -o ConnectTimeout=5 maju@127.0.0.1 "
            f"\"echo '{password}' | sudo -S -p '' shutdown -h now\""
        )
        subprocess.run(cmd, shell=True, capture_output=True, timeout=20)
    except Exception:  # noqa: BLE001
        pass


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):  # quiet
        pass

    def _cors(self, code=200):
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_OPTIONS(self):
        self._cors(204)

    def do_GET(self):
        if self.path == "/health":
            self._cors(200)
            self.wfile.write(json.dumps({"ok": True}).encode())
        else:
            self._cors(404)
            self.wfile.write(b'{"error": "not found"}')

    def do_POST(self):
        if self.path != "/shutdown":
            self._cors(404)
            self.wfile.write(b'{"error": "not found"}')
            return
        try:
            # Answer FIRST, then power off: the host dies mid-request otherwise
            # and the browser reports "Failed to fetch" for a shutdown that worked.
            threading.Timer(1.0, fire_shutdown).start()
            self._cors(200)
            self.wfile.write(json.dumps({
                "status": "ok",
                "output": "shutdown scheduled (2 s)",
            }).encode())
        except Exception as exc:  # noqa: BLE001
            self._cors(500)
            self.wfile.write(json.dumps({"status": "error", "output": str(exc)}).encode())


if __name__ == "__main__":
    ThreadingHTTPServer(("0.0.0.0", 8099), Handler).serve_forever()

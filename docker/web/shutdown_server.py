#!/usr/bin/env python3
"""Power and RViz endpoints for the tabs page (port 8099, host network = LAN-visible).

POST /shutdown -> SSH to the Xavier host (127.0.0.1 = the host, since the web
container runs on host network) and run `sudo shutdown -h now`. The maju
password comes from /run/secrets/shutdown-secret (600-perm file mounted from
the host, never in the repo). GET /health returns 200 without doing anything.

RViz is the most expensive thing in this cell — software GL on the Xavier's CPU. So it
is not started with the container: the tabs page asks for it when its tab is opened and
stops it when the tab is left.

    GET  /rviz/state  -> {"running": bool, "pid": int|null}
    POST /rviz/start  -> spawns rviz2 on display :10 (idempotent)
    POST /rviz/stop   -> SIGTERM to rviz2, the display stays up for the next start
"""

import json
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SECRET = "/run/secrets/shutdown-secret"
RVIZ_CMD = ("source /opt/ros/humble/install/setup.bash 2>/dev/null; "
            "DISPLAY=:10 LIBGL_ALWAYS_SOFTWARE=1 exec rviz2 -d /srv/web/rviz.rviz")
# RViz draws at most two compressed-image panels on this board, and there are three camera views
# with two arms: the two jaw cameras are composed into one frame (wrist_pair.py) while RViz runs.
PAIR_CMD = ("source /opt/ros/humble/install/setup.bash 2>/dev/null; "
            "exec python3 /srv/web/wrist_pair.py")
_rviz_lock = threading.Lock()


def rviz_pid():
    """PID of a running rviz2, or None. Matched on the binary, not on the launcher."""
    try:
        out = subprocess.run(["pgrep", "-f", "rviz2"], capture_output=True, timeout=5).stdout
    except Exception:  # noqa: BLE001
        return None
    pids = [int(p) for p in out.split() if p.strip()]
    return pids[0] if pids else None


def pair_pid():
    """PID of the two-wrist composer, or None."""
    try:
        out = subprocess.run(["pgrep", "-f", "wrist_pair.py"], capture_output=True, timeout=5).stdout
    except Exception:  # noqa: BLE001
        return None
    pids = [int(p) for p in out.split() if p.strip()]
    return pids[0] if pids else None


def pair_start():
    if pair_pid():
        return True
    subprocess.Popen(["bash", "-lc", PAIR_CMD], stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL, start_new_session=True)
    return True


def pair_stop():
    if not pair_pid():
        return True
    subprocess.run(["pkill", "-TERM", "-f", "wrist_pair.py"], capture_output=True, timeout=5)
    return True


def rviz_start():
    with _rviz_lock:
        if rviz_pid():
            return True
        subprocess.Popen(["bash", "-lc", RVIZ_CMD], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)
    pair_start()
    return True


def rviz_stop():
    with _rviz_lock:
        if not rviz_pid():
            return True
        for sig in ("TERM", "KILL"):
            subprocess.run(["pkill", "-%s" % sig, "-f", "rviz2"], capture_output=True, timeout=5)
            for _ in range(25):
                if not rviz_pid():
                    pair_stop()
                    return True
                threading.Event().wait(0.1)
    pair_stop()
    return rviz_pid() is None


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
        elif self.path.startswith("/rviz/state"):
            pid = rviz_pid()
            self._cors(200)
            self.wfile.write(json.dumps({"running": pid is not None, "pid": pid}).encode())
        else:
            self._cors(404)
            self.wfile.write(b'{"error": "not found"}')

    def do_POST(self):
        path = self.path.split("?")[0]
        if path in ("/rviz/start", "/rviz/stop"):
            try:
                ok = rviz_start() if path.endswith("start") else rviz_stop()
                pid = rviz_pid()
                self._cors(200 if ok else 500)
                self.wfile.write(json.dumps({"ok": ok, "running": pid is not None,
                                             "pid": pid}).encode())
            except Exception as exc:  # noqa: BLE001
                self._cors(500)
                self.wfile.write(json.dumps({"ok": False, "error": str(exc)}).encode())
            return
        if path != "/shutdown":
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

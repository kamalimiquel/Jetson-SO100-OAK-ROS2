#!/usr/bin/env python3
"""LeRobot joint server — stdlib-only HTTP API over the SO-100 serial bus.

Runs with the LeRobot venv (py3.12) inside the arm container; the ROS2 bridge
(py3.8 + rclpy) polls it over loopback HTTP. Two processes because the image's
rclpy is built for py3.8 and LeRobot needs >=3.10.

    GET  /status  -> {"connected": true, "calibrated": true}
    GET  /read    -> {"angles": [6 floats, degrees], "names": [...]}
    POST /write   -> body {"angles": [6 floats, degrees]}  (auto-enables torque)

Env: SO100_PORT (default /dev/ttyACM0), SO100_ID (default so100_cell),
     PORT (default 8002, bound to 127.0.0.1).
"""

import json
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig

PORT = int(os.environ.get("PORT", "8002"))
SERIAL = os.environ.get("SO100_PORT", "/dev/ttyACM0")
ARM_ID = os.environ.get("SO100_ID", "so100_cell")

# Hard joint limits (degrees) — the F710 client clamps too; these are the
# server-side backstop. Order: shoulder_pan, shoulder_lift, elbow_flex,
# wrist_flex, wrist_roll, gripper.
LO = [-170.0, -170.0, -170.0, -170.0, -170.0, 0.0]
HI = [170.0, 170.0, 170.0, 170.0, 170.0, 100.0]


def main() -> None:
    print(f"[joint_server] connecting SO100Follower id={ARM_ID} port={SERIAL}", flush=True)
    arm = SO100Follower(SO100FollowerConfig(port=SERIAL, id=ARM_ID))
    arm.connect()  # calibrate=True by default; raises if the calib file is missing
    bus = arm.bus
    names = list(bus.motors.keys())
    print(f"[joint_server] ready — {len(names)} motors, calibrated={arm.is_calibrated}", flush=True)
    torque_on = False

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # quiet
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/read":
                try:
                    angles = [float(bus.read("Present_Position", n)) for n in names]
                    self._json({"angles": angles, "names": names})
                except Exception as exc:  # noqa: BLE001
                    self._json({"error": str(exc)}, 500)
            elif self.path == "/status":
                self._json({"connected": True, "calibrated": arm.is_calibrated})
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            if self.path != "/write":
                self._json({"error": "not found"}, 404)
                return
            nonlocal torque_on
            try:
                length = int(self.headers.get("Content-Length", 0))
                angles = json.loads(self.rfile.read(length) or b"{}")["angles"]
                if len(angles) != len(names):
                    raise ValueError(f"expected {len(names)} angles")
                if not torque_on:
                    bus.enable_torque()
                    torque_on = True
                    print("[joint_server] torque enabled", flush=True)
                for name, value in zip(names, angles):
                    i = names.index(name)
                    value = float(value)
                    value = max(LO[i], min(HI[i], value))
                    bus.write("Goal_Position", name, value)
                self._json({"status": "ok"})
            except Exception as exc:  # noqa: BLE001
                self._json({"error": str(exc)}, 500)

    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()


if __name__ == "__main__":
    sys.exit(main())

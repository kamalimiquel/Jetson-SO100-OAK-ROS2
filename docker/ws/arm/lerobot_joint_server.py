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
import time
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


def connect_with_retry(arm, tries=15, delay=2.0):
    """Connect, and survive the glitches right after the board powers up.

    A checksum error on one motor (`Failed to enable torque ... Checksum error`) kills
    `arm.connect()` outright — and it is exactly what happens when the servos come up
    out of an abrupt power cut, mid-enumeration. One try means the arm stays dead and
    silent for the rest of the boot while `docker ps` still says "Up", which is how a
    restart looked like a crashed cell.
    """
    import time

    for attempt in range(1, tries + 1):
        try:
            arm.connect()
            return
        except Exception as exc:  # noqa: BLE001
            if attempt == tries:
                raise
            print(f"[joint_server] connect failed (attempt {attempt}/{tries}): {exc} — retrying in {delay:.0f}s",
                  flush=True)
            time.sleep(delay)


def main() -> None:
    print(f"[joint_server] connecting SO100Follower id={ARM_ID} port={SERIAL}", flush=True)
    arm = SO100Follower(SO100FollowerConfig(port=SERIAL, id=ARM_ID))
    connect_with_retry(arm)  # calibrate=True by default; raises if the calib file is missing
    bus = arm.bus
    names = list(bus.motors.keys())
    # Boot state is ALWAYS torque off: the arm must be free to move by hand until the
    # operator presses the button in the web page. Never inherit the servos' own state.
    try:
        bus.disable_torque()
    except Exception as exc:  # noqa: BLE001
        print(f"[joint_server] torque-off at boot failed: {exc}", flush=True)
    print(f"[joint_server] ready — {len(names)} motors, calibrated={arm.is_calibrated}, torque=OFF",
          flush=True)
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
            elif self.path == "/torque":
                self._json({"torque": bool(torque_on)})
            elif self.path == "/status":
                self._json({"connected": True, "calibrated": arm.is_calibrated})
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            nonlocal torque_on
            if self.path == "/torque":
                try:
                    length = int(self.headers.get("Content-Length", 0))
                    on = bool(json.loads(self.rfile.read(length) or b"{}").get("on"))
                    if on:
                        # Freeze the arm where it IS. A servo keeps its own Goal_Position, so
                        # `enable_torque()` alone makes it snap back to the last command it was
                        # given — the jump seen when arming, and the reason the arm went to a
                        # pose from *before* the torque was removed. Writing the present
                        # positions as goals first (torque still off) makes "arm" mean "stay".
                        present = {n: float(bus.read("Present_Position", n)) for n in names}
                        bus.sync_write("Goal_Position", present)
                        bus.enable_torque()
                        # Some Feetech firmwares ignore goal writes while torque is off and hold
                        # the OLD goal until the first packet after enabling; write the same
                        # values again with torque on, so the snap (if any) lasts one packet.
                        # Close the loop. Even with the goal equal to the present position a
                        # servo taking up a gravity load sags a little before it settles, and a
                        # firmware that ignored the loose write snaps first. Two or three short
                        # corrections put the arm back exactly where it was before arming.
                        corrections, worst = 0, 0.0
                        for _ in range(4):
                            time.sleep(0.15)
                            now = {n: float(bus.read("Present_Position", n)) for n in names}
                            worst = max(abs(now[n] - present[n]) for n in names)
                            if worst <= 0.5:
                                break
                            corrections += 1
                            bus.sync_write("Goal_Position", present)
                        print("[joint_server] armed at " +
                              ", ".join(f"{n}={present[n]:.1f}" for n in names) +
                              (f" — {corrections} correction(s), worst {worst:.1f}deg" if corrections
                               else f" — held it (worst {worst:.1f}deg)"), flush=True)
                    else:
                        bus.disable_torque()
                    torque_on = on
                    print(f"[joint_server] torque {'ENABLED' if on else 'DISABLED'}", flush=True)
                    self._json({"torque": bool(torque_on)})
                except Exception as exc:  # noqa: BLE001
                    self._json({"error": str(exc)}, 500)
                return
            if self.path != "/write":
                self._json({"error": "not found"}, 404)
                return
            try:
                length = int(self.headers.get("Content-Length", 0))
                angles = json.loads(self.rfile.read(length) or b"{}")["angles"]
                # 6 values = drive every joint; 5 = drive the first five and leave the
                # gripper completely alone (hand-eye sweeps and any caller that must not
                # move joint 6). More than 6 is always an error.
                if len(angles) not in (5, len(names)) or len(angles) > len(names):
                    raise ValueError(f"expected 5 or {len(names)} angles, got {len(angles)}")
                if not torque_on:
                    # do not energise the servos behind the operator's back: the web
                    # button is the gate, and the boot state is torque off
                    self._json({"error": "torque is off — press this arm's torque button in the web page "
                                 "(or hold B for 5 s on the pad)"}, 409)
                    return
                for name, value in zip(names, angles):
                    i = names.index(name)
                    value = float(value)
                    if value != value:      # NaN = caller does not want this joint driven
                        continue            # (leaves Goal_Position where the servo holds it)
                    value = max(LO[i], min(HI[i], value))
                    bus.write("Goal_Position", name, value)
                self._json({"status": "ok"})
            except Exception as exc:  # noqa: BLE001
                self._json({"error": str(exc)}, 500)

    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""LeRobot joint server — stdlib-only HTTP API over the SO-100 serial bus.

Runs with the LeRobot venv (py3.12) inside the arm container; the ROS2 bridge
(py3.8 + rclpy) polls it over loopback HTTP. Two processes because the image's
rclpy is built for py3.8 and LeRobot needs >=3.10.

    GET  /status  -> {"connected": true, "calibrated": true}
    GET  /read    -> {"angles": [6 floats, degrees], "names": [...]}
    GET  /torque  -> {"torque": false}
    GET  /owner   -> {"owner": "policy" | null, "seconds_left": 3.2}
    POST /claim   -> body {"owner": "policy", "ttl_s": 5.0}  takes exclusive ownership of this
                     arm's write path for ttl_s seconds (0 releases). While a lease is held,
                     /write and /torque need the header `X-Owner: <owner>` and answer 423
                     otherwise — the loopback port is the boundary a topic-level guard cannot
                     reach (ttyd on :7681 can curl it from inside the container).
    POST /torque  -> body {"on": true|false}  flips the servos' torque; enabling seeds the
                     goals from the PRESENT positions, so the arm holds where it stands
    POST /write   -> body {"angles": [6 floats, degrees]}  or
                          {"angles": [5 floats, degrees], "hold": ["gripper"]}
                     exact length, finite, inside the joint's range (add "clamp": true to mean
                     "as far as it goes"), one sync_write packet for every driven joint, and a
                     400 naming the joint when any of that fails. Torque must be ON: with
                     torque off it answers 409 and moves nothing (the web page's torque button
                     and pad-B are the two ways in).

Env: SO100_PORT (default /dev/ttyACM0), SO100_ID (default so100_cell),
     PORT (default 8002, bound to 127.0.0.1).
"""

import json
import os
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig

# The command contract lives in the shared lib so the arbiter and this server reject the same
# things for the same reasons (/opt/ws/lib in the container, docker/ws/lib in the repo).
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "lib"))
import command_contract as cc  # noqa: E402

PORT = int(os.environ.get("PORT", "8002"))
SERIAL = os.environ.get("SO100_PORT", "/dev/ttyACM0")
ARM_ID = os.environ.get("SO100_ID", "so100_cell")

# The joint table and its limits live in the shared command contract (docker/ws/lib), so the
# arbiter, the tests and this server cannot drift apart: order is shoulder_pan, shoulder_lift,
# elbow_flex, wrist_flex, wrist_roll, gripper.


def connect_with_retry(arm, tries=30, delay=2.0):
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
            # LeRobot's connect() marks the object connected *before* configure() talks to the
            # servos, so when configure() fails (a timeout at power-up is the usual one) the next
            # attempt dies instantly with DeviceAlreadyConnectedError and the retries do nothing.
            # Drop the half-open connection first, then the retry is a real one.
            try:
                if getattr(arm, "is_connected", False):
                    arm.disconnect()
            except Exception:  # noqa: BLE001 — a failed cleanup must not stop the retry
                pass
            if attempt == tries:
                raise
            print(f"[joint_server] connect failed (attempt {attempt}/{tries}): {exc} — "
                  f"retrying in {delay:.0f}s (the bus may still be powering up)", flush=True)
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

    # Exclusive ownership of this arm's write path. Unclaimed (owner None) keeps the 1.0
    # behaviour — the web torque button and the pad work with no token — because a bench that
    # suddenly needs a token would break the two controls the operator actually uses. The moment
    # a supervisor (the arbiter) or a policy claims the arm, every other writer on loopback or
    # on the topic is refused with 423, which is the one boundary a topic-level arbiter cannot
    # reach: ttyd on :7681 can `curl 127.0.0.1:8002/write` from inside this container.
    lease = {"owner": None, "until": 0.0}
    # Set when a write packet failed and the measured pose is not what was asked: the write path
    # stays closed until the arm is released/re-claimed or torque is cycled (which re-seeds every
    # goal from Present_Position). Never let the next command drive from an unknown pose.
    fault = None

    def lease_left():
        if lease["owner"] is None:
            return 0.0
        left = lease["until"] - time.time()
        if left <= 0.0:
            print(f"[joint_server] owner {lease['owner']} lease expired", flush=True)
            lease["owner"], lease["until"] = None, 0.0
            return 0.0
        return left

    def owner_refused(handler):
        """423 when a lease is held by someone else; None when the caller may write."""
        if lease_left() and handler.headers.get("X-Owner") != lease["owner"]:
            return "%s holds this arm (%.1fs left) — X-Owner must match" % (
                lease["owner"], lease_left())
        return None

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
            elif self.path == "/owner":
                self._json({"owner": lease["owner"], "seconds_left": round(lease_left(), 1)})
            elif self.path == "/status":
                self._json({"connected": True, "calibrated": arm.is_calibrated})
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            nonlocal torque_on, fault
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length) or b"{}"
            try:
                body = json.loads(raw)
            except ValueError as exc:
                self._json({"error": f"body is not JSON: {exc}"}, 400)
                return

            if self.path == "/claim":
                # Ownership of the write path, taken and given back explicitly. ttl_s = 0 (or a
                # dead supervisor's timeout) returns the arm to unclaimed, where the web button
                # and the pad work as before.
                who = str(body.get("owner") or "").strip()
                ttl = float(body.get("ttl_s") or 0.0)
                if not who or ttl <= 0.0:
                    lease["owner"], lease["until"] = None, 0.0
                    fault = None
                    print("[joint_server] write path released (unclaimed)", flush=True)
                    self._json({"owner": None, "seconds_left": 0.0})
                    return
                if lease_left() and who != lease["owner"]:
                    self._json({"error": "%s holds this arm (%.1fs left)" % (
                        lease["owner"], lease_left())}, 423)
                    return
                lease["owner"], lease["until"] = who, time.time() + ttl
                fault = None
                print(f"[joint_server] write path claimed by {who} for {ttl:.1f}s", flush=True)
                self._json({"owner": who, "seconds_left": round(lease_left(), 1)})
                return
            if self.path == "/release":
                if lease_left() and self.headers.get("X-Owner") != lease["owner"]:
                    self._json({"error": "%s holds this arm" % lease["owner"]}, 423)
                    return
                lease["owner"], lease["until"], fault = None, 0.0, None
                print("[joint_server] write path released", flush=True)
                self._json({"owner": None, "seconds_left": 0.0})
                return

            if self.path not in ("/torque", "/write"):
                self._json({"error": "not found"}, 404)
                return

            if self.path == "/write":
                # The lease gates MOTION. Torque stays the operator's switch: the web button and
                # the pad's B send no X-Owner, and a supervisor able to energise an arm is what
                # this design forbids (a policy talks the topic, never this port).
                refused = owner_refused(self)
                if refused:
                    self._json({"error": refused}, 423)
                    return

            if self.path == "/torque":
                try:
                    on = bool(body.get("on"))
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
                    # Cycling torque re-seeds every goal from the measured position, so it is
                    # also the physical way out of a held write path.
                    fault = None
                    print(f"[joint_server] torque {'ENABLED' if on else 'DISABLED'}", flush=True)
                    self._json({"torque": bool(torque_on)})
                except Exception as exc:  # noqa: BLE001
                    self._json({"error": str(exc)}, 500)
                return
            if self.path != "/write":
                self._json({"error": "not found"}, 404)
                return
            try:
                driven, held, notes = cc.validate_body(body)
            except cc.CommandError as exc:
                # 400, not the old 500: the caller can fix this, and the sentence says how.
                print(f"[joint_server] write rejected: {exc}", flush=True)
                self._json({"error": str(exc)}, 400)
                return
            if not torque_on:
                # do not energise the servos behind the operator's back: the web
                # button is the gate, and the boot state is torque off
                self._json({"error": "torque is off — press this arm's torque button in the web page "
                             "(or hold B for 5 s on the pad)"}, 409)
                return
            if fault:
                # A previous packet failed and the arm is not where it was asked to be. Letting
                # the next command through would drive the remaining joints from an unknown
                # pose, so the path stays closed until the operator releases/re-claims the arm
                # or cycles torque (which re-seeds every goal from the measured position).
                self._json({"error": "write path held after a failed packet — release or re-claim the "
                                     "arm, or cycle torque", "fault": fault}, 409)
                return
            try:
                # ONE packet for every driven joint. The old loop wrote motor by motor, so an
                # exception on the fourth left three moved and three not: a pose nobody asked
                # for, at full torque.
                bus.sync_write("Goal_Position", driven)
            except Exception as exc:  # noqa: BLE001
                # Did it actually fail? The packet may have reached some servos and not others,
                # and the bus's own answer is not the truth — the encoders are. Compare.
                try:
                    now = {n: float(bus.read("Present_Position", n)) for n in names}
                except Exception as read_exc:  # noqa: BLE001
                    fault = f"write failed ({exc}) and the arm could not be read ({read_exc})"
                    print(f"[joint_server] FAULT {fault}", flush=True)
                    self._json({"error": fault}, 502)
                    return
                off = {n: round(now[n] - v, 2) for n, v in driven.items() if abs(now[n] - v) > 2.0}
                if not off:
                    print(f"[joint_server] write reported {exc} but the arm is where it was told "
                          f"it should be — treating the packet as delivered", flush=True)
                    self._json({"status": "ok", "note": f"bus error ignored: {exc}",
                                "driven": driven, "held": sorted(held)})
                    return
                fault = "write failed (%s); measured differs from the command on %s" % (exc, off)
                print(f"[joint_server] FAULT {fault}", flush=True)
                self._json({"error": fault, "measured": now}, 502)
                return
            if notes:
                print("[joint_server] " + "; ".join(notes), flush=True)
            self._json({"status": "ok", "driven": {n: round(v, 2) for n, v in driven.items()},
                        "held": sorted(held)})

    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()


if __name__ == "__main__":
    sys.exit(main())

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
     PORT (default 8002, bound to 127.0.0.1),
     JOINT_SCALE_APPLY (default 0). With it on, /write reads degrees as PHYSICAL and divides by the
     measured scale of each joint (docker/ws/lib/joint_scale.json) before the packet; a body may still
     say {"units": "sdk"} to bypass. Off, every number is SDK degrees, as before.
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
import joint_scale as js  # noqa: E402
import units_manifest as um  # noqa: E402

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


def sdk_version():
    try:
        import importlib.metadata as md
        return md.version("lerobot")
    except Exception:  # noqa: BLE001 — a missing distribution is a warning, not a crash
        return None


def main() -> None:
    print(f"[joint_server] connecting SO100Follower id={ARM_ID} port={SERIAL}", flush=True)
    # use_degrees is stated, not inherited: the SDK default chooses between degrees and a -100..100
    # fraction for the five body joints, and nothing raises if it changes.
    # A joint can be declared ABSENT for a maintenance session (a servo unplugged on purpose, e.g. to
    # free the wrist for a roll sweep, or a link that has failed). It is removed from the bus map
    # *before* connecting — lerobot refuses to connect when a motor it expects is missing — and every
    # command must name it as held: the arm does not silently drive five and pretend it is six.
    absent = [a.strip() for a in os.environ.get("ABSENT_MOTORS", "").split(",") if a.strip()]
    unknown = [a for a in absent if a not in cc.JOINT_ORDER]
    if unknown:
        print(f"[joint_server] refusing to start: ABSENT_MOTORS names unknown joint(s) {unknown}",
              flush=True)
        return 1

    config = SO100FollowerConfig(port=SERIAL, id=ARM_ID, use_degrees=True)
    arm = SO100Follower(config)
    for a in absent:
        arm.bus.motors.pop(a, None)
        arm.bus.calibration.pop(a, None)
    if absent:
        # lerobot caches `ids` (a cached_property) from the calibration it loaded at __init__, so
        # popping the motor is not enough: its own motor check then trips over a KeyError from its
        # bookkeeping (motors_bus.py, _assert_motors_exist). Clearing the cache makes the declared
        # set the real one.
        for attr in ("ids", "_ids"):
            arm.bus.__dict__.pop(attr, None)
    connect_with_retry(arm)
    bus = arm.bus
    names = [n for n in list(bus.motors.keys()) if n not in absent]
    if absent:
        print(f"[joint_server] joints declared ABSENT: {', '.join(absent)} — the other "
              f"{len(names)} are readable and commandable; commands must name them as held", flush=True)
    # Fail closed: the numbers mean what the manifest says, or the arm does not come up.
    import inspect
    signatures = {m: inspect.signature(getattr(bus, m)).parameters for m in ("read", "write", "sync_write")}
    manifest = um.observed_from(arm, config=config, sdk_version=sdk_version(), signatures=signatures)
    errors, warnings = um.check(manifest, absent=absent)
    for line in um.describe(manifest):
        print("[joint_server] units  " + line, flush=True)
    for w in warnings:
        print(f"[joint_server] units WARNING: {w}", flush=True)
    if errors:
        for e in errors:
            print(f"[joint_server] units ERROR: {e}", flush=True)
        print("[joint_server] refusing to start: the joint units do not match the manifest "
              "(docker/ws/lib/units_manifest.py). Fix the SDK/config, then re-run "
              "tools/joint_units.py.", flush=True)
        raise SystemExit(2)
    calib = getattr(arm, "calibration_fpath", None)
    calib_sha = None
    try:
        import hashlib
        with open(calib, "rb") as fh:
            calib_sha = hashlib.sha256(fh.read()).hexdigest()
    except Exception:  # noqa: BLE001
        pass
    # The measured scale of each joint: what a commanded degree actually turns. Data, next to the
    # unit table, because it changes a command's meaning — so it is opt-in and it is announced.
    scale_apply = str(os.environ.get("JOINT_SCALE_APPLY", "0")).strip().lower() in ("1", "true", "yes")
    try:
        scale = js.load(js.path_for(ARM_ID))
    except js.ScaleError as exc:
        if scale_apply:
            print(f"[joint_server] refusing to start: JOINT_SCALE_APPLY=1 but {exc}", flush=True)
            return 1
        scale, scale_apply = None, False
        print(f"[joint_server] joint scale NOT applied ({exc})", flush=True)
    if scale and scale_apply:
        try:
            js.check_arm(scale, ARM_ID)
        except js.ScaleError as exc:
            print(f"[joint_server] refusing to start: JOINT_SCALE_APPLY=1 but {exc}", flush=True)
            return 1
    if scale:
        print(f"[joint_server] joint scale {js.describe(scale)}"
              f" — {'APPLIED: /write takes physical degrees' if scale_apply else 'loaded, not applied'}"
              f" (JOINT_SCALE_APPLY={'1' if scale_apply else '0'})", flush=True)

    units = {j["name"]: um.UNIT_OF.get(j["norm"], "?") for j in manifest["joints"]}
    print(f"[joint_server] units OK — sdk {manifest['sdk_version']}, degrees={manifest['use_degrees']}, "
          f"calibration={calib} sha256={str(calib_sha)[:12]}", flush=True)
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

    def torque_each(want_on):
        """Enable or disable ONE motor at a time, and say which refused.

        `bus.disable_torque()` with no argument is all-or-nothing: a single servo that stops answering
        (a pulled cable at the last link of the daisy chain is the usual way) makes the whole call fail,
        so the five healthy motors stay energised and the operator cannot let go of the arm. Looping
        costs a few milliseconds and turns one dead motor into one dead motor.
        """
        failed = {}
        for name in names:
            try:
                if want_on:
                    bus.enable_torque(motors=[name], num_retry=1)
                else:
                    bus.disable_torque(motors=[name], num_retry=1)
            except Exception as exc:  # noqa: BLE001
                failed[name] = str(exc)
        return failed

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
            elif self.path == "/read_raw":
                # Raw encoder ticks next to the SDK's value: the audit table of phase 1 needs both,
                # and reading them here avoids a second serial connection (the port is exclusive).
                try:
                    ticks = {n: int(bus.read("Present_Position", n, normalize=False)) for n in names}
                    norm = {n: float(bus.read("Present_Position", n)) for n in names}
                    body = {"names": names, "ticks": ticks, "normalized": norm, "unit": units,
                            "torque": bool(torque_on)}
                    if scale:
                        # What the SDK number is worth on the bench. The raw fields stay: a client
                        # that wants the encoder's opinion keeps it, and nothing is silently rescaled.
                        body["physical"] = {n: round(js.to_physical(scale, n, norm[n]), 2) for n in names}
                    self._json(body)
                except Exception as exc:  # noqa: BLE001
                    self._json({"error": str(exc)}, 500)
            elif self.path == "/manifest":
                # What this process believes it is running: the ledger records it, and a mismatch
                # with the deployed image is visible without reading the source.
                self._json({"arm_id": ARM_ID, "port": SERIAL, "sdk_version": manifest["sdk_version"],
                            "use_degrees": manifest["use_degrees"], "joints": manifest["joints"],
                            "unit": units, "calibration": str(calib) if calib else None,
                            "calibration_sha256": calib_sha,
                            "absent": absent,
                            "joint_scale": ({"sha256": scale["sha256"], "applied": scale_apply,
                                             "session": scale.get("session")} if scale else None),
                            "limits": {n: cc.LIMITS[n] for n in cc.JOINT_ORDER}})
            elif self.path == "/torque":
                self._json({"torque": bool(torque_on)})
            elif self.path == "/owner":
                self._json({"owner": lease["owner"], "seconds_left": round(lease_left(), 1)})
            elif self.path == "/scale":
                if not scale:
                    self._json({"error": "no joint_scale.json next to this server"}, 404)
                    return
                self._json({"applied": scale_apply, "arm": scale.get("arm"), "this_arm": ARM_ID,
                            "sha256": scale["sha256"],
                            "session": scale.get("session"), "evidence": scale.get("evidence"),
                            "definition": scale["definition"], "joints": scale["joints"]})
            elif self.path == "/diag":
                # Which motors answer, one at a time: a 500 on /read_raw says "something is wrong",
                # this says which link of the chain. Slow on a dead motor, so it is asked for.
                motors = {}
                for n in names:
                    try:
                        sdk = float(bus.read("Present_Position", n))
                        motors[n] = {"ok": True, "sdk": round(sdk, 2),
                                     "tick": int(bus.read("Present_Position", n, normalize=False))}
                    except Exception as exc:  # noqa: BLE001
                        motors[n] = {"ok": False, "error": str(exc)}
                self._json({"motors": motors,
                            "unreachable": [n for n, v in motors.items() if not v["ok"]]})
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
                changed = lease["owner"] != who
                lease["owner"], lease["until"] = who, time.time() + ttl
                fault = None
                if changed:
                    # A renewal every 0.5 s must not write a log line every time: it buries the
                    # fault and rejection lines this log exists for.
                    print(f"[joint_server] write path claimed by {who} (lease {ttl:.1f}s, renewals are "
                          f"silent)", flush=True)
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
                    failed, unreachable = {}, {}
                    if on:
                        # Freeze the arm where it IS: a servo keeps its own Goal_Position, so arming
                        # alone snaps it back to the last command it was given. Read what answers,
                        # seed those goals, then energise them one by one.
                        present = {}
                        for n in names:
                            try:
                                present[n] = float(bus.read("Present_Position", n))
                            except Exception as exc:  # noqa: BLE001
                                unreachable[n] = str(exc)
                        if present:
                            bus.sync_write("Goal_Position", present)
                        failed = torque_each(True)
                        corrections, worst = 0, 0.0
                        for _ in range(4):
                            time.sleep(0.15)
                            now, lost = {}, False
                            for n in present:
                                try:
                                    now[n] = float(bus.read("Present_Position", n))
                                except Exception:  # noqa: BLE001
                                    lost = True
                                    break
                            if lost:
                                break
                            worst = max(abs(now[n] - present[n]) for n in present)
                            if worst <= 0.5:
                                break
                            corrections += 1
                            bus.sync_write("Goal_Position", present)
                        print("[joint_server] armed at " +
                              ", ".join(f"{n}={present[n]:.1f}" for n in present) +
                              (f" — {corrections} correction(s), worst {worst:.1f}deg" if corrections
                               else f" — held it (worst {worst:.1f}deg)"), flush=True)
                        torque_on = bool(present) and len(present) > len(failed)
                    else:
                        failed = torque_each(False)
                        # Any motor that would not let go is still energised: torque stays TRUE so
                        # nothing downstream believes the arm is safe.
                        torque_on = bool(failed)
                    if failed or unreachable:
                        print("[joint_server] torque %s — motors that did not answer: %s%s"
                              % ("ON" if on else "OFF",
                                 ", ".join(sorted(set(list(failed) + list(unreachable)))),
                                 " — CUT POWER before touching the arm" if failed and not on else ""),
                              flush=True)
                    else:
                        print(f"[joint_server] torque {'ENABLED' if on else 'DISABLED'}", flush=True)
                    fault = None
                    self._json({"torque": bool(torque_on), "failed": failed,
                                "unreachable": unreachable,
                                "note": ("%d motor(s) did not answer%s" % (
                                    len(set(list(failed) + list(unreachable))),
                                    "; they are still energised" if failed and not on else ""))
                                if (failed or unreachable) else ""})
                except Exception as exc:  # noqa: BLE001
                    self._json({"error": str(exc)}, 500)
                return
            if self.path != "/write":
                self._json({"error": "not found"}, 404)
                return
            units_in = str(body.get("units") or "").strip().lower()
            if units_in not in ("", "sdk", "physical"):
                self._json({"error": "units must be 'sdk' or 'physical', got %r" % body.get("units")}, 400)
                return
            if units_in == "physical" and not scale_apply:
                self._json({"error": "this server takes SDK degrees: JOINT_SCALE_APPLY=0, so there is no "
                                     "measured scale in the path (start it with JOINT_SCALE_APPLY=1 to "
                                     "command physical degrees)"}, 400)
                return
            transform = (lambda n, v: js.to_sdk(scale, n, v)) if units_in == "physical" else None
            try:
                driven, held, notes = cc.validate_body(body, transform=transform)
            except cc.CommandError as exc:
                # 400, not the old 500: the caller can fix this, and the sentence says how.
                print(f"[joint_server] write rejected: {exc}", flush=True)
                self._json({"error": str(exc)}, 400)
                return
            if absent:
                driven_absent = sorted(set(driven) & set(absent))
                if driven_absent:
                    self._json({"error": "%s declared absent (ABSENT_MOTORS) — drive only %s and name the "
                                         "absent joint(s) as held" % (driven_absent, names)}, 400)
                    return
                if isinstance(body.get("hold"), list) and not set(absent) <= set(body.get("hold") or []):
                    self._json({"error": "a declared-absent joint must also be named in 'hold': %s"
                                         % sorted(set(absent) - set(body.get("hold") or []))}, 400)
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
                                "driven": driven, "held": sorted(held), "notes": notes})
                    return
                fault = "write failed (%s); measured differs from the command on %s" % (exc, off)
                print(f"[joint_server] FAULT {fault}", flush=True)
                self._json({"error": fault, "measured": now}, 502)
                return
            if notes:
                print("[joint_server] " + "; ".join(notes), flush=True)
            self._json({"status": "ok", "driven": {n: round(v, 2) for n, v in driven.items()},
                        "held": sorted(held), "notes": notes})

    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()


if __name__ == "__main__":
    sys.exit(main())

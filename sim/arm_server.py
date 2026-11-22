#!/usr/bin/env python3
"""A virtual arm that speaks the same four endpoints as the real one, backed by physics.

The point is not to simulate an arm; it is to make the simulator *invisible to everything above it*. The
supervisor, `skill_plan`, the recorders and the MCP endpoint talk HTTP to `127.0.0.1:8002`; if a MuJoCo model
answers on 8004 with the same contract, they all run unchanged and the profile switch is a port number.

    GET  /owner, /torque, /status, /read, /read_raw           read
    POST /claim, /release                                    lease (423 if someone else holds it)
    POST /torque  {"on": bool}                               the operator's switch, never leased
    POST /write   {"angles": [6], "units": "sdk", "hold": [...]}

Refusals mirror the real server on purpose, because a sim that is more permissive than the hardware teaches
the wrong lesson: 400 for a malformed body, 409 for torque off, 423 for a write without the lease. `hold` is
accepted with five values, exactly as on the bench.

**It fails closed**: without `mujoco` importable it exits with a sentence instead of pretending to be physics.
A kinematic stand-in would let a mission pass that touches a block, which is the one thing this phase exists to
catch.

    python3 sim/arm_server.py --model scene_two_arms.xml --port 8004 --arm so100_cell
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

JOINT_ORDER = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
DEFAULT_TTL_S = 15.0


class Refused(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code
        self.message = message


class MuJoCoModel:
    """Physics. `ctrl` when the model has actuators for the six joints, `qpos` only when it does not.

    Setting qpos directly is a kinematic pose, not a controlled joint: gravity cannot pull against a target
    and contact cannot push back. That is why the actuator path is the default and the qpos path says so in
    `/status`, so nobody mistakes one for the other.
    """

    def __init__(self, path, prefix=""):
        try:
            import mujoco  # noqa: F401
        except Exception as exc:                                     # noqa: BLE001
            raise SystemExit("sim: mujoco is not importable (%s) — refusing to run without physics" % exc)
        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        self.idx = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if not name:
                continue
            for joint in JOINT_ORDER:
                if name == prefix + joint:
                    self.idx[joint] = int(self.model.jnt_qposadr[i])
        missing = [j for j in JOINT_ORDER if j not in self.idx]
        if missing:
            raise SystemExit("sim: the model has no joint for %s (prefix %r)" % (missing, prefix))
        self.actuated = self.model.nu >= len(JOINT_ORDER)
        self.target = {j: 0.0 for j in JOINT_ORDER}       # radians
        for j, adr in self.idx.items():
            self.target[j] = float(self.data.qpos[adr])
        # Physics has to run on its own clock, not only when a command arrives: a servo that is told a
        # position holds it against gravity between commands, and that steady-state error is the whole
        # point of this phase. Stepping only on /write measured 0.04 s of a 4 s settle.
        self.lock = threading.Lock()
        self.enabled = threading.Event()                  # torque on
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        """Step in batches to keep simulated time equal to wall time.

        Sleeping one timestep per iteration looked right and ran ~20x slow: time.sleep(0.005) on this
        kernel sleeps tens of milliseconds, so the loop was paced by the scheduler, not by the model.
        A step costs ~0.22 ms here (22x real time), so the budget exists — it just has to be spent in
        batches, catching up to real time instead of sleeping between single steps.
        """
        dt = float(self.model.opt.timestep)
        t0, sim = time.time(), 0.0
        self.realtime = 1.0
        while not self._stop.is_set():
            if not self.enabled.is_set():
                t0, sim = time.time(), 0.0        # frozen while torque is off
                time.sleep(0.01)
                continue
            target = time.time() - t0
            n = int((target - sim) / dt)
            if n > 200:                            # more than a second behind: drop it, do not spiral
                t0, sim, n = time.time(), 0.0, 0
            if n <= 0:
                time.sleep(dt)
                continue
            with self.lock:
                for _ in range(n):
                    if self.actuated:
                        self.data.ctrl[:len(JOINT_ORDER)] = [self.target[j] for j in JOINT_ORDER]
                    self.mujoco.mj_step(self.model, self.data)
            sim += n * dt
            self.realtime = round(sim / max(1e-6, time.time() - t0), 3)

    def present_deg(self):
        with self.lock:
            return {j: math.degrees(float(self.data.qpos[adr])) for j, adr in self.idx.items()}

    def objects(self):
        """Every named body's world position. Sim-only: it is how a free body (a block) is watched
        falling and coming to rest, which the arm-side /read deliberately does not expose."""
        with self.lock:
            out = {}
            for i in range(self.model.nbody):
                name = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_BODY, i)
                if name and name != "world":
                    out[name] = [round(float(v), 4) for v in self.data.xpos[i]]
            return out

    def command_deg(self, angles):
        """angles: {joint: degrees}. Sets the servo targets; the physics loop does the moving.

        A kinematic (qpos-only) model has no servos, so there the pose is written directly and the
        distinction is stated in /status — a controlled joint and a teleported one are not the same.
        """
        with self.lock:
            for j, deg in angles.items():
                self.target[j] = math.radians(deg)
            if not self.actuated:
                for j, adr in self.idx.items():
                    self.data.qpos[adr] = self.target[j]
        return self.present_deg()


class State:
    def __init__(self, arm, model, mode):
        self.arm, self.model, self.mode = arm, model, mode
        self.lock = threading.Lock()
        self.owner, self.due = None, 0.0
        self.torque = False
        self.writes = 0

    def lease_left(self):
        return max(0.0, self.due - time.time())

    def claim(self, owner, ttl_s):
        now = time.time()
        if self.owner and self.owner != owner and self.lease_left() > 0:
            raise Refused(423, "arm is leased to %r for another %.1f s" % (self.owner, self.lease_left()))
        self.owner, self.due = owner, now + float(ttl_s)
        return {"owner": self.owner, "seconds_left": round(self.lease_left(), 1)}

    def check_writer(self, header_owner):
        if self.owner and self.lease_left() > 0 and header_owner != self.owner:
            raise Refused(423, "this arm is leased to %r — send X-Owner: %s" % (self.owner, self.owner))


def parse_write(body):
    if not isinstance(body.get("angles"), list):
        raise Refused(400, "angles must be a list of six values, or five with hold=['gripper']")
    units = str(body.get("units") or "sdk").lower()
    if units not in ("sdk", "physical"):
        raise Refused(400, "units must be 'sdk' or 'physical'")
    angles, hold = body["angles"], body.get("hold") or []
    if len(angles) == 5 and "gripper" in hold:
        angles = list(angles) + [None]
    elif len(angles) != 6:
        raise Refused(400, "expected 6 angles for %s, got %d" % (list(JOINT_ORDER), len(angles)))
    out = {}
    for name, value in zip(JOINT_ORDER, angles):
        if value is None:
            continue
        try:
            value = float(value)
        except (TypeError, ValueError):
            raise Refused(400, "%s is not a number" % name)
        if not math.isfinite(value) or abs(value) > 200.0:
            raise Refused(400, "%s = %r is not a plausible angle" % (name, value))
        out[name] = value
    return out


def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, payload):
            raw = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(raw)

        def _body(self):
            n = int(self.headers.get("Content-Length") or 0)
            if not n:
                return {}
            try:
                return json.loads(self.rfile.read(n).decode())
            except ValueError:
                raise Refused(400, "body is not JSON")

        def do_GET(self):
            with state.lock:
                if self.path == "/owner":
                    return self._send(200, {"owner": state.owner if state.lease_left() > 0 else None,
                                            "seconds_left": round(state.lease_left(), 1)})
                if self.path == "/torque":
                    return self._send(200, {"torque": state.torque})
                if self.path == "/objects":
                    return self._send(200, {"objects": state.model.objects()})
                if self.path == "/status":
                    return self._send(200, {"connected": True, "calibrated": True,
                                            "mode": state.mode, "writes": state.writes,
                                            "timestep": float(state.model.model.opt.timestep),
                                            "realtime": getattr(state.model, "realtime", 0.0)})
                if self.path in ("/read", "/read_raw"):
                    present = state.model.present_deg()
                    return self._send(200, {"angles": [present[j] for j in JOINT_ORDER],
                                            "normalized": present, "names": list(JOINT_ORDER)})
                return self._send(404, {"error": "not found"})

        def do_POST(self):
            try:
                body = self._body()
                with state.lock:
                    if self.path == "/claim":
                        return self._send(200, state.claim(body.get("owner") or "", body.get("ttl_s") or DEFAULT_TTL_S))
                    if self.path == "/release":
                        if (body.get("owner") or "") == state.owner:
                            state.owner, state.due = None, 0.0
                        return self._send(200, {"owner": state.owner})
                    if self.path == "/torque":                      # the operator's switch: never leased
                        state.torque = bool(body.get("on"))
                        if state.torque:
                            state.model.enabled.set()
                        else:
                            state.model.enabled.clear()
                        return self._send(200, {"torque": state.torque, "failed": {}, "unreachable": {}})
                    if self.path == "/write":
                        state.check_writer(self.headers.get("X-Owner"))
                        if not state.torque:
                            raise Refused(409, "torque is off — nothing is written while the servos are free")
                        angles = parse_write(body)
                        got = state.model.command_deg(angles)
                        state.writes += 1
                        return self._send(200, {"status": "ok", "driven": angles,
                                                "present": {j: round(v, 3) for j, v in got.items()}})
                return self._send(404, {"error": "not found"})
            except Refused as exc:
                return self._send(exc.code, {"error": exc.message})

    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="MuJoCo scene or MJCF")
    ap.add_argument("--port", type=int, default=8004)
    ap.add_argument("--arm", default="so100_cell")
    ap.add_argument("--prefix", default="", help="joint name prefix in the scene, e.g. 'arm2_'")
    args = ap.parse_args(argv)

    model = MuJoCoModel(args.model, args.prefix)
    mode = "actuated" if model.actuated else "qpos (kinematic — say so in /status)"
    model.start()
    state = State(args.arm, model, mode)
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(state))
    print("sim arm %r on 127.0.0.1:%d — %s, %d qpos" % (args.arm, args.port, mode, model.model.nq), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())

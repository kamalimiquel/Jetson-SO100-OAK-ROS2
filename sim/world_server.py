#!/usr/bin/env python3
"""Two arms in one physics world, served as two arms on two ports.

`arm_server.py` is one arm and one process. Two of those would be two separate universes: the block would fall
through the second arm because the second arm is not in the model that is stepping. This server holds ONE
`MjData`, steps it once, and exposes each arm's endpoints on its own port — so the supervisor sees two arms
exactly as it sees the two real ones, while the physics is shared.

    python3 sim/world_server.py --model scene_two_arms.xml \\
        --arm-port so100_cell=8004 --arm-port so100_cell_b=8005 \\
        --prefix "" arm2_

Leases and torque are per arm (each port answers for itself); the world and the timestep are shared. Torque
off on an arm freezes that arm's target, so a free arm does not hold a pose it would not hold in reality.

Without mujoco importable it exits with a sentence — a kinematic stand-in would let a mission pass that the real
block would stop.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

JOINT_ORDER = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


class Refused(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code, self.message = code, message


def parse_write(body):
    if not isinstance(body.get("angles"), list):
        raise Refused(400, "angles must be a list of six values, or five with hold=['gripper']")
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


class World:
    """One model, one clock, N arms. The only place that steps."""

    def __init__(self, path, arms, rate_hz=250.0):
        try:
            import mujoco
        except Exception as exc:                                     # noqa: BLE001
            raise SystemExit("sim: mujoco is not importable (%s) — refusing to run without physics" % exc)
        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        self.lock = threading.RLock()
        self.actuated = self.model.nu >= len(JOINT_ORDER) * len(arms)
        self.arms = {}
        for name, (prefix, _port) in arms.items():
            idx = {}
            for i in range(self.model.njnt):
                jname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
                for joint in JOINT_ORDER:
                    if jname == prefix + joint:
                        idx[joint] = int(self.model.jnt_qposadr[i])
            missing = [j for j in JOINT_ORDER if j not in idx]
            if missing:
                raise SystemExit("sim: %s has no joint for %s (prefix %r)" % (name, missing, prefix))
            self.arms[name] = {"idx": idx, "prefix": prefix, "port": _port,
                               "target": {j: float(self.data.qpos[a]) for j, a in idx.items()},
                               "torque": False, "owner": None, "due": 0.0, "writes": 0}
        self.rate = rate_hz
        threading.Thread(target=self._loop, daemon=True).start()

    # -- the clock -------------------------------------------------------------------------------
    def _loop(self):
        dt = float(self.model.opt.timestep)
        period = 1.0 / self.rate
        t0 = time.time()
        steps = 0
        while True:
            with self.lock:
                for arm in self.arms.values():
                    if arm["torque"]:
                        self._apply(arm)                 # position-driven arm; a free one is left to gravity
                self.mujoco.mj_step(self.model, self.data)
                steps += 1
            target_steps = (time.time() - t0) / dt
            if steps < target_steps:                 # catch up in batches instead of sleeping per step
                continue
            time.sleep(period)

    def _apply(self, arm):
        """Position control: the arm's joints go where they were told, at the model's timestep.

        Honest about what this is: the arm does not *yield* to a contact (no torque limit is simulated), but
        everything it touches does — gravity, contacts and the block falling are real, which is what the phase
        exists to test. A torque-limited arm would need actuators and a controller per joint; that is a bigger
        change and it would put a control law between the supervisor and the physics.
        """
        for joint, adr in arm["idx"].items():
            self.data.qpos[adr] = arm["target"][joint]

    def present(self, name):
        with self.lock:
            idx = self.arms[name]["idx"]
            return {j: math.degrees(float(self.data.qpos[a])) for j, a in idx.items()}

    def command(self, name, angles):
        with self.lock:
            arm = self.arms[name]
            arm["target"].update({j: math.radians(v) for j, v in angles.items()})
            arm["writes"] += 1
        return self.present(name)

    def objects(self):
        with self.lock:
            out = []
            for b in range(self.model.nbody):
                label = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_BODY, b)
                if not label:
                    continue
                out.append({"label": label,
                            "x": round(float(self.data.xpos[b][0]), 4),
                            "y": round(float(self.data.xpos[b][1]), 4),
                            "z": round(float(self.data.xpos[b][2]), 4)})
            return out


def make_handler(world, name):
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
            arm = world.arms[name]
            left = max(0.0, arm["due"] - time.time())
            if self.path == "/owner":
                return self._send(200, {"owner": arm["owner"] if left > 0 else None,
                                        "seconds_left": round(left, 1)})
            if self.path == "/torque":
                return self._send(200, {"torque": arm["torque"]})
            if self.path == "/status":
                return self._send(200, {"connected": True, "calibrated": True, "arm": name,
                                        "mode": "actuated" if world.actuated else "qpos (kinematic)",
                                        "writes": arm["writes"],
                                        "timestep": float(world.model.opt.timestep),
                                        "arms_in_world": list(world.arms)})
            if self.path == "/objects":
                return self._send(200, {"objects": world.objects()})
            if self.path in ("/read", "/read_raw"):
                present = world.present(name)
                return self._send(200, {"angles": [present[j] for j in JOINT_ORDER],
                                        "normalized": present, "names": list(JOINT_ORDER)})
            return self._send(404, {"error": "not found"})

        def do_POST(self):
            try:
                body = self._body()
                arm = world.arms[name]
                if self.path == "/claim":
                    left = max(0.0, arm["due"] - time.time())
                    who = body.get("owner") or ""
                    if arm["owner"] and arm["owner"] != who and left > 0:
                        raise Refused(423, "arm is leased to %r for another %.1f s" % (arm["owner"], left))
                    arm["owner"] = who
                    arm["due"] = time.time() + float(body.get("ttl_s") or 15.0)
                    return self._send(200, {"owner": who, "seconds_left": round(arm["due"] - time.time(), 1)})
                if self.path == "/release":
                    if (body.get("owner") or "") == arm["owner"]:
                        arm["owner"], arm["due"] = None, 0.0
                    return self._send(200, {"owner": arm["owner"]})
                if self.path == "/torque":                       # the operator's switch, never leased
                    arm["torque"] = bool(body.get("on"))
                    return self._send(200, {"torque": arm["torque"], "failed": {}, "unreachable": {}})
                if self.path == "/write":
                    left = max(0.0, arm["due"] - time.time())
                    if arm["owner"] and left > 0 and self.headers.get("X-Owner") != arm["owner"]:
                        raise Refused(423, "this arm is leased to %r — send X-Owner: %s"
                                           % (arm["owner"], arm["owner"]))
                    if not arm["torque"]:
                        raise Refused(409, "torque is off — nothing is written while the servos are free")
                    angles = parse_write(body)
                    got = world.command(name, angles)
                    return self._send(200, {"status": "ok", "driven": angles,
                                            "present": {j: round(v, 3) for j, v in got.items()}})
                return self._send(404, {"error": "not found"})
            except Refused as exc:
                return self._send(exc.code, {"error": exc.message})

    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--arm-port", action="append", required=True, metavar="ARM=PORT[:PREFIX]",
                    help="repeatable. The prefix goes after a colon, e.g. so100_cell_b=8005:arm2_ — "
                         "a shell that eats empty strings (PowerShell does) cannot supply '' as an argument")
    ap.add_argument("--prefix", action="append", default=[], metavar="PREFIX",
                    help="alternative to the ':PREFIX' form; '-' means no prefix")
    ap.add_argument("--rate", type=float, default=250.0)
    ap.add_argument("--viewer", action="store_true",
                    help="open a MuJoCo window on this machine. Without it the physics runs headless: the "
                         "ports answer and nothing is drawn, which is the normal way to run a cell")
    args = ap.parse_args(argv)

    pairs, inline = [], []
    for item in args.arm_port:
        arm, _, rest = item.partition("=")
        port, _, prefix = rest.partition(":")
        pairs.append((arm, port))
        inline.append(prefix)
    if args.prefix:
        prefixes = ["" if x == "-" else x for x in args.prefix]
        if len(prefixes) != len(pairs):
            raise SystemExit("sim: give one --prefix per --arm-port ('-' for none)")
    else:
        prefixes = inline
    arms = {arm: (prefixes[i], int(port)) for i, (arm, port) in enumerate(pairs)}

    world = World(args.model, arms, args.rate)
    servers = []
    for name in arms:
        srv = ThreadingHTTPServer(("0.0.0.0", arms[name][1]), make_handler(world, name))
        servers.append(srv)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
    print("sim: options model=%s viewer=%s rate=%.0f" % (args.model, bool(args.viewer), args.rate), flush=True)
    print("sim: one world, %d qpos, arms on %s"
          % (world.model.nq, ", ".join("%s:%d" % (n, arms[n][1]) for n in arms)), flush=True)
    if args.viewer:
        import mujoco
        # 3.x loads the viewer lazily: `mujoco.viewer` is an AttributeError until the submodule is imported,
        # which is why getattr(mujoco, "viewer", None) reported "no viewer" on a perfectly good 3.1.7.
        try:
            import mujoco.viewer as viewer                     # noqa: F401
        except Exception:                                      # noqa: BLE001
            viewer = None
        if viewer is None or not hasattr(viewer, "launch_passive"):
            raise SystemExit(
                "sim: this mujoco (%s) has no viewer. The classic viewer lives in 3.x and moved to MuJoCo "
                "Studio in 4.x, so the pin is 'pip install \"mujoco>=3,<4\"' (README 10.2). Or run without "
                "--viewer, which is how a cell normally runs."
                % getattr(mujoco, "__version__", "unknown"))
        try:
            handle = viewer.launch_passive(world.model, world.data)
        except Exception as exc:                                   # noqa: BLE001
            raise SystemExit("sim: the viewer could not open (%s: %s). The physics is fine without it: drop "
                             "--viewer and use GET /objects." % (type(exc).__name__, exc))
        print("sim: viewer open — close the window or Ctrl+C to stop", flush=True)
        print("sim: window is %s; if you cannot see it, look in the taskbar" %
              ("running" if handle.is_running() else "already closed"), flush=True)
        try:
            while handle.is_running():
                with world.lock:                      # the stepping thread owns the data; draw between steps
                    handle.sync()
                time.sleep(1.0 / 60.0)
        except KeyboardInterrupt:
            pass
        finally:
            handle.close()
        return 0
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())

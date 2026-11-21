#!/usr/bin/env python3
"""The supervisor: owns the arms, judges skill requests, and is the only writer.

It is deliberately thin. Every judgement lives in `docker/ws/lib/skill_plan.py` (pure, tested headlessly);
this file only does what needs a clock, a socket or DDS: claims each arm's write path, serves the
`so100_cell_interfaces/SkillRequest` service, reads the arms, and executes what `plan()` accepted.

Why a supervisor at all: chapter 2 of the MCP spec asks that every physical command go through a local
owner. The lease at each arm's port already enforces that mechanically — so this node's job is to *hold*
that lease for the duration of a mission and to be the one place a skill becomes a packet.

    ROS:      /cell/skill (so100_cell_interfaces/srv/SkillRequest)
    HTTP:     POST :8002/claim, :8003/claim  (X-Owner: supervisor)
    reads:    GET  :800N/read_raw, GET :800N/scale

Env: SUPERVISOR_PORT_MAP (default "so100_cell=8002,so100_cell_b=8003"), SUPERVISOR_TTL_S (15).
Refuses to start if the interface package is missing or an arm does not answer: a supervisor that cannot
see an arm cannot own it.
"""
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "lib"))
import command_contract as cc  # noqa: E402
import joint_scale as js      # noqa: E402
import skill_plan as sp       # noqa: E402

import rclpy                  # noqa: E402
from rclpy.node import Node   # noqa: E402

try:
    from so100_cell_interfaces.msg import SolveRequest, SolveResult
    from so100_cell_interfaces.srv import SkillRequest
except Exception:                                                        # noqa: BLE001
    print("supervisor: so100_cell_interfaces is not installed — build ros/so100_cell_interfaces first",
          flush=True)
    raise SystemExit(1)

OWNER = "supervisor"
TTL = float(os.environ.get("SUPERVISOR_TTL_S", "15"))
PORTS = dict(part.split("=") for part in
             os.environ.get("SUPERVISOR_PORT_MAP", "so100_cell=8002,so100_cell_b=8003").split(","))
PORTS = {k.strip(): int(v) for k, v in PORTS.items()}
# cell id -> ROS namespace. Two names for the same arm (the port is per serial, the topic per namespace),
# so the map is written once here instead of being guessed at every use.
NS = {"so100_cell": "arm1", "so100_cell_b": "arm2"}


def http(port, path, body=None, timeout=8.0):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request("http://127.0.0.1:%d%s" % (port, path), data=data,
                                 headers={"Content-Type": "application/json",
                                          "X-Owner": OWNER} if data else {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as fh:
            return json.loads(fh.read() or b"{}")
    except urllib.error.HTTPError as exc:
        return {"http": exc.code, "body": json.loads(exc.read() or b"{}")}


class Supervisor(Node):
    def __init__(self):
        super().__init__("cell_supervisor")
        self.arms = list(PORTS)
        self.owned = []
        self.tickets = 0
        self.create_service(SkillRequest, "/cell/skill", self.on_skill)
        # A pose is dispatched to that arm's IK and only written when the correlated result says ok.
        self.pub_solve = {ns: self.create_publisher(SolveRequest, "/%s/ik/solve" % ns, 10)
                          for ns in NS.values()}
        self.pending = {}
        for ns in NS.values():
            self.create_subscription(SolveResult, "/%s/ik/result" % ns,
                                     lambda m, n=ns: self.on_result(n, m), 10)
        # A thread, not an rclpy timer: this image's Timer rejects the clock it is handed
        # ("'function' object has no attribute 'handle'"), and a lease renewal does not need a clock —
        # it needs to happen every TTL/3 seconds whether or not anything is spinning.
        self.renew()
        threading.Thread(target=self._renew_forever, daemon=True).start()
        self.get_logger().info("supervisor: owns %s (ttl %.0fs)" % (self.owned, TTL))

    def _renew_forever(self):
        while True:
            time.sleep(TTL / 3.0)
            try:
                self.renew()
            except Exception as exc:                                     # noqa: BLE001
                self.get_logger().warn("supervisor: renewal failed: %s" % exc)

    # -- ownership ---------------------------------------------------------------------------------
    def renew(self):
        for arm, port in PORTS.items():
            r = http(port, "/claim", {"owner": OWNER, "ttl_s": TTL})
            if r.get("owner") == OWNER and arm not in self.owned:
                self.owned.append(arm)
                self.get_logger().info("supervisor: claimed %s" % arm)
            elif r.get("owner") != OWNER:
                self.get_logger().warn("supervisor: cannot claim %s: %s" % (arm, r))
                if arm in self.owned:
                    self.owned.remove(arm)

    # -- context for the pure planner ---------------------------------------------------------------
    def context(self):
        present, scales, jaw = {}, {}, {}
        for arm, port in PORTS.items():
            raw = http(port, "/read_raw")
            if "normalized" in raw:
                present[arm] = raw["normalized"]
            table = js.load(js.path_for(arm))
            scales[arm] = table
            curve = table["joints"]["gripper"].get("curve")
            if curve:
                jaw[arm] = curve
        return {"arms": self.arms, "owned": list(self.owned), "scales": scales,
                "jaw": jaw, "present": present}

    # -- the answer from IK --------------------------------------------------------------
    def on_result(self, ns, msg):
        arm = self.pending.pop(msg.request_id, None)
        if arm is None:
            return                      # not ours (another client's request, or already handled)
        if not msg.ok:
            self.get_logger().warn("move_pose %s refused by IK: %s" % (msg.request_id, msg.detail))
            return
        if len(msg.q_deg) != 5:
            self.get_logger().warn("move_pose %s: IK answered %d joints, expected 5 — not writing"
                                   % (msg.request_id, len(msg.q_deg)))
            return
        # Joints in JOINT_ORDER, jaw held: the solver never drives the gripper.
        r = http(PORTS[arm], "/write",
                 {"angles": [float(v) for v in msg.q_deg], "hold": ["gripper"], "units": "sdk"})
        self.get_logger().info("move_pose %s on %s -> %s"
                               % (msg.request_id, arm, r.get("status") or r))

    # -- the service -------------------------------------------------------------------------------
    def on_skill(self, request, response):
        verdict = sp.plan(request.skill, request.mission_id, request.args_json, self.context())
        if not verdict["accepted"]:
            response.accepted, response.reason, response.ticket = False, verdict["reason"], ""
            self.get_logger().warn("skill %r refused: %s" % (request.skill, verdict["reason"]))
            return response
        action = verdict["action"]
        arm, port = action["arm"], PORTS[action["arm"]]
        if action["kind"] == "solve":
            self.tickets += 1
            ticket = "m%d-%s-%d" % (self.tickets, arm, int(time.time()))
            req = SolveRequest()
            req.request_id = ticket
            req.target.header.frame_id = action["frame"]
            req.target.header.stamp = self.get_clock().now().to_msg()
            (req.target.pose.position.x, req.target.pose.position.y,
             req.target.pose.position.z) = action["pose"][:3]
            (req.target.pose.orientation.x, req.target.pose.orientation.y,
             req.target.pose.orientation.z, req.target.pose.orientation.w) = action["pose"][3:]
            req.max_pos_err_mm, req.max_ori_err_deg = action["max_pos"], action["max_ori"]
            self.pending[ticket] = arm
            self.pub_solve[NS[arm]].publish(req)
            response.accepted, response.ticket = True, ticket
            response.reason = ("dispatched to %s/ik/solve; the write happens only when the correlated "
                               "result says ok" % NS[arm])
            self.get_logger().info("skill move_pose on %s -> %s (%s)" % (arm, NS[arm], ticket))
            return response
        if action["kind"] == "write":
            # The array has no names: the order IS the contract, so it comes from JOINT_ORDER and
            # never from sorted() — alphabetical would drive pan with the wrist's number.
            order = [n for n in cc.JOINT_ORDER if n in action["driven"]]
            r = http(port, "/write", {"angles": [action["driven"][n] for n in order],
                                      "hold": action["held"], "units": "sdk"})
        else:
            raw = http(port, "/read_raw").get("normalized", {})
            angles = [raw.get(n, 0.0) for n in ("shoulder_pan", "shoulder_lift", "elbow_flex",
                                                "wrist_flex", "wrist_roll")]
            r = http(port, "/write", {"angles": angles + [action["pct"]], "units": "sdk"})
        ok = r.get("status") == "ok" or "status" in r
        self.tickets += 1
        response.accepted = bool(ok)
        response.reason = "ok" if ok else str(r)
        response.ticket = "m%d-%s-%d" % (self.tickets, arm, int(time.time()))
        self.get_logger().info("skill %r on %s -> %s (%s)"
                               % (request.skill, arm, "ok" if ok else r, response.ticket))
        return response


def main():
    rclpy.init()
    try:
        node = Supervisor()
    except Exception as exc:                                             # noqa: BLE001
        print("supervisor: refusing to start: %s" % exc, flush=True)
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Release first, always: a supervisor that dies holding the leases leaves both arms answering 423
        # to the operator until the TTL expires.
        for port in PORTS.values():
            try:
                http(port, "/release", {"owner": OWNER})
            except Exception as exc:                                     # noqa: BLE001
                print("supervisor: could not release port %d: %s" % (port, exc), flush=True)
        node.destroy_node()
        if rclpy.ok():
            # SIGINT already tore the context down in some builds; calling it twice raises a
            # RCLError that reads like a crash and is not.
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

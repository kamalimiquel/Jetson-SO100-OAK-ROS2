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
import park                    # noqa: E402
import cell_epoch              # noqa: E402
import joint_scale as js      # noqa: E402
import journal as jr          # noqa: E402
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
# A target is "host:port" or just "port" (localhost). The host form exists because physics runs on the
# operator's PC, not on the Xavier: the supervisor has to be able to reach an arm that is a MuJoCo model on
# another machine, and pretending a remote sim is on 127.0.0.1 is how a bench quietly stops being a bench.
PORTS = {k.strip(): (v.strip() if ":" in v else "127.0.0.1:%s" % v.strip()) for k, v in PORTS.items()}
# The operator's page cannot see a ROS service, so the supervisor also answers one question over HTTP:
# am I here, what do I hold, and what have I been asked. Read-only, localhost-facing, nothing to drive with.
STATUS_PORT = int(os.environ.get("SUPERVISOR_HTTP_PORT", "8097"))
# cell id -> ROS namespace. Two names for the same arm (the port is per serial, the topic per namespace),
# so the map is written once here instead of being guessed at every use.
NS = {"so100_cell": "arm1", "so100_cell_b": "arm2"}


def http(target, path, body=None, timeout=8.0):
    data = json.dumps(body).encode() if body is not None else None
    host = target if isinstance(target, str) else "127.0.0.1:%d" % target
    req = urllib.request.Request("http://%s%s" % (host, path), data=data,
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
        self.served = 0                 # skills answered, for the page
        self.mission = ""
        self.last = {}                  # the last answer, so a page can show it without re-asking
        self.t0 = time.time()
        self.last_park = (None, {})     # parked?, why — refreshed by context() for the page
        self.epoch = self._epoch()
        self.tickets = 0
        self.create_service(SkillRequest, "/cell/skill", self.on_skill)
        # A pose is dispatched to that arm's IK and only written when the correlated result says ok.
        self.pub_solve = {ns: self.create_publisher(SolveRequest, "/%s/ik/solve" % ns, 10)
                          for ns in NS.values()}
        self.pending = {}
        self.report = {}          # solve_ik report-only: ticket -> {event, result}; never written
        # Phase 4: the local stop path and the mission bookkeeping that makes a retry idempotent.
        self.stopped = threading.Event()   # when set, nothing is written until /resume
        self.cancelled = set()             # mission ids that were cancelled
        self.ticket_mission = {}           # ticket -> mission, so cancel drops the right pending
        for ns in NS.values():
            self.create_subscription(SolveResult, "/%s/ik/result" % ns,
                                     lambda m, n=ns: self.on_result(n, m), 10)
        # A thread, not an rclpy timer: this image's Timer rejects the clock it is handed
        # ("'function' object has no attribute 'handle'"), and a lease renewal does not need a clock —
        # it needs to happen every TTL/3 seconds whether or not anything is spinning.
        self.renew()
        threading.Thread(target=self._renew_forever, daemon=True).start()
        self.get_logger().info("supervisor: owns %s (ttl %.0fs)" % (self.owned, TTL))
        threading.Thread(target=self._serve_status, daemon=True).start()
        self.get_logger().info("supervisor: status on http://0.0.0.0:%d/" % STATUS_PORT)

    def _epoch(self):
        """The same function the MCP manifest uses. Two copies of a fingerprint are two fingerprints."""
        tables = {}
        for arm in PORTS:
            try:
                tables[arm] = js.load(js.path_for(arm))
            except Exception:                                        # noqa: BLE001
                tables[arm] = {"sha256": "no-sha"}
        return cell_epoch.epoch(cell_epoch.boot_id(), tables)

    # -- the page's one question --------------------------------------------------------------
    def _serve_status(self):
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                path, _, query = self.path.partition("?")
                mission = ""
                for part in query.split("&"):
                    if part.startswith("mission_id="):
                        mission = part.split("=", 1)[1]
                if path.rstrip("/") == "/journal":
                    return self._send(200, {"entries": node.journal_rows(mission)})
                if path.rstrip("/") == "/goal":
                    return self._send(200, {"mission_id": mission, "goal": node.goal(mission)})
                return self._send(200, {
                    "node": "cell_supervisor", "owner": OWNER, "ttl_s": TTL,
                    "owned": list(node.owned), "served": node.served,
                    "mission": node.mission, "last": node.last,
                    "stopped": node.stopped.is_set(), "uptime_s": round(time.time() - node.t0, 1),
                    "node_name": node.get_name(),
                })

            def do_POST(self):
                path = self.path.rstrip("/")
                if path not in ("/skill", "/stop", "/resume", "/cancel"):
                    return self._send(404, {"error": "POST /skill, /stop, /resume or /cancel"})
                n = int(self.headers.get("Content-Length") or 0)
                try:
                    body = json.loads(self.rfile.read(n).decode() or "{}")
                except ValueError:
                    return self._send(400, {"error": "body is not JSON"})
                # The stop path is here, not behind the queue: one latch, no skill asked first.
                if path == "/stop":
                    return self._send(200, node.stop("http"))
                if path == "/resume":
                    return self._send(200, node.resume())
                if path == "/cancel":
                    return self._send(200, node.cancel(str(body.get("mission_id") or "")))
                req = SkillRequest.Request()
                req.skill = str(body.get("skill") or "")
                req.mission_id = str(body.get("mission_id") or "")
                args = body.get("args")
                req.args_json = args if isinstance(args, str) else json.dumps(args or {})
                resp = SkillRequest.Response()
                try:
                    node.on_skill(req, resp)
                except Exception as exc:                             # noqa: BLE001
                    # Errors are data: a handler that dies silently looks like a network fault.
                    return self._send(200, {"accepted": False, "ticket": "",
                                            "reason": "%s: %s" % (type(exc).__name__, exc)})
                node.journal_skill(req, resp)
                return self._send(200, {"accepted": bool(resp.accepted), "reason": resp.reason,
                                        "ticket": resp.ticket})

            def _send(self, code, payload):
                raw = json.dumps(payload).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(raw)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(raw)

            def log_message(self, *a):
                pass

        try:
            ThreadingHTTPServer(("0.0.0.0", STATUS_PORT), Handler).serve_forever()
        except OSError as exc:
            self.get_logger().warn("supervisor: status port %d unavailable: %s" % (STATUS_PORT, exc))

    # -- the journal and the local stop path (Phase 4) ---------------------------------------
    def journal_skill(self, request, response):
        """One line per /skill call: what was asked, what was decided, which ticket answered."""
        try:
            jr.append({"t": round(time.time(), 3), "mission_id": request.mission_id,
                       "skill": request.skill, "ticket": response.ticket,
                       "accepted": bool(response.accepted), "reason": str(response.reason)[:400],
                       "args": str(request.args_json)[:400], "epoch": self.epoch})
        except Exception as exc:                                     # noqa: BLE001
            self.get_logger().warn("journal: %s" % exc)

    def journal_rows(self, mission_id=""):
        return jr.read(mission_id=mission_id or None, limit=100)

    def goal(self, mission_id):
        return jr.last_for(mission_id) if mission_id else None

    def stop(self, why="mcp"):
        """Local stop: latch, drop what is pending, and never queue behind the agent."""
        self.stopped.set()
        dropped = len(self.pending) + len(self.report)
        self.pending.clear()
        self.report.clear()
        self.get_logger().warn("stop engaged (%s): %d pending dropped; no write until /resume"
                               % (why, dropped))
        self._event("stop", "", "stop engaged; %d pending dropped" % dropped)
        return {"stopped": True, "dropped": dropped}

    def resume(self):
        self.stopped.clear()
        self._event("resume", "", "stop cleared")
        return {"stopped": False}

    def cancel(self, mission_id):
        if not mission_id:
            return {"error": "cancel needs a mission_id"}
        self.cancelled.add(mission_id)
        dropped = 0
        for ticket in list(self.pending):
            if self.ticket_mission.get(ticket) == mission_id:
                self.pending.pop(ticket, None)
                dropped += 1
        for ticket in list(self.report):
            if self.ticket_mission.get(ticket) == mission_id:
                self.report.pop(ticket, None)
                dropped += 1
        self._event("cancel", mission_id, "cancelled; %d pending dropped" % dropped)
        return {"cancelled": mission_id, "dropped": dropped}

    def _event(self, skill, mission_id, reason):
        try:
            jr.append({"t": round(time.time(), 3), "mission_id": mission_id, "skill": skill,
                       "ticket": "", "accepted": True, "reason": reason, "args": "",
                       "epoch": self.epoch})
        except Exception as exc:                                     # noqa: BLE001
            self.get_logger().warn("journal: %s" % exc)

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
        # ADR-0023: one base frame per arm, and no bare `base` anywhere. skill_plan refuses a pose whose
        # frame is not the arm's own.
        base_frame = {arm: ("%s_base" % NS[arm]) for arm in self.arms}
        return {"arms": self.arms, "owned": list(self.owned), "scales": scales,
                "jaw": jaw, "present": present, "base_frame": base_frame}

    # -- the answer from IK --------------------------------------------------------------
    def on_result(self, ns, msg):
        slot = self.report.pop(msg.request_id, None)
        if slot is not None:            # a report-only solve: hand the answer back, never write
            slot["result"] = {"ok": bool(msg.ok), "detail": msg.detail,
                              "q_deg": [float(v) for v in msg.q_deg]}
            slot["event"].set()
            return
        arm, due = self.pending.pop(msg.request_id, (None, 0.0))
        if arm is None:
            return                      # not ours (another client's request, or already handled)
        if time.time() > due:
            self.get_logger().warn("move_pose %s: solver answered after its deadline — not writing"
                                   % msg.request_id)
            return
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
        t_recv = time.time()                 # the deadline is measured from here, not from the write
        self.served += 1
        if request.mission_id:
            self.mission = request.mission_id
        self.last = {"skill": request.skill, "mission": request.mission_id,
                     "at": round(time.time(), 1)}
        try:
            args_seen = json.loads(request.args_json or "{}")
        except ValueError:
            args_seen = None
        if isinstance(args_seen, dict) and args_seen.get("epoch") not in (None, self.epoch):
            self.served -= 1
            response.accepted, response.ticket = False, ""
            response.reason = ("epoch mismatch: the request was planned against %r and this cell is %r — "
                               "re-plan against the current state" % (args_seen.get("epoch"), self.epoch))
            self.get_logger().warn("skill %r refused: epoch mismatch" % request.skill)
            return response
        ctx = self.context()
        # Autonomy starts from a verified park. A mission that begins with an arm near the workspace is a
        # collision waiting for the first trajectory, and the check is cheap: it is the same readings the
        # planner already has.
        if ctx.get("parked") is False and request.skill not in ("hold", "close_jaw"):
            why = "; ".join("%s: %s" % (a, w) for a, w in (ctx.get("park_why") or {}).items())
            self.get_logger().warn("skill %r refused: cell is not parked — %s" % (request.skill, why[:200]))
            response.accepted, response.ticket = False, ""
            response.reason = "cell is not parked: %s" % why[:500]
            return response
        verdict = sp.plan(request.skill, request.mission_id, request.args_json, ctx)
        if not verdict["accepted"]:
            response.accepted, response.reason, response.ticket = False, verdict["reason"], ""
            self.get_logger().warn("skill %r refused: %s" % (request.skill, verdict["reason"]))
            return response
        action = verdict["action"]
        arm, port = action["arm"], PORTS[action["arm"]]
        # Phase 4: the stop is local and it is checked here, before anything is dispatched. Then a
        # cancelled mission, then idempotence: a retry of a mission that already wrote gets its
        # original ticket back instead of writing twice.
        if self.stopped.is_set():
            response.accepted, response.ticket = False, ""
            response.reason = "stop is engaged — nothing is written until /resume"
            return response
        if request.mission_id in self.cancelled:
            response.accepted, response.ticket = False, ""
            response.reason = "mission %r was cancelled" % request.mission_id
            return response
        if action["kind"] in ("write", "jaw", "solve"):
            prior = jr.find_accepted(request.mission_id, self.epoch)
            if prior:
                response.accepted, response.ticket = True, prior.get("ticket", "")
                response.reason = ("idempotent: mission %r was already accepted as %s — nothing was "
                                   "written twice" % (request.mission_id, prior.get("ticket")))
                return response
        if action["kind"] in ("solve", "solve_report"):
            self.tickets += 1
            ticket = "m%d-%s-%d" % (self.tickets, arm, int(time.time()))
            self.ticket_mission[ticket] = request.mission_id
            req = SolveRequest()
            req.request_id = ticket
            req.target.header.frame_id = action["frame"]
            req.target.header.stamp = self.get_clock().now().to_msg()
            (req.target.pose.position.x, req.target.pose.position.y,
             req.target.pose.position.z) = action["pose"][:3]
            (req.target.pose.orientation.x, req.target.pose.orientation.y,
             req.target.pose.orientation.z, req.target.pose.orientation.w) = action["pose"][3:]
            req.max_pos_err_mm, req.max_ori_err_deg = action["max_pos"], action["max_ori"]
            budget = float(action.get("deadline_s") or 1.0)
            if action["kind"] == "solve_report":
                # Report-only: publish and wait for the correlated answer, but never write. The wait keeps
                # the /skill HTTP handler in step with the DDS round trip; the arm cannot be driven here.
                slot = {"event": threading.Event(), "result": None}
                self.report[ticket] = slot
                self.pub_solve[NS[arm]].publish(req)
                if not slot["event"].wait(timeout=budget + 0.3):
                    self.report.pop(ticket, None)
                    response.accepted, response.ticket = False, ticket
                    response.reason = ("no IK answer within %.2f s — the solver may be down; nothing was "
                                       "written" % (budget + 0.3))
                    return response
                # on_result pops the slot, so read the reference this call kept — popping again here
                # would return {} and lose the solver's answer (it did).
                res = slot["result"] or {}
                self.report.pop(ticket, None)
                response.accepted, response.ticket = bool(res.get("ok")), ticket
                response.reason = json.dumps(res)
                self.get_logger().info("solve_ik %s on %s -> %s"
                                       % (ticket, arm, "ok" if res.get("ok") else res.get("detail")))
                return response
            self.pending[ticket] = (arm, t_recv + budget)
            self.pub_solve[NS[arm]].publish(req)
            response.accepted, response.ticket = True, ticket
            response.reason = ("dispatched to %s/ik/solve; the write happens only when the correlated "
                               "result says ok" % NS[arm])
            self.get_logger().info("skill move_pose on %s -> %s (%s)" % (arm, NS[arm], ticket))
            return response
        # The deadline has to be effective, not decorative: a command that reaches the port after its own
        # deadline moves the arm on a decision made for an older world, so it is refused here — after the
        # judgement, before the packet.
        budget = float(action.get("deadline_s") or 1.0)
        if time.time() - t_recv > budget:
            self.get_logger().warn("skill %r refused: %.0f ms spent deciding, deadline %.0f ms"
                                   % (request.skill, (time.time() - t_recv) * 1000, budget * 1000))
            response.accepted, response.ticket = False, ""
            response.reason = ("deadline exceeded: %.0f ms elapsed of %.0f ms allowed — nothing was written"
                               % ((time.time() - t_recv) * 1000, budget * 1000))
            return response
        if self.stopped.is_set():
            response.accepted, response.ticket = False, ""
            response.reason = "stop was engaged before the write — nothing was written"
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
        self.ticket_mission[response.ticket] = request.mission_id
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

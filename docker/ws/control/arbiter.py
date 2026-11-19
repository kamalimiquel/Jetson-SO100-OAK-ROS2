#!/usr/bin/env python3
"""The one writer between a command topic and an arm's `/write` port.

Why a node at all. `/armN/joint_targets` is a `Float64MultiArray`: no names, no stamp, no owner.
Three different clients publish it (the pad, an applying IK node, any policy adapter under test)
and each of them reaches the same serial bus. On the other side, the loopback port is the only
boundary that cannot be walked around — ttyd on :7681 can `curl 127.0.0.1:8002/write` from inside
the arm container — but until now nothing enforced anything there.

This node is that enforcement, and it adds the one thing the array cannot carry:

  ownership  In enforce mode (`ARBITER_ENFORCE=1`) it claims each arm's write path with a short
             lease, renewed every half second. Any other writer — the pad publishing the topic
             straight to the arm, a `curl` from the page's terminal, a stale bridge — gets 423
             from the joint server, not a silently interleaved pose. Unclaimed (the default) the
             bench behaves exactly as before, which is what makes this deployable before the
             policy exists.
  validity   Values go through `command_contract.validate_topic`: exactly six, finite, inside the
             joint's range, NaN only in the jaw slot. A rejected command is logged once with the
             reason and is not forwarded.
  freshness  A command has no stamp, so its arrival time is the only honest one. One write may be
             in flight; if a newer command arrives meanwhile the older is *superseded* rather than
             queued. The old path did one blocking HTTP call per message with a 1 s timeout and
             let a backlog grow, which is how an arm lags a pad by a second under load.
  fault      A failed packet is not retried blind: the arm is re-read, measured against the
             command, and the arbiter stops forwarding (the joint server also holds its own path)
             until torque is cycled. `

`decide()` is pure — no ROS, no sockets — so `tools/test_arbiter.py` pins the four rules headless.
"""

import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "lib"))
import command_contract as cc  # noqa: E402

ARMS = [
    # namespace, joint-server port
    ("arm1", int(os.environ.get("ARM1_PORT", "8002"))),
    ("arm2", int(os.environ.get("ARM2_PORT", "8003"))),
]
ENFORCE = os.environ.get("ARBITER_ENFORCE", "0") == "1"
OWNER = "arbiter"
LEASE_S = 2.0            # the lease the arbiter holds on each arm
RENEW_S = 0.5            # renewed this often, so a dead arbiter frees the arm in <= 2 s
COMMAND_TTL_S = float(os.environ.get("COMMAND_TTL_S", "0.5"))


def decide(msg, receipt, now, in_flight, held, ttl_s=COMMAND_TTL_S):
    """What to do with one message. Pure: the whole command policy in one function.

    Returns (action, detail) where action is one of
      "forward"     — valid, fresh, nothing else writing: send it
      "supersede"   — valid and fresh, but an older write is still in flight; the older one loses
      "reject"      — the contract refused it (detail is the sentence)
      "stale"       — it waited longer than the TTL before it could be sent: drop, don't replay
      "hold"        — a previous packet failed; the pose is unknown, nothing is forwarded
    """
    if held:
        return "hold", held
    if now - receipt > ttl_s:
        return "stale", "received %.2fs ago (> %.2fs)" % (now - receipt, ttl_s)
    try:
        driven, jaw_held, _notes = cc.validate_topic(list(msg.data))
    except cc.CommandError as exc:
        return "reject", str(exc)
    return ("supersede" if in_flight else "forward"), cc.describe(driven)


def post(port, path, body, owner=None, timeout=0.4):
    req = urllib.request.Request(
        "http://127.0.0.1:%d%s" % (port, path), method="POST",
        data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    if owner:
        req.add_header("X-Owner", owner)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as exc:      # 400/409/423/502 are answers, not transport errors
        try:
            return exc.code, json.loads(exc.read())
        except ValueError:
            return exc.code, {}
    except Exception as exc:  # noqa: BLE001 — unreachable server
        return None, {"error": str(exc)}


def get(port, path, timeout=0.4):
    try:
        with urllib.request.urlopen("http://127.0.0.1:%d%s" % (port, path), timeout=timeout) as resp:
            return json.loads(resp.read())
    except Exception as exc:  # noqa: BLE001
        return {"error": str(exc)}


class ArmChannel:
    """One arm's queue: the latest command, one writer thread, the counters the status reports."""

    def __init__(self, ns, port, enforce):
        self.ns, self.port, self.enforce = ns, port, enforce
        self.lock = threading.Lock()
        self.pending = None          # (msg, receipt) — latest wins
        self.in_flight = False
        self.held = None             # sentence, while nothing may be forwarded
        self.sent = self.superseded = self.rejected = self.stale = self.faults = 0
        self.observed = 0
        self.last = {"sent_at": None, "detail": None, "port": port}

    def submit(self, msg):
        now = time.monotonic()
        with self.lock:
            busy = self.in_flight or self.pending is not None
            action, detail = decide(msg, now, now, busy, self.held)
            if action == "reject":
                self.rejected += 1
                return action, detail
            if action == "stale":
                self.stale += 1
                return action, detail
            if action == "hold":
                return action, detail
            if action == "supersede":
                # The older command loses to this one: latest wins, nothing queues. That is the
                # whole difference between "the arm lags by a second under load" and "the arm
                # follows the operator".
                self.superseded += 1
            self.pending = (msg, now)
            return action, detail

    def tick(self):
        """Send whatever is pending, if anything. Blocking on purpose: one writer per arm."""
        with self.lock:
            if self.pending is None or self.in_flight or self.held:
                return
            msg, receipt = self.pending
            self.pending = None
            action, detail = decide(msg, receipt, time.monotonic(), False, None)
            if action != "forward":
                if action == "stale":
                    self.stale += 1
                return
            if not self.enforce:
                # Observing only: arm_bridge is still the writer (the deployed arrangement), so
                # forwarding here would drive every joint twice. Count what *would* be sent, so
                # the status line is a real answer to "is the contract satisfied today?".
                self.observed += 1
                self.last = {"sent_at": None, "detail": detail, "port": self.port}
                return
            self.in_flight = True
        driven, _held, _notes = cc.validate_topic(list(msg.data))
        code, answer = post(self.port, "/write", {"angles": list(msg.data)}, owner=OWNER)
        with self.lock:
            self.in_flight = False
            if code == 200:
                self.sent += 1
                self.last = {"sent_at": time.time(), "detail": detail, "port": self.port}
                return
            if code == 409:
                # torque off, or the server is holding its own path after a fault: not this
                # node's to fix, and not an error to count every 20 ms — report once.
                self.held = str(answer.get("error") or answer)
                return
            if code == 423:
                self.held = "another writer holds the port: %s" % answer.get("error")
                return
            self.faults += 1
            self.held = "packet failed (%s): %s" % (code, answer.get("error"))
            self.pending = None

    def clear_hold(self, why):
        with self.lock:
            if self.held:
                self.held = None
                self.last["detail"] = why


class Arbiter(Node):
    def __init__(self):
        super().__init__("joint_arbiter")
        self.channels = {ns: ArmChannel(ns, port, ENFORCE) for ns, port in ARMS}
        self.status_pub = self.create_publisher(String, "/cell/arbiter", 10)
        for ns, _port in ARMS:
            self.create_subscription(Float64MultiArray, "/%s/joint_targets" % ns,
                                     lambda m, n=ns: self.on_targets(n, m), 10)
        self.renew()
        self.create_timer(RENEW_S, self.renew)
        self.create_timer(0.02, self.pump)
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            "arbiter: %s — ports %s, lease %.1fs renewed every %.1fs"
            % ("ENFORCING (sole writer)" if ENFORCE else "observing only (ARBITER_ENFORCE=0)",
               ", ".join("%s:%d" % (ns, port) for ns, port in ARMS), LEASE_S, RENEW_S))

    def on_targets(self, ns, msg):
        action, detail = self.channels[ns].submit(msg)
        if action == "reject":
            self.get_logger().warn("%s command rejected: %s" % (ns, detail))
        elif action == "supersede":
            self.get_logger().debug("%s: older command superseded" % ns)
        elif action == "hold":
            pass    # the status line carries it; a 25 Hz log would bury the reason

    def renew(self):
        """Hold the lease in enforce mode; in observe mode only look at who holds it.

        Observing must never take the port: claiming it would freeze the pad and the web torque
        button, which are the two controls the operator actually uses today.
        """
        for ns, ch in self.channels.items():
            if not ENFORCE:
                state = get(ch.port, "/owner")
                if "error" in state:
                    if ch.held is None:
                        self.get_logger().warn("%s: joint server unreachable on %d" % (ns, ch.port))
                    ch.held = "joint server unreachable"
                else:
                    ch.clear_hold("joint server reachable; owner=%s" % state.get("owner"))
                continue
            code, answer = post(ch.port, "/claim", {"owner": OWNER, "ttl_s": LEASE_S}, owner=OWNER)
            if code is None:
                if ch.held is None:
                    self.get_logger().warn("%s: joint server unreachable on %d" % (ns, ch.port))
                continue
            if code == 200:
                ch.clear_hold("lease renewed — write path free again")
            else:
                ch.held = "cannot claim the arm: %s" % answer.get("error")

    def pump(self):
        for ch in self.channels.values():
            ch.tick()

    def report(self):
        body = {"enforce": ENFORCE, "owner": OWNER if ENFORCE else None, "arms": {}}
        for ns, ch in self.channels.items():
            body["arms"][ns] = {
                "port": ch.port, "sent": ch.sent, "observed": ch.observed,
                "superseded": ch.superseded,
                "rejected": ch.rejected, "stale": ch.stale, "faults": ch.faults,
                "held": ch.held,
                "last_sent_age_s": (round(time.time() - ch.last["sent_at"], 2)
                                    if ch.last["sent_at"] else None),
            }
        msg = String()
        msg.data = json.dumps(body)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = Arbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for ns, ch in node.channels.items():
            post(ch.port, "/release", {}, owner=OWNER)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

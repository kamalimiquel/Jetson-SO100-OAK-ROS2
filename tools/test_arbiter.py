#!/usr/bin/env python3
"""Headless test of the arbiter — no ROS, no arms, no joint server.

The arbiter is the answer to `Float64MultiArray` having no names, no stamp and no owner: it is the
only writer to each arm's loopback port, it validates every command against the shared contract, it
lets a newer command supersede one in flight instead of queueing a backlog, and it stops writing
when a packet failed and the pose is unknown. Those four rules are pure, so they are pinned here.

    python3 tools/test_arbiter.py
"""

import importlib.util
import os
import sys
import types

ARB = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "docker", "ws", "control", "arbiter.py")
LIB = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "docker", "ws", "lib")

fails = []


def check(name, got, want):
    ok = got == want
    print(("  ok   " if ok else "  FAIL ") + "%s: %r" % (name, got) +
          ("" if ok else " (expected %r)" % (want,)))
    if not ok:
        fails.append(name)


class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("info", m))

    def warn(self, m):
        self.lines.append(("warn", m))

    def error(self, m):
        self.lines.append(("error", m))

    def debug(self, m):
        self.lines.append(("debug", m))


class FakeNode:
    def __init__(self, name):
        self.node_name = name
        self.subs, self.pubs, self.timers = {}, {}, []
        self.log = Log()

    def create_publisher(self, cls, name, qos):
        self.pubs[name] = Pub()
        return self.pubs[name]

    def create_subscription(self, cls, name, cb, qos):
        self.subs[name] = cb
        return object()

    def create_timer(self, period, cb):
        self.timers.append((period, cb))
        return object()

    def get_logger(self):
        return self.log

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=0))

    def destroy_node(self):
        pass


def load():
    rclpy = types.ModuleType("rclpy")
    for fn in ("init", "spin", "shutdown"):
        setattr(rclpy, fn, lambda *a, **k: None)
    node = types.ModuleType("rclpy.node")
    node.Node = FakeNode
    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.Float64MultiArray = type("Float64MultiArray", (Msg,), {})
    std_msg.String = type("String", (Msg,), {})
    sys.path.insert(0, LIB)
    for name, mod in {"rclpy": rclpy, "rclpy.node": node, "std_msgs": std,
                      "std_msgs.msg": std_msg}.items():
        sys.modules[name] = mod
    sys.modules["rclpy"].node = node
    sys.modules["std_msgs"].msg = std_msg
    spec = importlib.util.spec_from_file_location("arbiter", ARB)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["arbiter"] = mod
    spec.loader.exec_module(mod)
    return mod


def array(mod, values):
    m = mod.Float64MultiArray()
    m.data = list(values)
    return m


def main():
    mod = load()
    six = [10.0, -20.0, 30.0, -40.0, 50.0, 60.0]

    print("decide(): the whole command policy, no sockets")
    action, detail = mod.decide(array(mod, six), receipt=1.0, now=1.0, in_flight=False, held=None)
    check("a valid, fresh command is forwarded", action, "forward")
    check("and described by joint name", detail.startswith("shoulder_pan=10.0"), True)

    action, detail = mod.decide(array(mod, six[:5]), 1.0, 1.0, False, None)
    check("five values are rejected", action, "reject")
    check("with the contract's own sentence (NaN is the topic's way to say 'hold')",
          "NaN" in detail, True)

    action, detail = mod.decide(array(mod, [float("nan")] + six[1:]), 1.0, 1.0, False, None)
    check("NaN on an arm joint is rejected", (action, "shoulder_pan" in detail), ("reject", True))

    action, _ = mod.decide(array(mod, six), receipt=1.0, now=1.6, in_flight=False, held=None)
    check("a command older than the TTL is dropped, not replayed", action, "stale")

    action, detail = mod.decide(array(mod, six), 1.0, 1.0, False, "packet failed")
    check("a held write path forwards nothing", (action, detail), ("hold", "packet failed"))

    action, _ = mod.decide(array(mod, six), 1.0, 1.0, True, None)
    check("a command arriving over one in flight supersedes it", action, "supersede")

    print("the channel: latest wins, and a refusal stops the path")
    calls = []

    def fake_post(port, path, body, owner=None, timeout=0.4):
        calls.append((port, path, body, owner))
        return fake_post.answer.pop(0)

    mod.post = fake_post

    ch = mod.ArmChannel("arm1", 8002, enforce=True)
    fake_post.answer = [(200, {"status": "ok"})]
    ch.submit(array(mod, six))
    ch.tick()
    check("one 200 sends one write", (ch.sent, ch.faults), (1, 0))
    check("the write goes to the arm's own port", calls[-1][:2], (8002, "/write"))
    check("with the arbiter's owner header", calls[-1][3], "arbiter")

    # two commands, one tick: the second replaces the first, and only one write is sent
    fake_post.answer = [(200, {"status": "ok"})]
    ch.submit(array(mod, six))
    ch.submit(array(mod, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]))
    ch.tick()
    check("a newer command replaces the pending one", ch.superseded, 1)

    fake_post.answer = [(409, {"error": "torque is off"})]
    ch.submit(array(mod, six))
    ch.tick()
    check("409 torque-off holds the path without counting a fault", (ch.faults, ch.held is not None),
          (0, True))
    action, _ = mod.decide(array(mod, six), 1.0, 1.0, False, ch.held)
    check("and nothing is forwarded while held", action, "hold")
    ch.clear_hold("torque cycled")
    fake_post.answer = [(200, {"status": "ok"})]
    ch.submit(array(mod, six))
    ch.tick()
    check("after the hold clears, writes flow again", ch.sent, 3)

    fake_post.answer = [(423, {"error": "policy holds this arm (1.4s left)"})]
    ch.submit(array(mod, six))
    ch.tick()
    check("another writer's lease is reported as the reason", "another writer" in (ch.held or ""), True)
    ch.clear_hold("lease seen")

    fake_post.answer = [(502, {"error": "write failed; measured differs"})]
    ch.submit(array(mod, six))
    ch.tick()
    check("a failed packet is a fault, and the path is held", (ch.faults, ch.held is not None),
          (1, True))
    ch.clear_hold("operator cycled torque")

    fake_post.answer = [(400, {"error": "shoulder_pan: 190.0 is outside -170..170"})]
    ch.submit(array(mod, six))
    ch.tick()
    check("a 400 from the server is not silent either", ch.faults, 2)

    print("observing mode never takes the port")
    obs = mod.ArmChannel("arm2", 8003, enforce=False)
    before = len(calls)
    fake_post.answer = [(200, {"status": "ok"})]
    obs.submit(array(mod, six))
    obs.tick()
    check("no HTTP write while observing", len(calls) - before, 0)
    check("but the command is counted as observed", obs.observed, 1)
    check("and nothing is claimed as sent", obs.sent, 0)

    print("the node, as wired")
    node = mod.Arbiter()
    check("subscribes to both arms' targets", sorted(node.subs),
          ["/arm1/joint_targets", "/arm2/joint_targets"])
    check("publishes its own status", list(node.pubs), ["/cell/arbiter"])
    check("three timers: renew, pump, report", len(node.timers), 3)
    check("observing by default in this test", mod.ENFORCE, False)

    reads = []
    mod.get = lambda port, path, timeout=0.4: (reads.append((port, path)) or
                                               {"owner": None, "seconds_left": 0.0})
    calls.clear()
    node.renew()
    check("observing reads who holds each port", [p for _port, p in reads], ["/owner", "/owner"])
    check("and claims neither", calls, [])
    check("two arms, two ports read", sorted(port for port, _p in reads), [8002, 8003])

    fake_post.answer = [(200, {})]
    node.report()
    status = node.pubs["/cell/arbiter"].sent[-1].data
    check("the status line says enforce/observe and carries both arms",
          all(k in status for k in ('"enforce"', '"arm1"', '"arm2"')), True)

    print("\n%s" % ("all checks pass" if not fails else "%d check(s) FAILED" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

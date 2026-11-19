#!/usr/bin/env python3
"""Headless test of the merged cell state — no ROS, no arms.

Two rules live in `docker/ws/arm/joint_merge.py` and both used to be wrong in a way that produced
no error anywhere:

  * the merged message carried the **last** arrival's stamp, so a dead arm's frozen pose looked
    fresh under a live stamp;
  * a silent arm was never removed, so robot_state_publisher kept drawing it.

    python3 tools/test_joint_merge.py
"""

import importlib.util
import os
import sys
import types

MERGE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "docker", "ws", "arm", "joint_merge.py")

fails = []


def check(name, got, want):
    ok = got == want
    print(("  ok   " if ok else "  FAIL ") + "%s: %r" % (name, got) +
          ("" if ok else " (expected %r)" % (want,)))
    if not ok:
        fails.append(name)


# ---------------------------------------------------------------- fake rclpy
class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Stamp:
    def __init__(self, sec, nanosec=0):
        self.sec, self.nanosec = sec, nanosec


class Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def now(self):
        return types.SimpleNamespace(nanoseconds=int(self.t * 1e9))


class Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("info", m))

    def warn(self, m):
        self.lines.append(("warn", m))


class FakeNode:
    def __init__(self, name):
        self.node_name = name
        self.subs = {}
        self.pubs = {}
        self.clock = Clock()
        self.log = Log()

    def create_publisher(self, cls, name, qos):
        self.pubs[name] = Pub()
        return self.pubs[name]

    def create_subscription(self, cls, name, cb, qos):
        self.subs[name] = cb
        return object()

    def create_timer(self, period, cb):
        return object()

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return self.log

    def destroy_node(self):
        pass


def load():
    rclpy = types.ModuleType("rclpy")
    for fn in ("init", "spin", "shutdown"):
        setattr(rclpy, fn, lambda *a, **k: None)
    node = types.ModuleType("rclpy.node")
    node.Node = FakeNode
    sensor = types.ModuleType("sensor_msgs")
    sensor_msg = types.ModuleType("sensor_msgs.msg")
    def _js_init(self):
        self.header = Msg(stamp=None, frame_id="")
        self.name, self.position, self.velocity, self.effort = [], [], [], []

    sensor_msg.JointState = type("JointState", (Msg,), {"__init__": _js_init})
    for name, mod in {"rclpy": rclpy, "rclpy.node": node, "sensor_msgs": sensor,
                      "sensor_msgs.msg": sensor_msg}.items():
        sys.modules[name] = mod
    sys.modules["rclpy"].node = node
    sys.modules["sensor_msgs"].msg = sensor_msg
    spec = importlib.util.spec_from_file_location("joint_merge", MERGE)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["joint_merge"] = mod
    spec.loader.exec_module(mod)
    return mod


def state(mod, prefix, stamp, values):
    m = mod.JointState()
    m.header = Msg(stamp=stamp, frame_id="")
    m.name = ["%s_%s" % (prefix, n) for n in ("shoulder_pan", "gripper")]
    m.position = list(values)
    m.velocity = [0.0, 0.0]
    m.effort = [0.0, 0.0]
    return m


def main():
    mod = load()

    print("the pure rule")
    a = state(mod, "arm1", Stamp(10, 100), (1.0, 2.0))
    b = state(mod, "arm2", Stamp(10, 500), (3.0, 4.0))     # arrives later AND is newer
    names, pos, _vel, _eff, stamp, _frame, live, dropped = mod.merge(
        ["/a", "/b"], {"/a": a, "/b": b}, {"/a": 100.0, "/b": 100.2}, 100.3)
    check("both arms' joints are in the message", len(names), 4)
    check("names keep the order they first spoke", pos, [1.0, 2.0, 3.0, 4.0])
    check("the stamp is the OLDEST live arm, not the last arrival", (stamp.sec, stamp.nanosec),
          (10, 100))
    check("nothing dropped while both are inside the window", dropped, [])

    names, pos, _v, _e, stamp, _f, live, dropped = mod.merge(
        ["/a", "/b"], {"/a": a, "/b": b}, {"/a": 100.7, "/b": 100.2}, 100.9)
    check("0.7 s of silence is past the 0.5 s window", dropped, ["/b"])
    check("so its joints leave the message", len(names), 2)
    check("and the stamp is the surviving arm's", (stamp.sec, stamp.nanosec), (10, 100))

    names, _p, _v, _e, stamp, _f, live, dropped = mod.merge(
        ["/a", "/b"], {"/a": a, "/b": b}, {"/a": 100.8, "/b": 100.7}, 100.9)
    check("an arm that comes back is back", len(names), 4)
    check("with its own (older or newer) stamp accounted for", (stamp.sec, stamp.nanosec), (10, 100))

    check("a message with no live arm is empty, not a fake pose",
          mod.merge(["/a"], {"/a": a}, {"/a": 10.0}, 99.0)[0], [])

    print("the node")
    node = mod.JointMerge()
    check("subscribes to both radians topics", sorted(node.subs),
          ["/arm1/joint_states_rad", "/arm2/joint_states_rad"])
    check("publishes the merged topic", list(node.pubs), ["/cell/joint_states_rad"])

    node.clock.t = 300.0
    node.subs["/arm1/joint_states_rad"](state(mod, "arm1", Stamp(299, 0), (1.0, 2.0)))
    node.clock.t = 300.1
    node.subs["/arm2/joint_states_rad"](state(mod, "arm2", Stamp(300, 0), (3.0, 4.0)))
    out = node.pubs["/cell/joint_states_rad"].sent
    check("one message out per arrival", len(out), 2)
    check("the second one carries both arms", len(out[-1].name), 4)
    check("under arm 1's older stamp", (out[-1].header.stamp.sec, out[-1].header.stamp.nanosec),
          (299, 0))

    # arm 2 goes silent; arm 1 keeps publishing, which is what makes the drop observable
    node.clock.t = 301.2
    node.subs["/arm1/joint_states_rad"](state(mod, "arm1", Stamp(301, 100), (1.0, 2.0)))
    out = node.pubs["/cell/joint_states_rad"].sent
    check("the dead arm's joints are gone after 0.5 s", len(out[-1].name), 2)
    warns = [m for kind, m in node.log.lines if kind == "warn"]
    check("and the drop is said out loud, once", len(warns), 1)
    check("naming the silent topic", "/arm2/joint_states_rad" in warns[0], True)

    node.clock.t = 301.4
    node.subs["/arm1/joint_states_rad"](state(mod, "arm1", Stamp(301, 300), (1.0, 2.0)))
    warns = [m for kind, m in node.log.lines if kind == "warn"]
    check("a second silent tick does not repeat the warning", len(warns), 1)

    node.clock.t = 301.6
    node.subs["/arm2/joint_states_rad"](state(mod, "arm2", Stamp(301, 500), (3.0, 4.0)))
    out = node.pubs["/cell/joint_states_rad"].sent
    check("when it returns, both arms are back", len(out[-1].name), 4)
    infos = [m for kind, m in node.log.lines if kind == "info" and "publishing again" in m]
    check("and the return is logged too", len(infos), 1)

    print("\n%s" % ("all checks pass" if not fails else "%d check(s) FAILED" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

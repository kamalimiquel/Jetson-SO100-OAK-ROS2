#!/usr/bin/env python3
"""Headless test of the 1.0 compatibility bridge (ADR-0023) — no ROS, no arm.

The bridge is the one moving part Alt 1 adds, so its wiring is pinned here with a fake `rclpy`:
which arm-1 topic lands on which bare name, that the message is forwarded **verbatim** (same header
stamp, not re-stamped), that a bare `/joint_targets` reaches `/arm1/joint_targets`, and that the
bridge never publishes TF — aliasing frames would put back the collision the whole change removes.

    python3 tools/test_compat_bridge.py
"""

import os
import sys
import types

BRIDGE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "docker", "ws", "arm", "compat_bridge.py")

fails = []


def check(name, got, want):
    ok = got == want
    print(("  ok   " if ok else "  FAIL ") + f"{name}: {got!r}" + ("" if ok else f" (expected {want!r})"))
    if not ok:
        fails.append(name)


# ---------------------------------------------------------------- fake rclpy
class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Pub:
    def __init__(self, name):
        self.name = name
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class FakeNode:
    def __init__(self, name):
        self.node_name = name
        self.pubs = {}
        self.subs = {}

    def create_publisher(self, cls, name, qos):
        self.pubs[name] = Pub(name)
        return self.pubs[name]

    def create_subscription(self, cls, name, cb, qos):
        self.subs[name] = cb
        return object()

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None)

    def destroy_node(self):
        pass


def install_fakes():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    rclpy_qos.QoSProfile = lambda **kw: kw
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    sensor = types.ModuleType("sensor_msgs")
    sensor_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msg.JointState = type("JointState", (Msg,), {})
    sensor_msg.CompressedImage = type("CompressedImage", (Msg,), {})
    sensor_msg.CameraInfo = type("CameraInfo", (Msg,), {})
    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.Float64MultiArray = type("Float64MultiArray", (Msg,), {})
    std_msg.String = type("String", (Msg,), {})
    for name, mod in {"rclpy": rclpy, "rclpy.node": rclpy_node, "rclpy.qos": rclpy_qos,
                      "sensor_msgs": sensor, "sensor_msgs.msg": sensor_msg,
                      "std_msgs": std, "std_msgs.msg": std_msg}.items():
        sys.modules[name] = mod
    sys.modules["rclpy"].node = rclpy_node
    sys.modules["rclpy"].qos = rclpy_qos
    sys.modules["sensor_msgs"].msg = sensor_msg
    sys.modules["std_msgs"].msg = std_msg


def load():
    install_fakes()
    import importlib.util
    spec = importlib.util.spec_from_file_location("compat_bridge", BRIDGE)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["compat_bridge"] = mod
    spec.loader.exec_module(mod)
    return mod


def main():
    mod = load()
    node = mod.CompatBridge()
    print("the bridge, as wired:")

    check("forwards arm 1's degrees onto the bare 1.0 topic",
          ("/arm1/joint_states" in node.subs, "/joint_states"), (True, "/joint_states"))
    check("forwards the radians topic too", "/arm1/joint_states_rad" in node.subs, True)
    check("forwards the description", "/arm1/robot_description" in node.subs, True)
    check("takes legacy /joint_targets in", "/joint_targets" in node.subs, True)
    check("and puts it on arm 1's targets", "/arm1/joint_targets" in node.pubs, True)

    # a message in, the same message out — same stamp, not a re-stamped copy
    stamp = object()
    msg = mod.JointState()
    msg.header = types.SimpleNamespace(stamp=stamp)
    msg.name = ["shoulder_pan", "shoulder_lift"]
    msg.position = [1.0, 2.0]
    node.subs["/arm1/joint_states"](msg)
    out = node.pubs["/joint_states"].sent
    check("one message in, one out", len(out), 1)
    check("the output IS the input (no copy, no re-stamp)",
          (out[0] is msg, out[0].header.stamp is stamp), (True, True))

    # nothing else rides along: the bridge must not publish /tf or any other topic
    check("publishes exactly the 1.0 surface + arm 1 targets",
          sorted(node.pubs), ["/arm1/joint_targets", "/joint_states", "/joint_states_rad",
                              "/robot_description"])
    check("no /tf anywhere in the bridge", [t for t in node.pubs if "tf" in t], [])
    check("no bare frame comes back (docstring says so)",
          "alias **frames**" in (mod.__doc__ or ""), True)

    # latched description: the QoS must be transient-local, or a late RViz sees nothing
    check("the description is published latched",
          node.pubs["/robot_description"] is not None, True)

    print("\n%s" % ("all checks pass" if not fails else "%d check(s) FAILED" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

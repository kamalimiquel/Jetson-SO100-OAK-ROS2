#!/usr/bin/env python3
"""The released 1.0 surface, served from /arm1/* — the compatibility bridge of ADR-0023.

Since ADR-0023 both arms carry a namespace (`/arm1/*`, `arm2_*` and `/arm2/*`, `arm2_*` frames), so
no unprefixed topic or frame exists to collide with the other arm. The 1.0 surface that external
clients (and the 1.0 pad, twin and hand-eye tools) already use is **republished here**, from arm 1
only:

    /arm1/joint_states        -> /joint_states          JointState, degrees
    /arm1/joint_states_rad    -> /joint_states_rad      JointState, radians
    /arm1/robot_description   -> /robot_description     String, latched
    /joint_targets (in)       -> /arm1/joint_targets    Float64MultiArray, degrees

What it deliberately does NOT do: alias **frames**. `base` stops existing (arm 1's base is
`arm1_base`); a bridge that resurrected it would put back the very collision R2 is about. A 1.0-era
consumer that hardcoded `base` in RViz or `tf2_echo` must switch to `arm1_base`.

Cost, measured on the bench with `ros2 topic hz`: one extra hop on joint_states, same 50 Hz, same
header stamp — the message is forwarded, not regenerated. It runs only in arm 1's container
(entry.sh starts it for `ARM_NS` empty or `arm1`).

    ARM_COMPAT_STATES=/arm1/joint_states python3 compat_bridge.py     # single-arm boot
"""

import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

# (bridge name, source on the arm, published name, message class, latched?)
FORWARD = [
    ("states", "/arm1/joint_states", "/joint_states", JointState, False),
    ("states_rad", "/arm1/joint_states_rad", "/joint_states_rad", JointState, False),
    ("description", "/arm1/robot_description", "/robot_description", String, True),
]
# Legacy inputs: bare topics that a 1.0 client still publishes, mapped onto arm 1.
BACKWARD = [
    ("targets", "/joint_targets", "/arm1/joint_targets", Float64MultiArray, False),
]


def latched():
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CompatBridge(Node):
    """Re-publishes arm 1's messages on the bare 1.0 names, and bare commands back onto /arm1."""

    def __init__(self):
        super().__init__("compat_bridge")
        self.pubs = {}                      # keyed by the DESTINATION topic: that is what a test checks
        for _name, src, dst, cls, latch in FORWARD + BACKWARD:
            self.pubs[dst] = self.create_publisher(cls, dst, latched() if latch else 10)
            self.create_subscription(cls, src, self._forward(src, dst), latched() if latch else 10)
            self.get_logger().info(f"{src} -> {dst}")

    def _forward(self, src, dst):
        seen = {"n": 0}

        def cb(msg):
            self.pubs[dst].publish(msg)          # verbatim: same header, same stamp
            seen["n"] += 1
            if seen["n"] == 1:
                self.get_logger().info(f"first message {src} -> {dst} "
                                       f"({len(getattr(msg, 'name', []) or getattr(msg, 'data', []))} fields)")
            elif seen["n"] % 3000 == 0:
                self.get_logger().info(f"{src} -> {dst}: {seen['n']} messages")

        return cb


def main():
    rclpy.init()
    node = CompatBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

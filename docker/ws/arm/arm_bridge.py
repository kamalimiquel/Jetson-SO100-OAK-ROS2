#!/usr/bin/env python3
"""Arm ROS2 bridge (ADR-0006/0009) — image python3.8 + rclpy.

Polls the LeRobot joint server (loopback HTTP) at 50 Hz and publishes
/joint_states; forwards /joint_targets (Float64MultiArray, 6) to the server;
publishes the SO-101 URDF on /robot_description (latched) so
robot_state_publisher can emit TF for Foxglove.
"""

import json
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
SERVER = "http://127.0.0.1:8002"
URDF_PATH = "/opt/ws/arm/urdf/so101.urdf"


class ArmBridge(Node):
    def __init__(self):
        super().__init__("arm_bridge")
        self.pub_state = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_desc = self.create_publisher(
            String, "/robot_description",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(Float64MultiArray, "/joint_targets", self.on_targets, 10)
        self.create_timer(0.02, self.tick)
        self._warned = False

        try:
            with open(URDF_PATH) as fh:
                urdf = fh.read()
            msg = String()
            msg.data = urdf
            self.pub_desc.publish(msg)
            self.get_logger().info(f"published /robot_description ({len(urdf)} bytes)")
        except OSError as exc:
            self.get_logger().error(f"cannot read {URDF_PATH}: {exc}")

    def _http(self, method: str, path: str, payload=None):
        req = urllib.request.Request(
            SERVER + path, method=method,
            data=json.dumps(payload).encode() if payload is not None else None,
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=1.0) as resp:
            return json.loads(resp.read())

    def on_targets(self, msg: Float64MultiArray):
        try:
            self._http("POST", "/write", {"angles": list(msg.data)})
        except Exception as exc:  # noqa: BLE001
            # torque off answers 409 on every tick: warn once per distinct message
            msg = str(exc)
            if msg != getattr(self, "_last_write_err", None):
                self._last_write_err = msg
                self.get_logger().warn(f"write failed: {msg}")
            return
        if getattr(self, "_last_write_err", None):
            self._last_write_err = None
            self.get_logger().info("write ok again")

    def tick(self):
        try:
            data = self._http("GET", "/read")
        except Exception as exc:  # noqa: BLE001
            if not self._warned:
                self.get_logger().warn(f"joint server unreachable ({exc}) — retrying")
                self._warned = True
            return
        self._warned = False
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = JOINT_NAMES
        out.position = [float(v) for v in data["angles"]]
        self.pub_state.publish(out)


def main():
    rclpy.init()
    node = ArmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

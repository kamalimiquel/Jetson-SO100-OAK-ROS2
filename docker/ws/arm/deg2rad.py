"""deg -> rad bridge for robot_state_publisher.

arm_bridge publishes /joint_states in DEGREES (the LeRobot convention, useful
for teleop and for the MuJoCo twin, which converts explicitly). ROS tooling
(robot_state_publisher, RViz) interprets joint positions as RADIANS per
REP-103, so feeding degrees straight in rotates every link by ~57x too much —
that is the "RViz shows nonsense" symptom.

This node republishes the same message with radians; robot_state_publisher is
launched with `-r joint_states:=/joint_states_rad`.

Env: ARM_NS (topic namespace, default empty -> /joint_states -> /joint_states_rad;
     "arm2" -> /arm2/joint_states -> /arm2/joint_states_rad).
"""

import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

NS = os.environ.get("ARM_NS", "").strip("/")
# The cell URDF (so101_cell.urdf) names the second arm's joints arm2_*, so the merged joint-state
# topic carries those names and one robot_state_publisher can drive both arms.
JP = os.environ.get("JOINT_PREFIX", (NS + "_") if NS else "")


def topic(name: str) -> str:
    return f"/{NS}/{name}" if NS else f"/{name}"


class DegToRad(Node):
    def __init__(self):
        super().__init__("arm2_deg2rad" if NS else "deg2rad")
        self.pub = self.create_publisher(JointState, topic("joint_states_rad"), 10)
        self.sub = self.create_subscription(JointState, topic("joint_states"), self.on_msg, 10)
        self.get_logger().info(
            f"republishing {topic('joint_states')} (deg) as {topic('joint_states_rad')}"
        )

    def on_msg(self, msg: JointState):
        out = JointState()
        out.header = msg.header
        out.name = [(JP + n) if JP else n for n in msg.name]
        out.position = [math.radians(p) for p in msg.position]
        out.velocity = [math.radians(v) for v in msg.velocity]
        out.effort = msg.effort
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DegToRad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

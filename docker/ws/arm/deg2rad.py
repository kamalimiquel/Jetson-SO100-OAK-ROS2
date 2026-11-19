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


JAW = "gripper"
JAW_RAD_PER_PERCENT = math.radians(1.0)   # placeholder linear map; fit it from measured aperture


def topic(name: str) -> str:
    return f"/{NS}/{name}" if NS else f"/{name}"


class DegToRad(Node):
    def __init__(self):
        # The node name follows the namespace: hardcoding "arm2_deg2rad" made arm 1's instance
        # introduce itself as arm 2's (caught on the bench 2026-11-18, ADR-0023).
        super().__init__(f"{NS}_deg2rad" if NS else "deg2rad")
        self.pub = self.create_publisher(JointState, topic("joint_states_rad"), 10)
        self.sub = self.create_subscription(JointState, topic("joint_states"), self.on_msg, 10)
        self.get_logger().info(
            f"republishing {topic('joint_states')} (deg) as {topic('joint_states_rad')}"
        )
        self.get_logger().info(
            f"jaw: percent -> rad with %.4f per percent (uncalibrated proxy for the drawing)"
            % JAW_RAD_PER_PERCENT
        )

    def on_msg(self, msg: JointState):
        out = JointState()
        out.header = msg.header
        out.name = [(JP + n) if JP else n for n in msg.name]
        # The jaw is NOT an angle: /joint_states carries the LeRobot jaw percentage (0-100). Feeding
        # it through radians() happens to land inside the cell URDF's jaw limits (-10..100 deg), which
        # makes it look right, but that is a coincidence of how the model was authored, not a
        # calibration — re-homing the motor or fitting a different jaw gear moves the real aperture
        # without moving this number. So the map is written down and one constant can be fitted
        # later; until it is measured, TF's jaw is a drawing, not a measurement (spec §4).
        out.position = [JAW_RAD_PER_PERCENT * p if n == JAW else math.radians(p)
                        for n, p in zip(msg.name, msg.position)]
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

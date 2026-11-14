#!/usr/bin/env python3
"""Merge both arms' joint states into the single topic the cell URDF needs.

Arm 1 publishes /joint_states_rad and arm 2 /arm2/joint_states_rad, with arm 2's joint names
prefixed `arm2_` (deg2rad.py does that when ARM_NS is set). One robot_state_publisher reads the
merged topic and drives the whole two-arm tree (`so101_cell.urdf`), because RViz on this board only
draws the first RobotModel display — two separate models cannot both appear.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

TOPIC = "/cell/joint_states_rad"
SOURCES = ["/joint_states_rad", "/arm2/joint_states_rad"]


class JointMerge(Node):
    def __init__(self):
        super().__init__("joint_merge")
        self.pub = self.create_publisher(JointState, TOPIC, 10)
        self.latest = {}
        self.order = []
        for src in SOURCES:
            self.create_subscription(JointState, src, lambda m, s=src: self.on_msg(s, m), 10)
        self.get_logger().info("merging %s -> %s" % (", ".join(SOURCES), TOPIC))

    def on_msg(self, src, msg):
        if src not in self.latest:
            self.order.append(src)
        self.latest[src] = msg
        out = JointState()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        for key in self.order:
            m = self.latest[key]
            out.name.extend(m.name)
            out.position.extend(m.position)
            out.velocity.extend(m.velocity)
            out.effort.extend(m.effort)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = JointMerge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Live test of the running IK node: ask it for the pose the arm is in right now.

Reads /joint_states, computes that pose with the same chain the node uses, publishes it on
/ik/target and waits for /ik/status. The answer should be the current joint angles, and the
reported residual should be ~0 — if it is not, the node and the arm disagree about the model.

    docker exec so100-control bash -c "source /opt/ros/humble/install/setup.bash && \
        python3 /opt/tools/ik_livetest.py"
"""

import os
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

sys.path.insert(0, os.environ.get("IK_DIR", "/opt/ws/control"))
from ik_node import Chain, JOINTS  # noqa: E402


def mat_to_quat(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = (t + 1.0) ** 0.5 * 2.0
        return (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = (1.0 + R[0, 0] - R[1, 1] - R[2, 2]) ** 0.5 * 2.0
        return 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s
    if R[1, 1] > R[2, 2]:
        s = (1.0 + R[1, 1] - R[0, 0] - R[2, 2]) ** 0.5 * 2.0
        return (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s
    s = (1.0 + R[2, 2] - R[0, 0] - R[1, 1]) ** 0.5 * 2.0
    return (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s


class LiveTest(Node):
    def __init__(self):
        super().__init__("ik_livetest")
        self.chain = Chain()
        self.q = None
        self.status = None
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(String, "/ik/status", self.on_status, 10)
        self.pub = self.create_publisher(PoseStamped, "/ik/target", 10)
        self.sent = False

    def on_joints(self, msg):
        try:
            idx = [msg.name.index(j) for j in JOINTS]
        except ValueError:
            return
        self.q = [msg.position[i] for i in idx]

    def on_status(self, msg):
        self.status = msg.data

    def step(self):
        if self.q is None or self.sent:
            return
        T = self.chain.fk([__import__("math").radians(v) for v in self.q], jaw=0.0)
        qx, qy, qz, qw = mat_to_quat(T[:3, :3])
        msg = PoseStamped()
        msg.header.frame_id = "base"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = [float(v) for v in T[:3, 3]]
        msg.pose.orientation.x, msg.pose.orientation.y = float(qx), float(qy)
        msg.pose.orientation.z, msg.pose.orientation.w = float(qz), float(qw)
        self.get_logger().info("asking IK for the pose the arm is in: %s"
                               % ", ".join("%.1f" % v for v in self.q))
        self.pub.publish(msg)
        self.sent = True


def main():
    rclpy.init()
    node = LiveTest()
    for _ in range(80):                 # ~8 s: wait for joints, send, wait for the answer
        rclpy.spin_once(node, timeout_sec=0.1)
        node.step()
        if node.status:
            break
    if node.status:
        print("current joints: [%s]" % ", ".join("%.1f" % v for v in node.q))
        print("ik answered:    %s" % node.status)
        ok = "ok=True" in node.status
        print("LIVETEST %s" % ("PASS" if ok else "FAIL"))
        return 0 if ok else 1
    print("LIVETEST FAIL — no /ik/status within 8 s")
    return 1


if __name__ == "__main__":
    sys.exit(main())

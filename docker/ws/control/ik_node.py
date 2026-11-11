"""IK node: SE(3) pose -> joint angles for the SO-101 (the missing middle of ADR-0009).

The arm is driven in joint space (`/joint_targets`, degrees), so anything that wants to say
"put the gripper *there*" needs the inverse of FK. This node is that, and nothing else: it does
not plan, it does not touch torque, and it refuses to publish targets unless asked to.

    in   /ik/target     geometry_msgs/PoseStamped — pose of the tool tip in `base`
    out  /ik/solution   Float64MultiArray — 5 joint angles (degrees), always
    out  /ik/status     String — ok, iterations, position and orientation residual
    out  /joint_targets Float64MultiArray — 6 values, only when IK_APPLY=1

The gripper is never commanded: the 6th value echoes whatever /joint_states reports.

Method: damped least squares on the URDF chain, with the URDF's own joint limits. Five joints
cannot satisfy six constraints, so the solve is a least-squares fit — the residuals are reported
instead of pretending the pose was reached. Multi-start seeds are tried because 5-DOF chains have
plenty of local minima; the solution nearest the seed wins.

Env:
    IK_URDF     default /opt/ws/arm/urdf/so101.urdf
    IK_APPLY    "1" to also publish /joint_targets (leave unset to only report)
    IK_TOL_MM   default 2.0, IK_TOL_DEG default 2.0 — below this the solve counts as reached
"""

import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

URDF = os.environ.get("IK_URDF", "/opt/ws/arm/urdf/so101.urdf")
APPLY = os.environ.get("IK_APPLY", "") == "1"
TOL_MM = float(os.environ.get("IK_TOL_MM", "2.0"))
TOL_DEG = float(os.environ.get("IK_TOL_DEG", "2.0"))
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
# Orientation is weighted as metres-per-radian: a 5-DOF arm must trade orientation against
# position, and this is the knob that decides how much of the error lands where.
ORI_WEIGHT = 0.12
MAX_ITERS = 140
DAMP = 2e-3


def rpy2mat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_rot(axis, ang):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    x, y, z = a
    c, s = math.cos(ang), math.sin(ang)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


class Chain:
    """The revolute chain of the URDF, root link outwards, with the URDF's own limits.

    Parsed exactly the way tools/handeye_capture.py does it (same file, same order), so the FK
    here and the FK the calibration used cannot drift apart.
    """

    def __init__(self, path=URDF):
        root = ET.parse(path).getroot()
        joints, limits = {}, {}
        for j in root.findall("joint"):
            if j.get("type") != "revolute":
                continue
            o, a, lim = j.find("origin"), j.find("axis"), j.find("limit")
            child = j.find("child").get("link")
            joints[child] = (
                j.find("parent").get("link"),
                [float(v) for v in o.get("xyz").split()],
                [float(v) for v in o.get("rpy").split()],
                [float(v) for v in a.get("xyz").split()],
            )
            if lim is not None:
                limits[child] = (float(lim.get("lower")), float(lim.get("upper")))
        order, link = [], None
        while joints:
            nxt = next((c for c, (p, *_r) in joints.items() if link is None or p == link), None)
            if nxt is None:
                break
            order.append((nxt,) + joints.pop(nxt))
            link = nxt
        self.links = [o[0] for o in order]
        self.xyz = [np.array(o[2]) for o in order]
        self.rpy = [o[3] for o in order]
        self.axis = [np.array(o[4]) for o in order]
        self.limits = []
        for name in self.links:
            lo, hi = limits.get(name, (-math.pi, math.pi))
            self.limits.append((lo, hi))
        self.n = len(self.links)
        # the last revolute joint of this URDF is the jaw: it is held, not solved, so the IK
        # moves 5 joints and the gripper stays where it is (project rule: never command it)
        self.dof = max(1, self.n - 1)

    def fk(self, q, jaw=0.0):
        """(4x4) pose of the last link in `base`, radians. `q` may cover every joint or only
        the `dof` solved ones, in which case the jaw is appended at its held value."""
        q = list(q)
        if len(q) == self.n - 1:
            q.append(jaw)
        R = np.eye(3)
        p = np.zeros(3)
        for i in range(self.n):
            p = p + R @ self.xyz[i]
            R = R @ rpy2mat(*self.rpy[i]) @ axis_rot(self.axis[i], q[i])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p
        return T

    def fk_jac(self, q, jaw=0.0):
        """FK plus the geometric Jacobian of the tool frame, in one pass."""
        q = list(q)
        if len(q) == self.n - 1:
            q.append(jaw)
        R = np.eye(3)
        p = np.zeros(3)
        axes, orig = [], []
        for i in range(self.n):
            p = p + R @ self.xyz[i]
            R = R @ rpy2mat(*self.rpy[i])
            a = self.axis[i] / np.linalg.norm(self.axis[i])
            axes.append(R @ a)
            orig.append(p.copy())
            R = R @ axis_rot(self.axis[i], q[i])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p
        Jv = np.zeros((3, self.dof))
        Jw = np.zeros((3, self.dof))
        for j in range(self.dof):
            Jv[:, j] = np.cross(axes[j], p - orig[j])
            Jw[:, j] = axes[j]
        return T, Jv, Jw

    def clamp(self, q):
        return np.array([min(max(v, self.limits[i][0]), self.limits[i][1])
                         for i, v in enumerate(q)])


def pose_error(T, Tt):
    """(position error in metres, rotation vector of the orientation error)."""
    dp = Tt[:3, 3] - T[:3, 3]
    Re = Tt[:3, :3] @ T[:3, :3].T
    # rotation vector from the skew part; good enough while the error stays under ~90 deg
    w = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]]) * 0.5
    s = np.linalg.norm(w)
    if s > 1e-9:
        ang = math.atan2(s, max(-1.0, min(1.0, (Re[0, 0] + Re[1, 1] + Re[2, 2] - 1.0) * 0.5)))
        w = w / s * ang
    return dp, w


def quat_to_mat(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# ---------------------------------------------------------------- solving
def solve(chain, T_t, seed_deg, jaw=0.0):
    """Damped least squares with multi-start; returns (q_deg, iters, pos_m, ori_deg)."""
    best = None
    seeds = [np.radians(seed_deg)]
    seeds.append(np.radians(np.array([0.0, -20.0, 40.0, 0.0, 0.0][:chain.dof])))
    for d in (30.0, -30.0, 60.0, -60.0):
        s2 = np.array(seed_deg, dtype=float)[:chain.dof].copy()
        s2[1] = s2[1] + d
        seeds.append(np.radians(s2))
    for q0 in seeds:
        q = chain.clamp(np.array(q0, dtype=float))
        lam = DAMP
        for it in range(1, MAX_ITERS + 1):
            T, Jv, Jw = chain.fk_jac(q, jaw)
            dp, w = pose_error(T, T_t)
            if (np.linalg.norm(dp) * 1000.0 < TOL_MM / 4.0
                    and np.degrees(np.linalg.norm(w)) < TOL_DEG / 4.0):
                break
            err = np.concatenate([dp, w * ORI_WEIGHT])
            J = np.vstack([Jv, Jw * ORI_WEIGHT])
            step = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * np.eye(6), err)
            q_new = chain.clamp(q + step)
            # if a limit ate the step, push the damping up rather than spinning
            if np.allclose(q_new, q, atol=1e-9):
                lam = min(lam * 4.0, 1.0)
            else:
                lam = max(lam * 0.7, 1e-4)
            q = q_new
        T = chain.fk(q, jaw)
        dp, w = pose_error(T, T_t)
        cand = (float(np.linalg.norm(dp)), float(np.degrees(np.linalg.norm(w))), it, q)
        if best is None or (cand[0] + 0.001 * cand[1]) < (best[0] + 0.001 * best[1]):
            best = cand
        if best[0] * 1000.0 < TOL_MM and best[1] < TOL_DEG:
            break
    pos_m, ori_deg, iters, q = best
    return np.degrees(q), iters, pos_m, ori_deg


class IkNode(Node):
    def __init__(self):
        super().__init__("ik")
        self.chain = Chain()
        self.pub_sol = self.create_publisher(Float64MultiArray, "/ik/solution", 10)
        self.pub_status = self.create_publisher(String, "/ik/status", 10)
        self.pub_target = (self.create_publisher(Float64MultiArray, "/joint_targets", 10)
                           if APPLY else None)
        self.q_deg = None
        self.grip_deg = None
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(PoseStamped, "/ik/target", self.on_target, 10)
        self.get_logger().info(
            "ik: %d joints %s | limits %s | %s"
            % (self.chain.n, self.chain.links,
               ["%.0f..%.0f deg" % (math.degrees(a), math.degrees(b)) for a, b in self.chain.limits],
               "APPLY (publishes /joint_targets)" if APPLY else "report only"))

    def on_joints(self, msg):
        try:
            idx = [msg.name.index(j) for j in JOINTS]
        except ValueError:
            return
        self.q_deg = np.array([msg.position[i] for i in idx], dtype=float)
        try:
            self.grip_deg = float(msg.position[msg.name.index("gripper")])
        except (ValueError, IndexError):
            pass

    def on_target(self, msg):
        T_t = np.eye(4)
        T_t[:3, :3] = quat_to_mat(msg.pose.orientation.x, msg.pose.orientation.y,
                                  msg.pose.orientation.z, msg.pose.orientation.w)
        T_t[:3, 3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        if msg.header.frame_id and msg.header.frame_id not in ("base", ""):
            self._status("rejected: frame_id is %r, expected 'base' (no extrinsics are applied "
                         "here)" % msg.header.frame_id, ok=False)
            return
        seed = self.q_deg if self.q_deg is not None else np.zeros(self.chain.dof)
        q_deg, iters, pos_m, ori_deg = solve(self.chain, T_t, seed)
        ok = (pos_m * 1000.0 <= TOL_MM) and (ori_deg <= TOL_DEG)
        out = Float64MultiArray()
        out.data = [float(v) for v in q_deg]
        self.pub_sol.publish(out)
        self._status("ok=%s pos_err=%.1f mm ori_err=%.1f deg iters=%d q_deg=%s"
                     % (ok, pos_m * 1000.0, ori_deg, iters,
                        "[" + ", ".join("%.1f" % v for v in q_deg) + "]"), ok=ok)
        if self.pub_target is not None and ok:
            tgt = Float64MultiArray()
            grip = self.grip_deg if self.grip_deg is not None else 50.0
            tgt.data = [float(v) for v in q_deg] + [grip]
            self.pub_target.publish(tgt)

    def _status(self, text, ok):
        m = String()
        m.data = text
        self.pub_status.publish(m)
        (self.get_logger().info if ok else self.get_logger().warn)("ik: " + text)


def main():
    rclpy.init()
    node = IkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

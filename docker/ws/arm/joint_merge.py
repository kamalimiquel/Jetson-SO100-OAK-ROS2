#!/usr/bin/env python3
"""Merge both arms' joint states into the single topic the cell URDF needs.

Arm 1 publishes /arm1/joint_states_rad and arm 2 /arm2/joint_states_rad, with the joint names
prefixed `arm1_` / `arm2_` (deg2rad.py does that when ARM_NS is set — both arms run with one since
ADR-0023). One robot_state_publisher reads the merged topic and drives the whole two-arm tree
(`so101_cell.urdf`), because RViz on this board only draws the first RobotModel display — two
separate models cannot both appear.

Two things this file used to get wrong, both of the kind that has no error anywhere:

  * **The stamp was the last arrival's.** `out.header.stamp = msg.header.stamp` meant a dead arm
    kept publishing its frozen pose under a fresh stamp: the merged state was not stale, it was
    *disguised as current*, and RViz drew a dead arm as if it were live. Now the merged message
    carries the **oldest** live arm's stamp — it is only ever as fresh as its slowest member, and
    an age computed from it is honest.
  * **Nothing was dropped.** An arm that stops publishing now loses its joints after MERGE_STALE_S
    (0.5 s default) and is named once in the log, so nothing downstream is told a frozen pose is
    current. Measured on the board: tf2 keeps answering `lookup_transform(..., Time())` with the last
    pose indefinitely (it only prunes when new data arrives) and RViz keeps drawing it, so the visible
    change is the *stamp* stopping, not the model vanishing — say that, do not promise a disappearing
    arm. When the arm comes back it is named again.

`merge()` is a pure function of (latest messages, monotonic arrival times, now) so the rule is
pinned headless by `tools/test_joint_merge.py` — no ROS, no arms.
"""

import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

TOPIC = "/cell/joint_states_rad"
SOURCES = ["/arm1/joint_states_rad", "/arm2/joint_states_rad"]
# How long an arm may be silent before its joints are pulled out of the merged message. 0.5 s is
# ~12 missed samples at arm 1's bench rate of 24-27 Hz: long enough not to fire on a hiccup, short
# enough that nobody reads a stopped arm as a standing one.
STALE_S = float(os.environ.get("MERGE_STALE_S", "0.5"))


def merge(order, latest, arrivals, now, stale_s=STALE_S):
    """Build the merged JointState fields from what has arrived.

    order    — sources in the order they first spoke (the message's name order stays stable)
    latest   — {source: JointState}
    arrivals — {source: monotonic seconds at arrival}
    now      — monotonic seconds now

    Returns (names, positions, velocities, efforts, stamp, frame_id, live, dropped):
    `live` and `dropped` are the source lists the caller logs; `stamp` is the oldest live stamp.
    """
    live = [s for s in order if s in arrivals and (now - arrivals[s]) <= stale_s]
    dropped = [s for s in order if s not in live]
    names, position, velocity, effort = [], [], [], []
    stamp, frame_id = None, ""
    for src in live:
        msg = latest[src]
        names.extend(msg.name)
        position.extend(msg.position)
        velocity.extend(msg.velocity)
        effort.extend(msg.effort)
        if msg.header.frame_id:
            frame_id = msg.header.frame_id
        s = msg.header.stamp
        if stamp is None or (s.sec, s.nanosec) < (stamp.sec, stamp.nanosec):
            stamp = s
    return names, position, velocity, effort, stamp, frame_id, live, dropped


class JointMerge(Node):
    def __init__(self):
        super().__init__("joint_merge")
        self.pub = self.create_publisher(JointState, TOPIC, 10)
        self.latest = {}
        self.arrivals = {}
        self.order = []
        self.dropped = []          # what the last message left out, so a change is logged once
        for src in SOURCES:
            self.create_subscription(JointState, src, lambda m, s=src: self.on_msg(s, m), 10)
        self.get_logger().info("merging %s -> %s (an arm silent %.1fs is dropped)"
                               % (", ".join(SOURCES), TOPIC, STALE_S))

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_msg(self, src, msg):
        if src not in self.latest:
            self.order.append(src)
        self.latest[src] = msg
        self.arrivals[src] = self.now()

        names, position, velocity, effort, stamp, frame_id, live, dropped = merge(
            self.order, self.latest, self.arrivals, self.now())
        for src_gone in dropped:
            if src_gone not in self.dropped:
                self.get_logger().warn(
                    "%s silent for more than %.1fs — dropping its joints from %s. The merged state no "
                    "longer claims that arm; a TF/RViz consumer keeps its last drawn pose until its own "
                    "buffer expires, so check the stamp, not the picture"
                    % (src_gone, STALE_S, TOPIC))
        for src_back in self.dropped:
            if src_back not in dropped:
                self.get_logger().info("%s is publishing again — back in %s" % (src_back, TOPIC))
        self.dropped = dropped
        if not names:
            return
        out = JointState()
        if stamp is not None:
            out.header.stamp = stamp       # oldest live arm: the merged state is as fresh as that
        out.header.frame_id = frame_id
        out.name = names
        out.position = position
        out.velocity = velocity
        out.effort = effort
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

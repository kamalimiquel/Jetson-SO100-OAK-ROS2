#!/usr/bin/env python3
"""Publish a prefixed copy of the arm URDF for RViz's second RobotModel.

Why this exists: the second arm's TF tree is published with `frame_prefix: arm2_` by its
robot_state_publisher, while the URDF it loads keeps the plain link names. RViz's own "TF Prefix"
field is meant to bridge that, but on this build the display never leaves its error state with it
(red icon, nothing drawn), and `Description Source: File` does not load here either — only a
latched topic works. So this node publishes the same URDF with every link/joint name already
prefixed, on its own topic, and the display needs no TF Prefix at all.

    MODEL_URDF=/opt/ws/arm/urdf/so101.urdf MODEL_PREFIX=arm2_ \
    MODEL_TOPIC=/arm2/model_description python3 model_description_pub.py
"""

import os
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

URDF = os.environ.get("MODEL_URDF", "/opt/ws/arm/urdf/so101.urdf")
# RViz keys its mesh cache by resource path: two RobotModels pointing at the SAME .obj files end up
# with the second one rendering nothing (silently, icon still green). Rewriting the mesh directory in
# the published copy gives this model its own paths (a symlink to the same files on the server).
MESH_FROM = os.environ.get("MODEL_MESH_FROM", "")
MESH_TO = os.environ.get("MODEL_MESH_TO", "")
PREFIX = os.environ.get("MODEL_PREFIX", "arm2_")
TOPIC = os.environ.get("MODEL_TOPIC", "/arm2/model_description")


def prefixed_urdf(path, prefix):
    tree = ET.parse(path)
    root = tree.getroot()
    for tag in ("link", "joint"):
        for el in root.findall(tag):
            name = el.get("name")
            if prefix and name and not name.startswith(prefix):
                el.set("name", prefix + name)
    for joint in root.findall("joint"):
        for tag in ("parent", "child"):
            el = joint.find(tag)
            link = el.get("link") if el is not None else None
            if link and not link.startswith(prefix):
                el.set("link", prefix + link)
    if MESH_FROM and MESH_TO:
        for mesh in root.iter("mesh"):
            fn = mesh.get("filename")
            if fn:
                mesh.set("filename", fn.replace(MESH_FROM, MESH_TO))
    if prefix:
        root.set("name", prefix.rstrip("_") + "_so101")
    return ET.tostring(root, encoding="unicode")


def main():
    if not os.path.exists(URDF):
        print("model_description_pub: %s not found, exiting" % URDF, flush=True)
        return 1
    xml = prefixed_urdf(URDF, PREFIX)

    rclpy.init()
    node = rclpy.create_node("model_description_pub")
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(String, TOPIC, qos)
    msg = String()
    msg.data = xml
    # Latched: publish a few times so a subscriber that appears during discovery still gets it,
    # then stay alive so the transient-local sample is not dropped (the publisher owns it).
    for _ in range(3):
        pub.publish(msg)
        time.sleep(1.0)
    print("model_description_pub: %d bytes on %s (prefix %r)" % (len(xml), TOPIC, PREFIX), flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

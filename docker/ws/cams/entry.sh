#!/bin/bash
# cams container entrypoint: single-process depthai -> ROS2 node (py3.8).
set -e
python3 /opt/ws/cams/oak_ros_node.py

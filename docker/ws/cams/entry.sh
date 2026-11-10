#!/bin/bash
# cams container entrypoint: single-process depthai -> ROS2 node (py3.8),
# plus the browser view of the scene camera on :8090 (framing checks, aiming).
set -e
python3 /opt/ws/cams/oak_views.py &
python3 /opt/ws/cams/cloud_view.py &
python3 /opt/ws/cams/mjpeg_view.py &
python3 /opt/ws/cams/oak_ros_node.py

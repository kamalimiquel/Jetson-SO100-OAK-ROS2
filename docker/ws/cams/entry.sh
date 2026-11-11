#!/bin/bash
# cams container entrypoint: single-process depthai -> ROS2 node (py3.8),
# plus the browser view of the scene camera on :8090 (framing checks, aiming).
set -e
python3 /opt/ws/cams/oak_views.py &
python3 /opt/ws/cams/cloud_view.py &
python3 /opt/ws/cams/mjpeg_view.py &
# wrist camera (ADR-0022): a plain UVC webcam. Waits politely if the device is absent, so the
# rest of the container is unaffected on a bench without it.
python3 /opt/ws/cams/wrist_cam_node.py &
# Focus test for that camera (ADR-0022), :8093 — idle until the tab asks for it (/start).
python3 /opt/ws/cams/oak_ros_node.py

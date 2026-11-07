#!/bin/bash
# arm container entrypoint (image entrypoint already sourced the ROS env):
#  1. LeRobot joint server (py3.12 venv, serial bus, loopback HTTP :8002)
#  2. ROS2 arm bridge (py3.8 + rclpy)  -> /joint_states, /joint_targets, /robot_description
#  3. robot_state_publisher            -> TF for Foxglove from the URDF + joint states
set -e

/home/maju/lerobot/.venv/bin/python /opt/ws/arm/lerobot_joint_server.py &
sleep 2
python3 /opt/ws/arm/arm_bridge.py &
ros2 run robot_state_publisher robot_state_publisher &

wait

#!/bin/bash
# arm container entrypoint (image entrypoint already sourced the ROS env):
#  1. LeRobot joint server (py3.12 venv, serial bus, loopback HTTP :8002)
#  2. ROS2 arm bridge (py3.8 + rclpy)  -> /joint_states, /joint_targets, /robot_description
#  3. robot_state_publisher            -> TF for Foxglove from the URDF + joint states
set -e

# lerobot looks for its calibration under $HOME/.cache/huggingface — the
# container runs as root, so point HOME at the mount (host maju home).
export HOME=/home/maju

/home/maju/lerobot/.venv/bin/python /opt/ws/arm/lerobot_joint_server.py &
sleep 2
python3 /opt/ws/arm/arm_bridge.py &

# robot_state_publisher reads the robot_description PARAMETER (not the topic)
# in Humble — pass the URDF directly, with a retry for the boot race.
URDF=$(cat /opt/ws/arm/urdf/so100.urdf)
(until ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$URDF"; do echo "rsp: retrying in 2s"; sleep 2; done) &

wait

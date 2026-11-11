#!/bin/bash
# arm container entrypoint (image entrypoint already sourced the ROS env):
#  1. LeRobot joint server (py3.12 venv, serial bus, loopback HTTP :8002)
#  2. torque switch for the web page (LAN :8092 -> loopback :8002)
#  3. ROS2 arm bridge (py3.8 + rclpy)  -> /joint_states, /joint_targets, /robot_description
#  4. robot_state_publisher            -> TF for RViz from the URDF + joint states
set -e

# lerobot looks for its calibration under $HOME/.cache/huggingface — the
# container runs as root, so point HOME at the mount (host maju home).
export HOME=/home/maju

/home/maju/lerobot/.venv/bin/python /opt/ws/arm/lerobot_joint_server.py &
sleep 2
python3 /opt/ws/arm/torque_api.py &
python3 /opt/ws/arm/arm_bridge.py &

# robot_state_publisher reads the robot_description PARAMETER (not the topic)
# in Humble — pass the URDF directly. Re-read it on every retry: a container
# started while the file is being synced must not cache an empty string.
# Joint values are in DEGREES (LeRobot); RSP needs RADIANS (REP-103), so it
# consumes the converted topic from deg2rad.py.
python3 /opt/ws/arm/deg2rad.py &
(until URDF=$(cat /opt/ws/arm/urdf/so101.urdf) && [ -n "$URDF" ] \
     && ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$URDF" -r joint_states:=/joint_states_rad; do
   echo "rsp: retrying in 2s"; sleep 2
 done) &

# 5. Wrist camera frame (ADR-0022) — a STATIC transform, published once on /tf_static.
#    The webcam is clipped to the moving jaw, so its pose relative to that link never changes:
#    x/y/z are metres from the jaw link origin to the lens centre, and the rpy turns the link's
#    axes into the optical convention (z forward out of the lens, x right, y down the image).
#    These are the NOMINAL values of the printed bracket. Measure yours and override them from
#    docker-compose (WRIST_TF_X … WRIST_TF_YAW); the procedure is in README §10.5.
ros2 run tf2_ros static_transform_publisher \
  --x "${WRIST_TF_X:-0.0}" --y "${WRIST_TF_Y:-0.028}" --z "${WRIST_TF_Z:-0.035}" \
  --roll "${WRIST_TF_ROLL:-3.14159}" --pitch "${WRIST_TF_PITCH:-0.0}" --yaw "${WRIST_TF_YAW:-0.0}" \
  --frame-id moving_jaw_so101_v1 --child-frame-id gripper_color_optical_frame &

wait

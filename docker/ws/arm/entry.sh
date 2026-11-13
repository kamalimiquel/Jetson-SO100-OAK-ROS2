#!/bin/bash
# arm container entrypoint (image entrypoint already sourced the ROS env):
#  1. LeRobot joint server (py3.12 venv, serial bus, loopback HTTP :8002 / :8003)
#  2. torque switch for the web page (LAN :8092 / :8095 -> loopback)
#  3. ROS2 arm bridge (py3.8 + rclpy)  -> /joint_states, /joint_targets, /robot_description
#  4. robot_state_publisher            -> TF for RViz from the URDF + joint states
#  5. wrist camera static transform
#
# Two-arm (Phase 1): ARM_NS non-empty (e.g. "arm2") renames the topics via the
# bridge/deg2rad (they read it themselves) and prefixes this container's TF with
# ${NS}_ (frame_prefix + the wrist frames), so the two trees coexist in one graph.
set -e

# lerobot looks for its calibration under $HOME/.cache/huggingface — the
# container runs as root, so point HOME at the mount (host maju home).
export HOME=/home/maju

NS="${ARM_NS:-}"
FP="${NS:+${NS}_}"          # frame prefix: "" for arm 1, "arm2_" for arm 2
JSR="${NS:+${NS}/}joint_states_rad"   # RSP consumes the radians topic of THIS arm

/home/maju/lerobot/.venv/bin/python /opt/ws/arm/lerobot_joint_server.py &
sleep 2
# arm_bridge and deg2rad read the port/namespace from their own env (inherited).
python3 /opt/ws/arm/torque_api.py &
python3 /opt/ws/arm/arm_bridge.py &
python3 /opt/ws/arm/deg2rad.py &

# robot_state_publisher reads the robot_description PARAMETER (not the topic)
# in Humble — pass the URDF directly. Joint values are in DEGREES (LeRobot); RSP
# needs RADIANS (REP-103), so it consumes the converted topic from deg2rad.py.
# frame_prefix (only when ARM_NS is set, i.e. the second arm) keeps that arm's
# TF tree separate. An empty prefix would be an invalid -p frame_prefix:= argument.
(
  while true; do
    URDF=$(cat /opt/ws/arm/urdf/so101.urdf 2>/dev/null) || { echo "rsp: no urdf, retry in 2s"; sleep 2; continue; }
    [ -n "$URDF" ] || { echo "rsp: empty urdf, retry in 2s"; sleep 2; continue; }
    RSP_ARGS=(-p "robot_description:=$URDF" -r "joint_states:=/$JSR")
    [ -n "$FP" ] && RSP_ARGS+=(-p "frame_prefix:=$FP")
    ros2 run robot_state_publisher robot_state_publisher --ros-args "${RSP_ARGS[@]}" || true
    echo "rsp: retrying in 2s"; sleep 2
  done
) &

# Wrist camera frame (ADR-0022) — a STATIC transform, published once on /tf_static.
# The webcam is clipped to the moving jaw, so its pose relative to that link never changes:
# x/y/z are metres from the jaw link origin to the lens centre, and the rpy turns the link's
# axes into the optical convention (z forward out of the lens, x right, y down the image).
# These are the NOMINAL values of the printed bracket. Measure yours and override them from
# docker-compose (WRIST_TF_X … WRIST_TF_YAW); the procedure is in README §10.5.
# A second arm prefixes both frame ids (${FP}moving_jaw… -> ${FP}gripper…), Phase 1.
ros2 run tf2_ros static_transform_publisher \
  --x "${WRIST_TF_X:-0.0}" --y "${WRIST_TF_Y:-0.028}" --z "${WRIST_TF_Z:-0.035}" \
  --roll "${WRIST_TF_ROLL:-3.14159}" --pitch "${WRIST_TF_PITCH:-0.0}" --yaw "${WRIST_TF_YAW:-0.0}" \
  --frame-id "${FP}moving_jaw_so101_v1" --child-frame-id "${FP}gripper_color_optical_frame" &

# Where the second arm stands relative to arm 1's base (Phase 1). The 30 cm of
# requirement 2 lives ONLY here (README §3.6 R3): one static transform
# base -> "${NS}_base" at (0, 0.30, 0); the MuJoCo twin must read the same number.
if [ -n "$NS" ]; then
  ros2 run tf2_ros static_transform_publisher \
    --x "${ARM2_OFFSET_X:-0.0}" --y "${ARM2_OFFSET_Y:-0.30}" --z "${ARM2_OFFSET_Z:-0.0}" \
    --frame-id base --child-frame-id "${NS}_base" &
fi

wait

# ADR-0006: SO-100 arm driver

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (alternative 2 of 3)

## Context

The SO-100 arm needs a driver that serves telemetry and commands to ROS2 and aligns with the project's learned-policy goals (Phase 4 episode recording, Phase 6b training on Windows). phosphobot is a mature third-party stack, but its Python bridge and HTTP commanding are a detour from ROS2; a ros2_control C++ driver is the long-term native option.

## Alternatives

1. **phosphobot + rclpy bridge** — Pros: mature and self-contained, web/gamepad teleop included, per-arm calibration. Cons: Python bridge, HTTP commanding, no ros2_control.
2. **LeRobot (SO-ARM100) firmware + SDK** — Pros: the training ecosystem (datasets, MuJoCo sim, policies) matches the project's VLM/learned-policy goals. Cons: requires reflashing the bus adapter firmware; Python SDK; teleop and calibration go through LeRobot's own procedures.
3. **Custom C++ driver (Feetech SCS protocol) + ros2_control hardware_interface** — Pros: pure C++, real-time control loop, MoveIt-compatible. Cons: the most work; re-implements calibration, torque handling and teleop glue.

## Decision

**LeRobot (SO-ARM100) firmware + SDK.** The arm's bus adapter is flashed with the official LeRobot feetech firmware and driven with the `lerobot` Python SDK; a custom rclpy node bridges it to ROS2.

## Consequences

- The bus adapter must be flashed with the official LeRobot firmware — a one-time hardware step on the stock arm.
- Teleop and calibration use LeRobot's procedures: calibration is stored per arm; teleop comes from Phase 6c (F710) and Phase 7 (web UI).
- The SDK needs Python ≥3.10 on the Jetson (uv-managed venv, py3.12); serial access via udev (VID `1a86`, ttyACM*), group `dialout`.
- Recorded episodes go straight to LeRobot dataset format, and the Windows PC can train policies on LeRobot's SO-100 MuJoCo sim without conversion glue.
- The ROS2 bridge (rclpy) publishes joint states and exposes a joint-command interface; a swap to a ros2_control C++ driver later is the escape hatch if MoveIt becomes necessary.

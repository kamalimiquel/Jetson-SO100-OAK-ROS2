# ADR-0006: SO-100 arm driver

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (alternative 2 of 3)

## Context

The SO-100 arm needs a driver that serves telemetry and commands to ROS2 and aligns with the project's learned-policy goals (Phase 4 episode recording, Phase 6b training on Windows). phosphobot is a mature third-party stack, but its Python bridge and HTTP commanding are a detour from ROS2; a ros2_control C++ driver is the long-term native option.

## Alternatives

1. **phosphobot + rclpy bridge** — Pros: mature and self-contained, web/gamepad teleop included, per-arm calibration. Cons: Python bridge, HTTP commanding, no ros2_control.
2. **LeRobot (SO-ARM100) SDK** — Pros: the training ecosystem (datasets, MuJoCo sim, policies) matches the project's VLM/learned-policy goals. Cons: Python SDK; teleop and calibration go through LeRobot's own procedures; requires a one-time motor setup (IDs + baudrate in servo EEPROM).
3. **Custom C++ driver (Feetech SCS protocol) + ros2_control hardware_interface** — Pros: pure C++, real-time control loop, MoveIt-compatible. Cons: the most work; re-implements calibration, torque handling and teleop glue.

## Decision

**LeRobot (SO-ARM100) SDK.** The arm is driven with the `lerobot` Python SDK over the stock Feetech serial bus — the CH9102 USB-UART adapter carries no firmware, so there is nothing to flash; a custom rclpy node bridges it to ROS2. A one-time motor setup (`lerobot-setup-motors`: unique IDs 1–6 and bus baudrate written to each servo's EEPROM) precedes first use, followed by LeRobot's per-arm calibration.

## Consequences

- One-time setup step: motor IDs (1–6) and bus baudrate written to the servos' EEPROM via `lerobot-setup-motors`; the CH9102 adapter itself carries no firmware. *Amended 2026-11-07 after checking current LeRobot docs: no flashing step exists for the SO-100.*
- Teleop and calibration use LeRobot's procedures: calibration is stored per arm; teleop comes from Phase 6c (F710) and Phase 7 (web UI).
- The SDK needs Python ≥3.10 on the Jetson (uv-managed venv, py3.12); serial access via udev (VID `1a86`, ttyACM*), group `dialout`.
- Recorded episodes go straight to LeRobot dataset format, and the Windows PC can train policies on LeRobot's SO-100 MuJoCo sim without conversion glue.
- The ROS2 bridge (rclpy) publishes joint states and exposes a joint-command interface; a swap to a ros2_control C++ driver later is the escape hatch if MoveIt becomes necessary.

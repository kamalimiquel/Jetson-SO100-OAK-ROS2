# ADR-0017 amendment: F710 teleop over rosbridge (Python client)

- Status: Accepted (amends ADR-0017)
- Date: 2026-11-09
- Decided by: user ("continuamos con la implementación del f710")

## Context

ADR-0017 chose a **native Windows C++ client** (rclcpp + SDL2 + DDS) for the F710. The code exists (`windows/f710_teleop.cpp`) but has never been compiled, and compiling it requires the full ROS2 Humble Windows install, Visual Studio Build Tools and an SDL2 dev tree — a large surface for a teleop pad, with the extra risk of a FastDDS version skew between the Jetson and Windows.

Since then the cell gained a **rosbridge WebSocket** (ADR-0019) already proven for the MuJoCo twin, and the transport test on 2026-11-09 showed publishing `/joint_targets` through it moves nothing when the values equal the current pose (max delta 0.0°).

## Alternatives

1. **Finish the C++ client (ADR-0017 as written)** — full ROS2 Windows stack. Pros: first-class ROS node, low latency. Cons: heavy toolchain, never validated, DDS skew risk.
2. **Python client over rosbridge** (`pygame` + `roslibpy`) — Pros: two pip packages, same transport as the twin, ~200 lines, easy to tweak; no ROS2 on Windows. Cons: JSON/WebSocket hop adds ~2–5 ms per message (irrelevant at 50 Hz teleop).
3. **Python client with native DDS on Windows** (`pip install rclpy` on Windows) — same weight as option 1 with none of its benefit.

## Decision

**Option 2**: `windows/f710_teleop_ws.py`. It mirrors the C++ semantics — joint-step mode (D-pad selects joint, left stick moves it, LB/RB gripper), cartesian-feel mode (START toggles), record/replay (X / B) — and keeps the same safety rules: nothing is published until `/joint_states` arrives, targets are seeded at the arm's real pose, deltas and limits are clamped, and publishing stops when the pad is idle.

`windows/f710_teleop.cpp` stays in the tree as the reference implementation and as the path to option 1 if latency ever matters.

## Consequences

- Windows needs only `pip install pygame roslibpy`; the pad must be in **X mode** (switch on the back).
- `/joint_targets` carries degrees (LeRobot convention), limits `-170…170` per joint, gripper `0…100`.
- The Jetson bridge still enforces the real limits — the client can never push the arm past them.
- Verified end-to-end 2026-11-09: rosbridge → `/joint_targets` → arm bridge accepted a 6-value target vector; sending the current pose produced no motion.

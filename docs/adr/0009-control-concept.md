# ADR-0009: Arm control concept (VLM-oriented)

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (alternative 1 of 3)

## Context

The control concept determines the interface a future VLM (and GraspGen) uses to command the arm. GraspGen and VLM pipelines emit 6-DoF poses in SE(3); the LeRobot SDK (ADR-0006) commands joint angles natively.

## Alternatives

1. **Joint-space API** — the system/model outputs 6 joint angles + jaw; direct to the LeRobot SDK. Pros: simplest, no IK machinery on the Jetson, natural fit with LeRobot episode recording (joint states). Cons: any model that reasons in Cartesian space (VLM, GraspGen) needs a pose→joints conversion somewhere.
2. **Cartesian 6-DoF API** — `move_to_pose` actions with system-side IK and calibrated TF. Pros: the natural interface for VLM/GraspGen (they emit SE(3) poses). Cons: IK/calibration machinery on the Jetson.
3. **Skill library on top of Cartesian** — the model picks a skill + waypoint. Pros: most robust. Cons: most engineering; constrains the model to the skills written.

## Decision

**Joint-space API**: the arm exposes joint commands (6 angles + jaw) and publishes joint state; an rclpy bridge translates to the LeRobot SDK. No IK/planning stack on the Jetson.

## Consequences

- A pose→joint-angle translation layer (IK) **will be required** before any VLM/GraspGen integration: both emit SE(3) poses. That converter can live on the Windows PC (where GraspGen/VLM run) or become a later Jetson node — flagged for the Phase 4 implementation session.
- Episode recording stores joint angles — matches LeRobot's dataset format directly (a plus for imitation learning).
- The joint API remains the low-level layer if/when a Cartesian API is added later (superseding this ADR is possible without breaking it).
- Wrist-camera servoing will need incremental joint targets computed externally.
- Interface: ROS2 topic/action for joint commands + `/joint_states` feedback; MoveIt/ros2_control not required.

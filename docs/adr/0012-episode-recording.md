# ADR-0012: Episode recording for policy training

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (alternative 1 of 3)

## Context

Episodes (camera feeds + joint states + commands) must be captured for imitation learning on the Windows PC and must end in LeRobot dataset format. The arm is driven through the LeRobot SDK (ADR-0006) while the cameras publish via ROS2 (ADR-0007, ADR-0008) — two sources, one timeline needed.

## Alternatives

1. **rosbag2 on the Jetson + offline conversion to LeRobot format** — Pros: one capture path for everything, replayable for debugging, aligned timestamps. Cons: a conversion step before training.
2. **LeRobot native recorder** (`lerobot record`) — Pros: episodes land directly in LeRobot format. Cons: its camera stack does not support OAK-D (depthai), so the vision feeds would have to be spliced in anyway; no clean ROS2 replay.
3. **Both** — Pros: best of both worlds. Cons: two capture systems to keep aligned during every recording session.

## Decision

**rosbag2 + offline conversion**: record all synchronized topics on the Jetson; convert to LeRobot dataset format with an offline script.

## Consequences

- Recording pipeline: scene color/depth, wrist color/depth, `/joint_states`, joint commands — all in one bag.
- A conversion script (bag → LeRobot dataset, preserving episode structure and the SO-100 joint order) lives in the repo (`tools/`).
- rosbag2 recording runs in the control container; bags are transferred to the Windows PC for training.
- Episode boundaries are marked (service call / topic trigger) so the converter slices episodes cleanly.

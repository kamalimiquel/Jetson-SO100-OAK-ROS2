# ADR-0010: Camera mounting

- Status: Accepted — **amended 2026-11-11 by ADR-0022**: the eye-in-hand camera is a USB RGB webcam, not an OAK-D Lite
- Date: 2026-11-07
- Decided by: user (alternative 3 of 3)

## Context

Two depth cameras exist: the scene OAK-D and the wrist OAK-D Lite (ADR-0007, ADR-0008). Mounting determines what each camera sees, how calibration is set up, and which feed GraspGen consumes as primary input.

## Alternatives

1. **Eye-to-hand** (scene OAK-D fixed over the tabletop) — Pros: stable frame, full scene view, GraspGen's canonical setup. Cons: self-occlusion behind the arm; needs a rigid fixed mount.
2. **Eye-in-hand** (wrist Lite only) — Pros: close views, no fixed mount. Cons: narrow FOV, scene lost during motion — insufficient alone for scene-level grasping.
3. **Both** — fixed scene OAK-D as primary + wrist Lite for refinement and pre-grasp verification. Pros: each camera does what it's best at. Cons: two extrinsic calibrations to maintain.

## Decision

**Both**: scene OAK-D mounted fixed overlooking the workspace (eye-to-hand, primary perception source); OAK-D Lite on the gripper (eye-in-hand, refinement and pre-grasp verification).

## Consequences

- Two extrinsic calibrations: `scene_cam → arm_base` and `wrist_cam → arm_base`. The wrist one varies with the arm's pose and is published through TF from the wrist link.
- GraspGen consumes the scene point cloud as primary input; wrist depth crops are used for close-range refinement.
- The scene mount must be rigid and repeatable; if it ever moves, recalibrate.
- RViz shows both camera frames in the TF tree.

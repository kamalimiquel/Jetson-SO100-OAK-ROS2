# ADR-0011: Extrinsic calibration method

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (alternative 1 of 3)

## Context

Both cameras need extrinsics to the arm (ADR-0010): `scene_cam → arm_base` (fixed) and `wrist_cam → arm_base` (joint-dependent, published via TF). GraspGen needs the scene camera's frame mapped into the arm's frame to execute its grasp proposals.

## Alternatives

1. **easy_handeye + ChArUco board** — the standard ROS2 package; solves eye-to-hand and eye-in-hand calibrations from a set of sample poses. Pros: proven, repeatable procedure. Cons: one package to build; the board must be held or attached during calibration.
2. **Manual calibration** — known poses, hand measurement, a solvePnP script. Pros: zero extra dependencies. Cons: slow, error-prone, painful to redo.
3. **Custom AprilTag solver** — an in-house implementation in the spirit of easy_handeye. Pros: full control. Cons: reinvents what easy_handeye already does well.

## Decision

**easy_handeye + ChArUco board** for both cameras.

## Consequences

- A ChArUco board must be printed (standard sizes work at tabletop distance); calibration runs follow easy_handeye's procedure (eye-to-hand for the scene camera, eye-in-hand for the wrist camera).
- The resulting transforms feed the TF tree and the GraspGen pose conversion (scene frame → arm base).
- easy_handeye runs in the perception/control container during calibration sessions only — not a runtime dependency.
- Recalibration is required whenever the scene mount moves (ADR-0010).

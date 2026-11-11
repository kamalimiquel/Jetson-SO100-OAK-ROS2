# ADR-0011: Extrinsic calibration method

- Status: Accepted — **amended 2026-11-09**: solved with OpenCV, not easy_handeye; **amended 2026-11-11 by ADR-0022**: the wrist camera is a USB RGB webcam
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

## Amendment 2026-11-09 — solver: OpenCV in the container, not easy_handeye

easy_handeye was dropped as the *solver*, not as the *method*: the procedure (ChArUco
board on the gripper, N poses, solve AX = XB) stands; only the software that solves it changes.

**Why.** easy_handeye for Humble pulls `aruco_ros` plus RViz/Qt plugins, and none of those
exist as debs for the stack's real runtime image (focal / Python 3.8, dustynv ROS). Building
them means a second from-source RViz tree — hours of compile on a 4-core NX for a calibration
that runs twice in the project's life. OpenCV is already in the perception container and ships
`cv2.calibrateHandEye`, which solves exactly the same AX = XB with the same input (per-pose
gripper pose + camera-to-board pose).

**What replaces each piece of easy_handeye**

| easy_handeye piece | Replacement |
|---|---|
| pose sampling GUI | script: step `/joint_targets` through ~20 poses, wait for settle, capture |
| `robot_state_publisher` / TF lookup | `tf2` `base` → `jaw`, or FK straight from the URDF |
| `aruco_ros` detection | `cv2.aruco.CharucoDetector` + `solvePnP` on the detected corners |
| `calibrateHandEye` service | `cv2.calibrateHandEye` (eye-to-hand), result inverted to `base ← scene_cam` |
| publishing the result as TF | static transform published by the perception container / stored in YAML |

**One addition this forced.** The scene OAK has no factory intrinsics available in the current
depthai build, and hand-eye needs a real camera matrix — so the same image set also feeds
`cv2.calibrateCamera` (ChArUco corners from the same ~20 views). Both results —

- `scene_cam` intrinsics → `docs/calibration/scene_cam_intrinsics.yaml`
- `base ← scene_cam` extrinsic → `docs/calibration/scene_cam_extrinsic.yaml`

— are committed, so the transform survives reflashes and container rebuilds, and the calibration
can be re-derived from the stored images without a new session.

**Scope of this amendment.** Scene OAK-D only (the user's call, 2026-11-09). The wrist OAK-D Lite
is calibrated with the same script when it is physically mounted (eye-in-hand: the same solver,
the board fixed in the workspace instead of on the gripper).

**Board.** `docs/charuco-board.pdf` — 5×5 squares, DICT_4X4_50, 18 mm squares / 13.5 mm markers
(90 × 90 mm), with a 100 mm scale bar to verify the printer did not rescale.

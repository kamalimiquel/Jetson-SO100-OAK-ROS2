# ADR-0008: Gripper camera — OAK-D Lite on the shared depthai node

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (after discussing GraspGen input requirements)

## Context

The gripper camera decision was re-opened in light of GraspGen: grasp generation consumes object point clouds, not raw video, so depth near the gripper is more valuable than a plain RGB wrist cam. The user chose a second OAK-D for the wrist.

## Alternatives

1. **OAK-D Lite (wrist) driven by the same custom depthai node** — Pros: one driver stack (ADR-0007), depth at the wrist for servoing and grasp refinement, no extra dependencies. Cons: USB3 bandwidth shared with the scene camera; must respect the Lite's minimum stereo range.
2. **Plain UVC USB camera + `usb_cam`** — Pros: lightest, zero risk, trivial driver. Cons: no depth at the wrist; a second, unrelated driver stack.
3. **Standard OAK-D at the wrist** — Pros: same as 1 with a longer baseline. Cons: minimum depth ~30 cm is above wrist working distance; ~100 g on a weak wrist servo.

## Decision

**OAK-D Lite wrist-mounted on the gripper, driven by the same custom depthai node**, namespaced `/oak/gripper` (scene camera: `/oak/scene`).

## Consequences

- Two depthai devices live in the `cams` container; the node instantiates one pipeline per device with per-device namespaces and camera_info.
- USB3 bandwidth budget on the NX: per-device resolution/FPS caps (e.g. scene 1080p/30, wrist 720p/30 or lower); both links must be verified as USB3 (see the USB2-link trap).
- Wrist depth must be validated against the Lite's minimum range for the actual working distance; per-device calibration.
- Weight check: Lite (~60 g) + bracket + cable on the SO-100 wrist servo.
- GraspGen: the scene point cloud remains the primary input; the wrist depth/color feed is used for refinement and pre-grasp verification (Phase 4).

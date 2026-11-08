# ADR-0019: MuJoCo twin transport — rosbridge WebSocket

- Status: Accepted
- Date: 2026-11-08
- Decided by: user (alternative 3 of 3, after a Zenoh discussion)

## Context

ADR-0016 puts MuJoCo (and later Isaac Sim) training on the Windows PC. The first milestone is the *digital twin*: MuJoCo mirrors the real arm live so training, replay and pose sanity-checks run against the same joint space as the hardware. The twin needs the arm's position stream (`/joint_states`, 50 Hz) on the Windows PC over the LAN. ROS2's own middleware is DDS over UDP, which Windows-only Python tools do not speak natively.

## Alternatives

1. **Native ROS2 on Windows**: install ROS2 Humble for Windows + rclpy and subscribe to DDS directly. Pros: no extra server; the training code is a first-class ROS node. Cons: ROS2 Windows is a big install (VS Build Tools, lots of packages) and version-skew between the Jetson's FastDDS build and the Windows one is a known source of silent discovery failures.
2. **HTTP polling**: poll the arm container's joint server (`GET /read` on port 8002) over the LAN. Pros: trivially simple; already built. Cons: polling at 50 Hz over HTTP is latency-noisy; it bypasses ROS2 (future cameras/forces would need another channel); the port would need LAN exposure.
3. **rosbridge WebSocket on the Jetson**: a rosbridge server subscribes to any ROS2 topic and forwards it as JSON over a single WebSocket; the Windows side uses the small `roslibpy` client. Pros: one well-known protocol, tiny Windows footprint (pure pip), any topic becomes available later (images, forces, actions) with zero new plumbing. Cons: one more service on the Xavier.

Zenoh was considered as a fourth option and rejected: correct idea, but DDS already carries the data and adding a second middleware buys nothing at this scale.

## Decision

**rosbridge WebSocket.** A `so100-rosbridge` container runs `rosbridge_server` on port 9090 (ws://). The Windows twin (`windows/mujoco_twin.py`) connects with `roslibpy`, subscribes to `/joint_states`, and mirrors it into MuJoCo as a kinematic mirror.

Implementation notes recorded because each cost debugging time:

- **Standalone jammy (Ubuntu 22.04) image.** rosbridge_suite for Humble requires Python >=3.9; the cell images are focal (Ubuntu 20.04) with Python 3.8, so rosbridge cannot run inside them. The official `arm64v8/ros:humble-ros-base` image carries rosbridge as ready binary packages (`ros-humble-rosbridge-suite`). Discovery and data across the two FastDDS versions work once `ipc: host` is set (below).
- **`ipc: host` is mandatory.** FastDDS shared-memory transport carries the data plane over `/dev/shm`; without the host IPC namespace the container discovers topics but receives no data.
- **The twin is kinematic, not dynamic.** MuJoCo `viewer.launch_passive` hands control back to the script every frame; the script writes qpos from `/joint_states` and calls `mj_forward` + `sync` — physics is never stepped, so nothing fights the real arm.

## Consequences

- Port 9090 (WebSocket) on the Xavier, domain 42, host network + `ipc: host`.
- Windows needs only `pip install roslibpy "mujoco>=3,<4"` — no ROS2 Windows install for the twin.
- Future consumers (grasp requests, images) ride the same WebSocket.
- MuJoCo 4.x is excluded for now: it moved the classic viewer into MuJoCo Studio, and the twin pins the 3.x line.

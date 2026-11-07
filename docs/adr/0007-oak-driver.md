# ADR-0007: OAK-D POE driver

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (alternative 2 of 3, node implemented in Python)

## Context

The OAK-D POE (USB-C) provides color + stereo depth for the perception pipeline. The C++ ROS2 driver ecosystem is blocked today: the depthai 2.x pip wheel no longer ships headers or `depthaiConfig.cmake`, so `depthai_ros_driver` cannot build against it. The project wants a purpose-built node with full control of the stereo pipeline; the user chose Python for the node itself.

## Alternatives

1. **rclpy bridge with the pinned `depthai<3` wheel** — Pros: works out of the box. Cons: hostage to the pinned wheel; no headers for future C++ work.
2. **`depthai-core` from source + custom node** — Pros: full control, up-to-date SDK, headers available (unblocks C++ nodes later). Cons: the arm64/L4T source build is the cost.
3. **UVC mode + `usb_cam`** — Pros: trivial. Cons: loses stereo depth entirely (plain RGB webcam).

## Decision

**`depthai-core` built from source + a custom node implemented in Python (rclpy).** The node is purpose-built for this cell (color, stereo depth, camera_info, pointcloud when needed), sitting on top of the source-built SDK.

## Consequences

- `depthai-core` is built on the Jetson (arm64, L4T R35.x) and the `depthai-python` bindings are compiled against that build — this also restores the headers and `depthaiConfig.cmake`, so `depthai_ros_driver` or a custom C++ node become possible later.
- The node itself is Python (rclpy). It supersedes ad-hoc bridge scripts; the pinned `depthai<3>` wheel remains the documented fallback.
- Fallback: if the source build becomes a time sink, the same node runs unchanged against the pinned `depthai<3` wheel — identical Python API.
- In Docker: mount `/dev/bus/usb` + cgroup rule `c 189:* rmw`, or the camera will not open. Verify the USB3 link before blaming the driver.
- Build cost on the NX: expect one long build (~30-60+ min); ensure swap/disk headroom, not RAM-only.

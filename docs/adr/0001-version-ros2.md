# ADR-0001: ROS2 version for the Xavier NX

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (alternative 1 of 3)

## Context

The cell is governed by a Jetson Xavier NX 16 GB with JetPack 5.1.5 (L4T R35.x, Ubuntu 20.04). The Xavier NX has no JetPack 6, so it cannot move off Ubuntu 20.04 without a reflash. The ROS2 version must also coexist with the GPU vision ecosystem (Isaac ROS) and with a Windows 11 PC on the same LAN.

## Alternatives

1. **Humble LTS in Docker containers** — base image `dustynv/ros:humble-ros-base-l4t-r35.x` (ROS built from source at `/opt/ros/humble/install/`).
   - Pros: current LTS (supported until 2027), Isaac ROS compatible, reproducible and isolated from the host.
   - Cons: no native ROS2 on the host; all tooling goes through containers.
2. **Foxy native** (Ubuntu 20.04 packages).
   - Pros: direct apt install.
   - Cons: EOL since May 2023 (no security patches), incompatible with current Isaac ROS.
3. **Humble built from source on the host** (colcon).
   - Pros: native, no containers.
   - Cons: fragile (glibc and L4T patches), pollutes the Jetson, costly maintenance.

## Decision

ROS2 **Humble LTS in Docker containers**, base image `dustynv/ros:humble-ros-base-l4t-r35.x`. R35.x images run on any L4T R35.x from JetPack 5.1+ without matching the exact patch level.

## Consequences

- All Xavier nodes live in containers; the host only carries Docker + nvidia-container-toolkit.
- Containers use `--network=host` so topics are visible across containers and machines without extra configuration.
- The source path in this image is `/opt/ros/humble/install/setup.bash` (there is no `/opt/ros/humble/setup.bash`).
- Any future GPU perception (Isaac ROS) starts from NVIDIA images compatible with L4T R35.x.

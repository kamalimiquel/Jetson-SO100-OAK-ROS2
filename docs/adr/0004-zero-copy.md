# ADR-0004: Zero-copy strategy

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (alternatives 1+2 of 3 — hybrid)

## Context

Camera frames (OAK-D color 1080p + stereo depth 720p, gripper camera) must travel through the perception pipeline without unnecessary RAM copies on a 16 GB Xavier NX. The chosen RMW (ADR-0002, FastDDS) already offers a shared-memory transport; the container topology (ADR-0003) places domains in separate processes/containers.

## Alternatives

1. **ROS2 intra-process communication** (Composition + LoanedMessages) — zero copies between nodes in the same process; ideal for the camera→perception pipeline. Cons: only within one process; nodes must be built as components.
2. **FastDDS shared-memory transport** — zero copies between processes on the same host via `/dev/shm`. Cons: requires a shared IPC namespace across containers and a large enough segment.
3. **CUDA zero-copy** (pinned memory, cudaMemcpy / Isaac ROS Nitros) — frames stay in GPU memory end to end. Cons: requires Isaac ROS images; pays off only with dense GPU perception.

## Decision

**Hybrid 1+2**: intra-process composition for the perception pipeline; FastDDS shared-memory transport between containers/processes on the Xavier; plain DDS for light telemetry and the Windows PC link. CUDA/Nitros zero-copy is revisited when GPU models enter the pipeline.

## Consequences

- Perception nodes must be built as rclcpp components so they can be composed into one process (LoanedMessages).
- FastDDS SHM requires a shared `/dev/shm` across the containers that exchange frames (Phase 2c) and a segment large enough for image payloads (raise the default 64 MB).
- Anything leaving the Xavier (the Windows PC link) goes over UDP DDS regardless of the SHM transport.
- Verification on the Xavier must confirm the SHM transport is actually negotiated for the frame topics (FastDDS logs / participant introspection).

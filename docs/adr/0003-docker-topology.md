# ADR-0003: Docker container topology

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (alternative 2 of 3)

## Context

The Xavier NX hosts the whole software stack: arm driver, two cameras, perception, control and a web UI. The container topology determines how domains are isolated, how they communicate, and how the zero-copy / shared-memory strategy (Phase 2b/2c) applies.

## Alternatives

1. **Monolithic container** — Pros: simplest, no inter-container IPC, no compose. Cons: all domains coupled; a camera change rebuilds everything; no per-domain restarts.
2. **One container per domain with docker compose** — e.g. `arm` (SO-100), `cams` (OAK-D + gripper), `perception`, `control`, `web` (RViz/rqt/SSH). Pros: per-domain isolation, independent restarts, natural fit for the shared IPC-namespace strategy. Cons: inter-domain communication must be designed carefully.
3. **One container per ROS2 node** — Pros: maximum granularity. Cons: disproportionate orchestration for ~10 nodes; configuration noise without benefit.

## Decision

**One container per domain, orchestrated with docker compose.**

## Consequences

- Tentative services: `arm`, `cams`, `perception`, `control`, `web` (names finalized when the compose file lands).
- Containers run with `--network=host` (ADR-0001): DDS traffic flows on the host network; FastDDS shared-memory transport applies where the IPC namespace is shared (Phase 2c).
- Only the camera domain mounts `/dev/bus/usb` + cgroup rule; the arm domain mounts the serial device; the web domain exposes the LAN ports.
- Each domain rebuilds and restarts independently.

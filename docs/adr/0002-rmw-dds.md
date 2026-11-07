# ADR-0002: DDS middleware (RMW) and domain

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (alternative 2 of 3)

## Context

The RMW conditions transport between nodes on the Xavier, between containers on the same host, and between the Xavier and the Windows 11 PC (Phase 6). It interacts with the zero-copy strategy (Phase 2): if the RMW offers shared-memory transport, part of the inter-process zero-copy comes for free.

## Alternatives

1. **CycloneDDS** — Pros: very light, excellent embedded performance, reliable multi-machine LAN discovery. Cons: no built-in inter-process shared-memory transport.
2. **FastDDS** (Humble default) — Pros: built-in shared-memory transport between processes on the same host, useful for inter-container zero-copy. Cons: heavier on the Jetson; pickier multicast discovery between machines.
3. **RTI Connext** — Pros: enterprise robustness, advanced QoS. Cons: commercial license, heavy configuration for this scale.

## Decision

**FastDDS** (`rmw_fastrtps_cpp`), Humble's default RMW. Cell-wide DDS domain: **`ROS_DOMAIN_ID=42`** (shared by Xavier and Windows).

## Consequences

- FastDDS shared-memory transport activates automatically between processes on the same host. To cross containers it needs a shared `/dev/shm` segment (IPC namespaces — Phase 2c); without it, FastDDS falls back to UDP.
- On the Jetson, FastDDS uses somewhat more RAM/CPU than CycloneDDS; keep an eye on it when profiling the image pipeline.
- Cross-machine discovery: if multicast fails between Xavier and Windows, configure static FastDDS peers (participant XML). To verify in Phase 6.
- Every machine in the cell uses `ROS_DOMAIN_ID=42`.

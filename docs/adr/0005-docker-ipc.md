# ADR-0005: Docker IPC namespaces

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (alternative 1 of 3)

## Context

FastDDS shared-memory transport (ADR-0002, ADR-0004) requires the containers exchanging frames to share a POSIX shared-memory segment (`/dev/shm`). By default every Docker container gets its own IPC namespace, so SHM cannot cross container or host boundaries.

## Alternatives

1. **`--ipc=host`** — the container shares the host's IPC namespace: SHM just works everywhere (container↔container, container↔host). Cons: every container sees the host's `/dev/shm` and POSIX IPC; per-domain isolation is reduced.
2. **`ipc: shareable` + `ipc: service:<name>`** — a private namespace shared only among the stack's containers; the host stays isolated. Cons: must track which service owns the namespace.
3. **`ipc: container:<name>`** — attach to one specific container's namespace. Cons: startup-order dependency on the donor container; less readable in compose.

## Decision

**`--ipc=host` on all stack containers.**

## Consequences

- FastDDS SHM works between all containers and between containers and host processes (e.g. a host-side phosphobot instance or a debug tool).
- With `--ipc=host` the container mounts the host's `/dev/shm` directly (default 50% of RAM on Ubuntu — several GB on this board), so the compose `shm_size` knob is not needed and would be ignored.
- Any process with access to the host's `/dev/shm` can see the segments — acceptable on this single-purpose LAN device; revisit only if the board starts hosting unrelated workloads.
- Future CUDA pinned-memory zero-copy is unaffected: it goes through the NVIDIA runtime, not the IPC namespace.
- The compose file will set `ipc: host` on the services that exchange frames.

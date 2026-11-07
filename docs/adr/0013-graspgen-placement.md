# ADR-0013: GraspGen deployment location

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (alternative 1 of 3)

## Context

GraspGen (NVlabs — diffusion 6-DoF grasp generation from object point clouds + segmentation masks; client-server mode, ~20 Hz on a desktop GPU) is the grasp-model target. It must fit the cell's hardware: a Xavier NX 16 GB (weak GPU) and a Windows 11 PC (the training machine, Phase 6b).

## Alternatives

1. **Windows PC GPU via GraspGen's client-server mode** — the Jetson sends the segmented scene point cloud; Windows returns ranked grasps. Pros: runs at its designed speed, the NX stays light, the pose→joint converter (ADR-0009) is colocated, the PC is already the training machine. Cons: grasping depends on the PC being on and the LAN healthy.
2. **On the Xavier NX** — Pros: self-contained, no network dependency. Cons: a diffusion model on a 384-core Volta GPU means seconds per generation at best; TensorRT porting is real work; 16 GB RAM is tight with the full ROS2 stack.
3. **Hybrid** (small discriminator on the NX + full generator on Windows) — Pros: degraded-but-alive grasping if the PC drops. Cons: the most engineering; premature until option 1 is running.

## Decision

**GraspGen runs on the Windows 11 PC**, served through its client-server mode over the LAN.

## Consequences

- The Jetson's perception container sends the segmented object point cloud (scene OAK-D) to the GraspGen server as a request payload (LAN DDS/HTTP bridge node).
- The returned SE(3) grasps are converted to joint targets on the Windows side (per ADR-0009) or shipped as poses to a future IK node.
- Operational constraint documented: grasping requires the PC and the LAN.
- The GraspGen server setup (PyTorch environment, checkpoints including a gripper config adapted to the SO-100 jaw) is part of Phase 6b on Windows.
- The hybrid fallback stays available later if offline grasping becomes a requirement.

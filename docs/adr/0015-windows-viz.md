# ADR-0015: Windows visualization — noVNC in Chromium

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (custom option: noVNC viewed in Chromium)

## Context

The Windows 11 PC must visualize the arm's position and camera images. The Xavier already runs headless RViz2 served via noVNC (ADR-0014).

## Alternatives presented

1. Native Foxglove Studio (+ `foxglove_bridge` on the Jetson) — modern, but adds a bridge node and a second viewer stack.
2. RViz2 via WSL2 + Docker — pixel-identical to the Jetson, but WSL2 + WSLg + DDS-over-WSL networking is the highest-friction route.
3. Native Windows RViz2 — exists, but is the least-supported corner of the ROS2 ecosystem.

## Decision

**View the noVNC stream (RViz2 from ADR-0014) in Chromium on the Windows PC.** No ROS2 installation on Windows for visualization.

*Amended 2026-11-07:* the RViz2 stream is replaced by **Foxglove Studio in the browser** (user chose alternative 2 of the re-opened decision): noVNC/Chromium dropped, the browser connects over WebSocket to the foxglove_bridge in the web container. Reason: ros-humble-*.deb packages do not exist for Ubuntu 20.04 (the ROS apt repo only ships foxy for focal), so headless RViz2 would require a multi-hour from-source build of RViz + OGRE on the Xavier. Foxglove delivers the same URDF/TF/images/pointcloud view from the browser.

## Consequences

- Visualization needs zero ROS2/DDS setup on Windows: Chromium connects over LAN HTTP/WebSocket to the noVNC port of the RViz container.
- The FastDDS cross-machine discovery risk (ADR-0002) no longer applies to visualization; it still applies to the GraspGen bridge and the F710 `/joy` link (Phase 6c).
- noVNC at LAN latency is fine for visualization; full-rate camera feeds in the RViz tab may need throttled/compressed topics if bandwidth becomes an issue.
- The same endpoint becomes the "RViz" tab of the Phase 7 web interface — one viewer everywhere.
- Foxglove Studio remains an optional later addition for topic plots and data analysis, not for day-to-day viewing.

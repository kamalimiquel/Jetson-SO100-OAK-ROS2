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

*Amended 2026-11-07, reverted the same day:* the Foxglove detour was undone by the user's decision — the original noVNC pattern is restored, with **RViz2 built from source** in the web image (ros2/rviz + deps at humble, OGRE via rviz_ogre_vendor), because no humble debs exist for Ubuntu 20.04. Viewed in Chromium over noVNC (ws://xavier:6080).

## Consequences

- Visualization needs zero ROS2/DDS setup on Windows: Chromium connects over LAN HTTP/WebSocket to the noVNC port of the RViz container.
- The FastDDS cross-machine discovery risk (ADR-0002) no longer applies to visualization; it still applies to the GraspGen bridge and the F710 `/joy` link (Phase 6c).
- noVNC at LAN latency is fine for visualization; full-rate camera feeds in the RViz tab may need throttled/compressed topics if bandwidth becomes an issue.
- The same endpoint becomes the "RViz" tab of the Phase 7 web interface — one viewer everywhere.
- Foxglove Studio remains an optional later addition for topic plots and data analysis, not for day-to-day viewing.

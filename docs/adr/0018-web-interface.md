# ADR-0018: 3-tab web interface

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (alternative 1 of 3)

## Context

A web interface with three tabs: RViz, rqt_graph, and a terminal into the Xavier's ROS2 container. RViz cannot run in a browser; it is streamed (noVNC, ADR-0014, ADR-0015).

## Alternatives

1. **The noVNC-tabs + ttyd pattern**: noVNC tabs (RViz2 + rqt_graph) + ttyd terminal + a small static page with three tabs. Pros: every piece is mature, standard tooling; the RViz tab is literally the ADR-0015 endpoint. Cons: three services to keep alive.
2. **Embedded Foxglove + noVNC rqt_graph + ttyd** — Pros: smoother 3D. Cons: replaces literal RViz with something else; adds the `foxglove_bridge` node.
3. **Three containers behind nginx/caddy** — Pros: clean URLs per tab, TLS possible. Cons: more moving parts for little gain at this scale.

## Decision

**The noVNC-tabs + ttyd pattern**: RViz2 and rqt_graph headless via noVNC, terminal via ttyd (a shell **inside** the ROS2 container, not SSH to the host), unified by a small static tabbed page.

*Amended 2026-11-07:* literal RViz2 turned out infeasible on Ubuntu 20.04 (no humble debs for focal — the apt repo only carries foxy), and a from-source RViz+OGRE build is a multi-hour endeavor with bad history on this board. The user re-chose **alternative 2 of this ADR**: Foxglove Studio in the browser over a foxglove_bridge WebSocket (ws://xavier:8765), plus the ttyd terminal tab. Tabs: 3D view | Graph (Foxglove Rosgraph panel) | Terminal (ttyd).

## Consequences

- The `web` domain runs: RViz2 (noVNC), rqt_graph (noVNC), ttyd, and a static page embedding all three.
- The RViz tab reuses the ADR-0015 endpoint — same URL for Chromium on Windows and any browser on the LAN.
- ttyd opens a shell inside the ROS2 container under a dedicated user; LAN-exposed only; ports documented in the README.
- rqt_graph follows the same Xvfb/noVNC container pattern as RViz2.

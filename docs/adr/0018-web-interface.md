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

*Amended 2026-11-07, reverted the same day:* the Foxglove detour was undone by the user's decision — **alternative 1 of this ADR is restored**: RViz2 (built from source, no focal debs) headless via noVNC at :6080, terminal via ttyd at :7681, unified by the tabs page at :80. Tabs: RViz | Terminal.

## Consequences

- The `web` domain runs: RViz2 (noVNC), rqt_graph (noVNC), ttyd, and a static page embedding all three.
- The RViz tab reuses the ADR-0015 endpoint — same URL for Chromium on Windows and any browser on the LAN.
- ttyd opens a shell inside the ROS2 container under a dedicated user; LAN-exposed only; ports documented in the README.
- rqt_graph follows the same Xvfb/noVNC container pattern as RViz2.

*Amended 2026-11-09:* the shutdown button (:8099) and the Terminal tab (:7681) were
reported broken and are now confirmed working by the operator. The terminal is a
writable shell inside the web container, so `ros2` commands work there but `docker`
commands do not — use SSH for those.

# ADR-0014: RViz on the Xavier

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (alternative 1 of 3)

## Context

The Xavier must visualize the arm pose (URDF/TF), both camera feeds and pointclouds. The board runs headless in operation, and Phase 7 needs a web "RViz" tab.

## Alternatives

1. **Headless RViz2 in a container + noVNC** — the standard Xvfb/VNC + noVNC recipe. Pros: well-trodden pattern, no monitor needed, doubles as the Phase 7 web tab. Cons: one more container in the stack.
2. **Native RViz2 on the Xavier desktop** — Pros: direct, no streaming layer. Cons: needs a GUI session on the board; awkward for the web tabs anyway.
3. **Foxglove Studio on the Windows PC as the primary viewer** + minimal RViz on the Jetson for debugging — Pros: modern, light on the Jetson. Cons: does not meet the on-Xavier requirement; a complement, not the answer.

## Decision

**Headless RViz2 in its own container, viewed via noVNC.**

## Consequences

- The viz domain runs RViz2 headless (desktop image, baked URDF meshes, Xvfb/VNC) per the standard recipe.
- rqt_graph joins the same noVNC pattern in Phase 7, reusing the same container approach.
- The RViz config (panels for TF, both images, pointcloud, joint state) lives in the repo.
- Foxglove Studio on the Windows PC may be added later as a complementary viewer (Phase 6a).

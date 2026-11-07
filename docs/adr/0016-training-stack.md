# ADR-0016: Policy training stack on Windows

- Status: Accepted
- Date: 2026-11-07
- Decided by: user (alternative 3 of 3)

## Context

Learned policies (imitation learning / RL) train on the Windows 11 PC using episodes recorded via rosbag2 (ADR-0012) and LeRobot-format datasets. Two mature simulation stacks are candidates, each with a different strength.

## Alternatives

1. **MuJoCo** (mujoco-python) — Pros: free, fast, light; MuJoCo Menagerie ships the SO-ARM100 model and LeRobot provides a MuJoCo sim for this arm; ideal for daily RL/IL iteration. Cons: basic rendering, no photorealistic synthetic data.
2. **Isaac Sim / Isaac Lab** (Omniverse) — Pros: photorealistic rendering, domain randomization, synthetic vision data for VLM training; GraspGen exports grasps to USD for Isaac Sim. Cons: heavy install, needs a capable NVIDIA GPU, steep learning curve.
3. **Both** — MuJoCo for fast iteration, Isaac Sim for photoreal data and sim-to-real validation. Pros: each tool where it's strongest. Cons: two stacks to install and learn.

## Decision

**Both**: MuJoCo for daily policy iteration; Isaac Sim for photoreal synthetic data and sim-to-real validation.

## Consequences

- Start with MuJoCo (SO-ARM100 model, LeRobot integration) to close the train→evaluate loop quickly.
- Isaac Sim/Isaac Lab comes second, focused on domain-randomized visual data and VLM training data generation (its USD workflow also dovetails with GraspGen's USD export).
- The Windows PC's GPU determines Isaac Sim viability; if it lacks a supported NVIDIA GPU, Isaac Sim is deferred and MuJoCo carries training (flag before installing).
- Training artifacts (checkpoints, datasets) stay on the Windows PC; the repo only carries configs and launch scripts.

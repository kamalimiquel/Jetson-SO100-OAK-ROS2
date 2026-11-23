# ADR 0028 — physics in the loop, on the operator's PC, behind the arm's own interface

**Status:** accepted, 2026-11-22.

## Context

The twin (ADR-0019) mirrors the real arm by rosbridge. That was enough to prove that commands arrive and to
measure the lag — 1.3 deg at 20 Hz over a 10 deg ramp — and it says nothing about contact, gravity, an arm that
cannot reach under its own load, or a jaw closing on a block two millimetres too wide. Those are the questions a
manipulation policy has to face, so the simulator had to become something that answers them.

## Options considered

**A. Swap the viewer for a ROS-controlled simulator** (`mujoco_ros2_control`, the XLeRobot path). Idiomatic for
ROS, but it puts the physics behind topics: the supervisor talks HTTP to arms, so nothing above it would be the
same code in simulation and on the bench. Rejected: the thing validated would stop being the thing flown.

**B. Simulate at the arm's port.** Wrap the model in a server that answers `/claim`, `/torque`, `/write`,
`/read` exactly as the hardware does, and let MuJoCo step underneath. The supervisor, `skill_plan`, the journal,
the MCP endpoint and every test are then *the same code* against sim and against the servo bus; the profile
switch is a port number, and a missing simulator fails closed instead of falling back to the physical cell.
Chosen.

**C. Keep it kinematic.** Pose the arm, no dynamics. Cheapest, and useless for the questions above.

## Decision

1. **Physics never runs on the Xavier.** It ran there for an afternoon and was removed with its dependencies:
   the Xavier is the body, and a physics engine buys nothing while costing the RAM perception needs.
2. **`sim/arm_server.py` serves one arm; `sim/world_server.py` serves one world with several arms.** Two arms in
   one scene must share one `MjData` or the block falls through the second arm, because it is not in the model
   that is stepping. Leases and torque are per arm — each port answers for itself — while the world and the clock
   are shared.
3. **The ports bind `0.0.0.0` and the supervisor's port map accepts `host:port`.** A simulator on the operator's
   PC is a remote arm; a loopback default made the LAN request fail with ECONNREFUSED.
4. **The arms are position-driven, and the code says so.** Joints go where they are told, so a contact pushes the
   block, not the arm. Gravity, falls and contacts are real; a torque-limited arm needs actuators and a control
   law between the supervisor and the physics, and that is a separate decision (ADR pending).
5. **Physics does not start until the viewer has drawn once.** A 20 cm fall takes 0.2 s; starting the clock in the
   constructor meant the block was already resting when the window appeared, which reads as "it never fell".
6. **The server counts steps and reports a dead loop.** A Python thread that raises dies silently while the ports
   keep answering every request, so the world exposes `steps`, `loop_error` and prints `PHYSICS LOOP DIED`.
7. **`sim/model_facts.py` exists so a mechanism is measured, not guessed.** Three explanations were asserted in a
   row (gravcomp, friction, springs) before anyone integrated a second of raw MuJoCo; it prints the disable
   flags, the per-joint passive parameters, and what actually moved.

## Consequences

- A mission can be rehearsed against physics with the real supervisor, and a failure that is physical (the block
  does not lift) is distinguishable from one that is not (the solver refused).
- The arm holds poses it would not hold in reality when the model carries gravcomp, joint friction or springs;
  those three are now reported by `add_block`, because a model that is *better* than the machine misleads.
- Nothing here validates a grasp until the block is placed along the jaw's closing axis, which `/objects` does not
  yet expose.

# ADR 0027 — one supervisor owns the arms; a skill is written only when the solver agrees

**Status:** accepted, 2026-11-21. Applies to `docker/ws/control/supervisor.py`, `docker/ws/lib/skill_plan.py`,
`ros/so100_cell_interfaces`.

## Context

The command contract (ADR-0024) made one writer per arm possible and the measured scales (ADR-0026) made a
degree mean a degree. What neither answers is the question an agent actually asks: *"put the arm there"* —
and the previous answer was that several parties could write it: the pad, the web button, a script, anything
on the loopback port. Each of them is defensible alone; together they are a bus with no admission control.

Chapter 2 of the MCP specification asks for a local owner, and section 6 for observable contracts. The
mechanical part already existed — a lease at each arm's port, `X-Owner`, 423 for everyone else. What was
missing was something that *holds* that lease for the duration of a mission, judges what is being asked, and
is the single place where a request becomes a packet.

## Options considered

**A. Let every client claim its own lease.** The lease is already the mechanism; a policy or a script claims
directly and writes.
*Pros:* no new node, no new surface, the port is the only contract. *Cons:* every client has to reimplement
the same judgement (units, limits, jaw curve, reachability) or skip it; two clients that both want the arm
race each other; and nothing correlates a request with the motion it produced, so a mission's outcome cannot
be scored. Rejected: it moves the safety rules into N clients.

**B. A supervisor node that owns the ports and serves a typed skill service** (chosen).
*Pros:* one place for the judgement, one place for correlation, and the arm is mechanically owned for the
length of a mission rather than per command. The judgement lives in a pure module with no ROS and no sockets,
so it is tested headlessly (30 checks) and cannot drift from what the node does. *Cons:* a single point of
failure — if it dies holding the leases, the operator gets 423 until the TTL expires. Mitigated: renewal in a
daemon thread, release on every exit path, and a short TTL by default.

**C. Put the judgement in the arbiter.** It already sees every command on the topic and is the natural place
for a veto.
*Pros:* no new process, the arbitration path is where refusals already live. *Cons:* the arbiter is
deliberately a *topic* component — it has no ownership of the ports, so it cannot stop a direct write; and it
has no notion of a mission, so it could not correlate or score one. Rejected: right instinct, wrong layer.

## Decision

1. **`cell_supervisor` owns the arms for a mission.** It claims each arm's port with its own name, renews from
   a daemon thread (this image's `rclpy` `Timer` rejects the clock it is handed — a renewal needs a period, not
   a clock), and releases on every exit path, one port at a time with its own error so one dead arm does not
   hide the other.
2. **The judgement is a pure function.** `skill_plan.plan(skill, mission_id, args, ctx)` decides and returns
   an action or a refusal with a sentence; it imports neither ROS nor sockets, and it is the only place that
   knows a skill exists. A mission id is mandatory: an outcome with no mission cannot be scored or traced.
3. **Nothing is written that the solver has not blessed.** `move_pose` validates the pose (finite, normalised
   quaternion, that arm's base frame only, arm owned and reporting), dispatches a `SolveRequest` whose
   `request_id` is the ticket, and writes the joints only when the correlated `SolveResult` arrives with
   `ok`. Refusals are logged and nothing moves; a result with the wrong joint count is refused, not padded.
   The solver never drives the jaw — `hold: ["gripper"]` is explicit.
4. **The node is not started by `entry.sh`.** A supervisor holding the leases answers 423 to the operator's own
   writes. It comes up with a mission, and the default arrangement stays exactly as it was.
5. **Tolerances are the caller's, bounded by the node's own.** An agent asking for 10 mm does not get 10 mm;
   the node's numbers are the fuse.

## Consequences

- The chain agent → supervisor → IK → bus exists and is testable end to end without an arm for the judging
  half, and was verified on the bench for the executing half: with torque off, `move_joint` reached the port
  and stopped at the operator's switch (409), an unknown arm was refused before any ticket, `open_jaw 40 mm`
  resolved through that arm's own curve, and stopping the node returned `/owner` to null on both ports.
- Two names for the same arm now meet in one file: the port is per serial (`so100_cell`), the topic per
  namespace (`arm1`). The map is written once in the supervisor, not guessed at each use.
- A supervisor that dies badly can still block the operator for one TTL. That is a real cost, chosen over the
  alternative of no owner at all, and the reason the default TTL is short.
- The web surface and the commanded twin (chapter 2, row 7) are still missing: the supervisor is reachable by
  service and by `curl`, not yet by the page the operator actually looks at.

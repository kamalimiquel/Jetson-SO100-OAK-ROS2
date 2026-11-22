# The MCP endpoint — plan, phases and the test that closes each one

Spec sections 6–8. Written 2026-11-22, against `main`.

## The one rule everything else follows from

**The endpoint does not drive the robot; it asks.** It runs outside the containers, with no ROS, no `rclpy`
and no serial port. Every motion goes through `cell_supervisor`, which already claims the leases and already
knows how to refuse. If the endpoint dies, the cell keeps working exactly as before — that is a requirement,
not a happy accident.

Corollary: the endpoint must never be able to claim a lease. The moment it can, there are two owners and the
ownership model is gone.

## Phases

| phase | deliverable | closed by |
|---|---|---|
| 0 | identity and manifest: `capabilities`, `get_state` | a manifest that reports an unreachable arm as unreachable and a boot epoch that changes with the arm |
| 1 | Streamable HTTP transport, two profiles (`robot`, `sim`) | `sim` with no simulator refuses to start instead of falling back to the physical cell |
| 2 | `move_arm_bounded`, `set_gripper_aperture` (over `/cell/skill`) | four negative tests before the positive one: foreign lease, stale epoch, expired deadline, wrong start pose |
| 3 | `solve_ik` (report-only), `detect_objects`, `camera_snapshot` via owner RPC | a snapshot names its camera, its stamp and its hash; an unknown camera is refused with the list |
| 4 | `get_goal`, `cancel_goal`, `stop`, journal | killing the endpoint mid-mission leaves the arm stopped and the journal honest; a retry does not move twice |
| 5 | the whole thing against the twin first | a mission that has not passed in simulation never runs on hardware |

## What each tool may do

| tool | backing | it may |
|---|---|---|
| `capabilities`, `get_state` | config, arms' `/owner`, `/status`, `/torque`, `/scale`, `images.lock` | read only |
| `move_arm_bounded` | `SkillRequest` → supervisor → IK → `/write` | ask; the supervisor decides |
| `set_gripper_aperture` | same, jaw mm resolved by that arm's curve | ask |
| `solve_ik` | `SolveRequest` (report-only) | compute; never write |
| `detect_objects`, `camera_snapshot` | owner RPC, images never enter this container | read with an observation id |
| `get_goal`, `cancel_goal`, `stop` | journal + local stop path | stop is local, never queued behind the agent |

A request carries **boot epoch, calibration revision, mission id, deadline and the start pose the caller
believes**. The supervisor refuses when the epoch or the pose disagree: that is what stops a plan computed
against a robot that has since restarted.

Refusals are translated, not swallowed: `423` → `busy`, `409` → `torque_off`, `IK ok=false` →
`unreachable_pose` with the solver's sentence, and a step below the measured deadband → **accepted with a
warning** (it is not an error; the servo simply may not move).

## Risks, and what contains each

| risk | contained by |
|---|---|
| the agent decides on a 300 ms-old observation | deadline per request, mandatory `observation_id` |
| pad and agent writing at once | the supervisor owns the leases; the endpoint cannot claim |
| deadband: commanded a move that never happened | the warning already implemented in `skill_plan` |
| images cannot enter the container | owner RPC with hash and stamp |
| a retried request moves twice | `mission_id` idempotence + epoch |

## What this plan refuses to do

Expose `/write` or the arm port as a tool. Run inference in the MCP container. Open rosbridge to the agent.
Give the endpoint the ability to claim an arm. Each of those is one line of code and the end of the model.

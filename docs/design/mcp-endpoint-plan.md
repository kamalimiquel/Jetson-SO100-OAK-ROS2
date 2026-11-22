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
| 2 | `move_joint_bounded`, `set_gripper_aperture` (over `/cell/skill`) | four negative tests before the positive one: foreign lease, stale epoch, expired deadline, wrong start pose |
| 3 | `solve_ik` (report-only), `detect_objects`, `camera_snapshot` via owner RPC — **done 2026-11-22** | a snapshot names its camera, its stamp and its hash; an unknown camera is refused with the list |
| 4 | `get_goal`, `cancel_goal`, `stop`, journal — **done 2026-11-22** | killing the endpoint mid-mission leaves the arm stopped and the journal honest; a retry does not move twice |
| 5 | **physics in the loop**, then the whole thing against it | a mission that has not passed with contact and gravity never runs on hardware |

## What each tool may do

| tool | backing | it may |
|---|---|---|
| `capabilities`, `get_state` | config, arms' `/owner`, `/status`, `/torque`, `/scale`, `images.lock` | read only |
| `move_joint_bounded` | `SkillRequest` in **joint space** → supervisor → `/write` | ask; the supervisor decides |
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

## Two corrections that change the shape

**Motion is joint space, not pose.** `move_pose` and the IK stay, but as *report-only*: an agent may ask what
joint angles a pose would need, and the answer is advice. Motion itself is a named array of degrees —
`shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll` plus the jaw in millimetres — which is what
the scale table, the deadband and the per-arm calibration describe, and which needs no extrinsics, no TCP and
no hand-eye. Pose-space motion would drag the retired hand-eye back in through the side door; joint space does
not depend on it at all. The price is that the agent has to do the reasoning — and that price is worth paying
while the calibration that would justify pose commands does not exist.

**Physics has to be in the loop now.** A kinematic mirror was enough to prove that commands arrive and that the
transient lag is ~1.3 deg at 20 Hz, and it says nothing about contact, gravity, the arm that cannot reach under
its own load, or what happens when a jaw closes on a block that is 2 mm too wide. So the simulator stops being
a viewer and becomes an arm.

The way to do that without a rewrite is not a ROS-level swap: **the physics model is wrapped as a virtual arm
server on its own port, with the same four endpoints the real arm exposes** — `POST /claim`, `POST /torque`,
`POST /write`, `GET /read` — backed by MuJoCo (`mujoco` or `mujoco_ros2_control` for the model and the loop).
Then the supervisor, the skill planner, the MCP endpoint and the tests are the *same code* against sim and
against hardware, the profile switch is a port number, and the sim profile still fails closed: no simulator
listening means no cell, never the real one by accident.

Acceptance for that phase: a mission that fails on the real arm — a grasp that closes 2 mm short, an elbow
that sags 6 deg under load — fails in the sim too, for a physical reason, and the twin reports the contact.

## What this plan refuses to do

Expose `/write` or the arm port as a tool. Run inference in the MCP container. Open rosbridge to the agent.
Give the endpoint the ability to claim an arm. Each of those is one line of code and the end of the model.

It also refuses to put physics behind a ROS topic swap: a simulator that speaks topics but not the port
contract is a second integration, and the thing we would be validating would no longer be the thing we fly.

## Phase 3, as built

- **`solve_ik`** is a `skill_plan` skill of kind `solve_report`: the supervisor publishes the same
  `SolveRequest` it uses for `move_pose`, waits for the correlated `SolveResult` and returns it — and
  never calls `/write`. The endpoint defaults its deadline to 5 s because the DLS solver takes ~3 s
  on this CPU; a motion still gets the 1 s the write path is judged against. Canonical `arm1`/`arm2`
  are mapped to the ports' serial names inside `skill_plan`, and a pose in any frame but the arm's own
  (`arm1_base` / `arm2_base`) is refused with the frame it wanted.
- **`camera_snapshot` / `detect_objects`** go to the owner: `docker/ws/cams/owner_rpc.py` serves
  `GET /cameras`, `POST /snapshot` and `POST /detect` on :8098, keeps the freshest frame per camera
  and writes the JPEG under `/state/snapshots`. The endpoint only ever sees `{camera, stamp, age_s,
  sha256, bytes, path, observation_id}` — never the pixels. An unknown camera is refused with the
  list; a frame older than 2 s is `STALE`; with no tracker installed `detect_objects` says `MISSING`
  rather than inventing boxes.
- **Verified on the Xavier** (2026-11-22): `tools/list` returns the seven tools; a real `arm1_wrist`
  snapshot came back with its stamp, age 0.03 s and sha256; `camera_snapshot("nope")` was refused with
  `["arm1_wrist","arm2_wrist","scene"]`; `solve_ik` returned the solver's own answer
  (`ok=false, pos_err=…, iters=140`) for poses the arm could not reach, which is the honest report.
- **The positive case, closed:** `tools/ik_pose_fixture.py` turns a joint vector into the pose the
  tool actually reaches (FK + quaternion). Fed back, `solve_ik` answered `ok=true, pos_err=0.0 mm,
  ori_err=0.0 deg, iters=7` — the report-only path is proven in both directions.

## Phase 4, as built

- **Journal** (`docker/ws/lib/journal.py`): append-only JSONL at `/episodes/journal.jsonl` (mounted
  from the host, so it outlives the container), one line per `/skill` call with mission, skill,
  ticket, verdict, reason, args and epoch. `read`, `last_for` and `find_accepted` are pure functions.
- **Idempotence** is scoped to the skills that write (`move_joint`, `move_pose`, `hold`, `open_jaw`,
  `close_jaw`). A retried `mission_id` that already wrote is answered with its original ticket and
  the note "nothing was written twice"; a report-only `solve_ik` is deliberately *not* in that set,
  because a report changes nothing and may legitimately be repeated.
- **Stop is local.** `POST /stop` latches and drops anything pending; the latch is checked twice —
  when the skill arrives and again immediately before `/write` — so a request that was already on its
  way cannot slip through. `/resume` is an operator action (POST), not an agent tool: an agent that
  stopped a cell should not be the one to un-stop it. `/cancel` drops the pending tickets of one
  mission and refuses it from then on; `/goal` and `/journal` read the file.
- **Tools**: `get_goal`, `cancel_goal`, `stop` (ten in total now), each a direct HTTP call to the
  supervisor — `stop` never queues behind the agent, it is one POST with a 3 s timeout.
- **Verified on the Xavier** (2026-11-22): `/stop` → `{"stopped": true}`, the next `move_joint`
  refused with "stop is engaged", `/resume` → `{"stopped": false}` and the next one back to the
  ordinary 409 torque-off; a retry of a mission already accepted returned `seed-2` with "nothing
  written twice"; `get_goal` returned that entry, an unknown mission `null`; all three tools through
  `/mcp`.
- **Not exercised with torque on:** no real write was accepted during this pass (the arms were left
  with torque off), so idempotence was proven against a seeded journal entry rather than against a
  motion that actually happened. The code path is the same; the honest label is "seeded".

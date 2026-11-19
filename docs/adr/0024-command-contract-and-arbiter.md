# ADR-0024: a command contract, one writer per arm, and episodes that keep their own stamps

- Status: Accepted (the contract, the server boundary and the recorder are implemented and tested
  headless; **the enforcing mode of the arbiter has not been run on the bench**)
- Date: 2026-11-19
- Decided by: the user, acting on the review of `SO100_policy_specification.md` §2

## Context

`/armN/joint_targets` is a `std_msgs/Float64MultiArray`: **no names, no stamp, no owner**. Five, six
or seven values all "worked" — `zip` in `/write` truncated in silence, so five left the jaw at its
previous goal and seven dropped the last, and the arm moved either way. Three clients publish that
topic (the pad, an applying IK node, anything under test) and each reaches the same serial bus; the
loopback port is the only boundary a topic-level guard cannot be walked around, because the writable
ttyd on `:7681` can `curl 127.0.0.1:8002/write` from inside the container. The server also wrote
motor by motor, so an exception on the fourth left three moved and three not.

The merged cell state had the same shape of defect from the other side: `joint_merge.py` ended with
`out.header.stamp = msg.header.stamp` — the last arrival's — so an arm that died kept its frozen pose
under a fresh stamp. Not stale: **disguised as current**. And nothing consumed an episode: the ADRs
selected rosbag2 plus an offline export, but no recorder existed, so a policy comparison had no
artifact with a per-arm age in it.

## Alternatives

1. **Document the array's convention and leave the code alone.** Cheapest, and it is what the table in
   the spec describes today. Keeps the only interface in the cell where one wrong element raises
   nothing, and keeps the merged state lying about which arm is alive.
2. **Alt 1 (this ADR)** — a single shared contract module used by the joint server *and* an arbiter
   node; exclusive ownership at the HTTP port; one sync packet per command; the merge stamped with
   the oldest live arm and a silent arm dropped after 0.5 s; an episode recorder that stores each
   message's own stamp. Cost: two new nodes and a visible behaviour change in `/write`.
3. **A ROS 2 action/service with named fields, replacing the topic.** Structurally right — names and a
   goal id in the message — but it re-points the pad, the IK node, the compat bridge and every
   rosbridge client at once, and the pad's 50 Hz stream does not want action semantics. Rejected as a
   bigger change than the defect requires; worth revisiting when a policy becomes the main writer.

## Decision

**Alt 1.** Concretely:

- `docker/ws/lib/command_contract.py` — the only place a command becomes valid. Six values in
  `JOINT_ORDER`; five are legal **only** with `"hold": ["gripper"]`, and the array length must equal
  the joints not declared held. Non-finite and out-of-range values are refused (400) naming the joint;
  `"clamp": true` is the explicit "as far as it goes". On the topic, where JSON's `hold` does not
  exist, `NaN` is the in-band way to say "leave this one alone" and is accepted **only** for the jaw.
- `lerobot_joint_server.py` — validates through the contract, answers **400** (not 500) with the
  sentence, drives every driven joint in **one** `sync_write` packet, and on a failed packet re-reads
  the encoders: if the arm is where it was told, the packet counts as delivered; if not, the path is
  held (**409** with the fault and the measured pose) until torque is cycled or the arm re-claimed.
- **Ownership at the port**: `GET /owner`, `POST /claim {"owner", "ttl_s"}`, `POST /release`. While a
  lease is held, `/write` and `/torque` require `X-Owner: <owner>` and answer **423** otherwise.
  Unclaimed is the default, which is what keeps the web torque button and the pad working unchanged.
- `docker/ws/control/arbiter.py` — subscribes `/arm1|2/joint_targets`, validates with the same
  contract, keeps **one** write in flight per arm (a newer command supersedes the pending one instead
  of queueing a backlog), records the receipt time as the only honest stamp, and publishes
  `/cell/arbiter` (1 Hz JSON: sent/observed/superseded/rejected/stale/faults, the sentence while held).
  `ARBITER_ENFORCE=0` (default) observes and validates only, with `arm_bridge` still writing;
  `ARBITER_ENFORCE=1` claims each arm's lease (2 s ttl, renewed every 0.5 s) and becomes the sole
  writer, which requires `ARM_WRITE_MODE=arbiter` on the arm containers so `arm_bridge` stops
  subscribing to targets.
- `joint_merge.py` — the merged message carries the **oldest** live arm's stamp, and an arm silent for
  more than `MERGE_STALE_S` (0.5 s) loses its joints with one log line, so `robot_state_publisher` and
  RViz lose that arm instead of drawing a lie. The return is logged too.
- `docker/ws/control/recorder.py` + `docker/ws/lib/episode_log.py` — an episode under `RECORD_DIR`,
  written from a **separate thread** (writing inside the executor starved the subscriptions on the
  board):
  `manifest.json`, `observations.jsonl`, `commands.jsonl`, `frames/*.jpg`. Arm-specific state only
  (`/armN/joint_states`, never the merged topic), each observation with the message's own stamp, the
  receipt clock and the age between them; stale samples are counted, not smoothed. Refuses to start
  with `RECORD_DIR` empty.
- `tools/check_namespaces.py` grows `check_command_surface()`: one packet, no per-motor loop, the
  contract enforced at the port, a lease endpoint, and the recorder never reading the merged state.

## Consequences

- **A wrong command is now a sentence.** The pad already sends six; `tools/handeye_capture.py` used to
  send five on purpose and now sends six with the jaw commanded to its present position (rosbridge
  cannot carry `NaN` in JSON, so `hold` has no in-band form there).
- **Enforcing mode changes who may move the arm.** With `ARBITER_ENFORCE=1` the pad's direct publishes
  are refused until the pad (or a supervisor) goes through the arbiter; that is deliberate and is why
  the switch defaults to observe.
- **A dead arm is visible.** RViz loses it after 0.5 s instead of holding it; the merged topic's stamp
  is now the oldest member's, so any age computed from it is honest rather than flattering.
- **Episodes are files, not bags.** Two JSONL files and a manifest, with the pixels the policy would
  see. A bag remains the tool for graph-level inspection; the episode is the record of the decision.
- The jaw in TF is still a drawing: `deg2rad.JAW_RAD_PER_PERCENT` is an explicit uncalibrated proxy,
  and the recorder's manifest says `calibrated: false`.

## Verification

Headless, no hardware, all green at this commit:

- `tools/test_command_contract.py` — 36 checks (five alone refused with the hint, five + hold
  accepted, seven refused, NaN/inf, limits, `clamp: true`, malformed bodies, `NaN` legal only in the
  jaw slot on the topic).
- `tools/test_joint_merge.py` — 21 checks (oldest live stamp; an arm 0.7 s silent dropped; the drop
  logged once; the return logged; no live arm → no message).
- `tools/test_arbiter.py` — 29 checks (forward/reject/stale/hold/supersede, one HTTP write per tick to
  the arm's own port with `X-Owner`, 409 holds without counting a fault, 423 names the other writer,
  502 is a fault and holds, observing mode sends **nothing** and claims **nothing**).
- `tools/test_episode_recorder.py` — 27 checks (age = receipt − stamp, stale counted, frame written
  once, no stamp → no age, manifest counts match the lines, two episodes in one second do not share a
  directory, the node subscribes to `/armN/joint_states` and never to `/cell/joint_states_rad`).
- `tools/test_joint_server.py` — 29 checks against the **real handler** on a loopback port with a fake
  bus: five values → 400 with the hint, seven → 400, NaN on an arm joint → 400, 190° → 400 naming the
  joint, nothing reaching the bus for any of them; torque off → 409; one `sync_write` for six joints
  and not one per-motor write; five + `hold` puts five joints in the packet and says so; claim → 423
  for every other writer including a wrong `X-Owner`; a failed packet → 502 with the measured pose and
  then 409 until torque is cycled; a packet that failed but landed answered 200 with the note.
- `tools/check_namespaces.py` → `the invariant holds`, including the new command-surface section.
- `tools/test_compat_bridge.py` → 11 checks (unchanged), `tools/test_ik_frame.py` → 10 (unchanged).

**On the Xavier, 2026-11-19** (rsync + `compose up -d --force-recreate arm arm2 control`, rosbridge
restarted), torque off on both arms:

- `POST /write` with five values → **400** with the hint; seven → 400; `190°` → 400 naming
  `shoulder_pan`; six with torque off → **409**; `POST /claim {"owner":"test"}` → `/write` without the
  header → **423** (`test holds this arm (19.8s left)`), `/release` → `{"owner": null}`.
- `/cell/arbiter` publishes the JSON status; the arbiter logs `observing only (ARBITER_ENFORCE=0)`.
- **Dropout proven on the bench**: `docker stop so100-arm2` → `/cell/joint_states_rad` goes from 12
  names to arm 1's 6 and logs `arm2/joint_states_rad silent for more than 0.5s — dropping its joints
  (RViz will lose that arm, on purpose)`; `docker start` → back to 12 and `publishing again`.
- Merged stamp age measured with a probe: min 0.016 s, **median 0.03 s**, max 0.183 s (arm 1 at
  ~22 Hz, arm 2 at ~45 Hz). Arm 1 was dropped once for 0.9 s while `ros2 topic echo` and Python probes
  shared the board — the threshold is `MERGE_STALE_S`, raise it to 1.0 if RViz blinks under load.
- A real episode: 12 s, 47 + 46 arm samples, 51 wrist frames, 24 scene frames, max age 0.36 s, no
  stale sample, manifest carrying the two caveats (jaw proxy, scene without metric pose).

Not yet verified: enforcing mode with the pad in hand (it is off by default) and `sync_write` on the
bus with torque on — the arms were loose and nothing was moved.

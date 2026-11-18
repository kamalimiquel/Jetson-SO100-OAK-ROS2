# ADR-0023: symmetric namespaces for both arms + a thin compatibility bridge

- Status: Accepted
- Date: 2026-11-17
- Decided by: user ("implementa Alt 1 — symmetric namespaces for both arms")
- Supersedes in part: the naming half of ADR-0009 (arm 1 keeps the bare topics); the join described
  in the two-arm plan Phase 4 stays, with different names

## Context

Since Phase 1 the cell has been **asymmetric**: arm 1 published `/joint_states`, `/joint_targets`,
`/joint_states_rad` and `/robot_description` on the bare names and its TF rooted at `base`, while arm 2
lived under `/arm2/*` with the `arm2_` frame prefix. That saved churn on the released 1.0 surface and
it is the reason README §16 R2 exists: a single frame without a prefix **silently merges the two TF
trees** — RViz draws one arm's shoulder on the other's base and nothing reports an error. The prefix
was applied at one `robot_state_publisher` (`frame_prefix: arm2_`) and hoped to hold everywhere else,
which is the kind of rule that survives right up to the day someone adds a static transform.

## Alternatives

1. **Status quo** — arm 1 bare, arm 2 prefixed, prefix enforced by convention. Cheapest; keeps the
   whole class of "bare name collides" bugs, and the failure has no symptom until RViz looks wrong.
2. **Alt 1 (this ADR)** — prefix *both* arms (`/arm1/*` + `arm1_*`, `/arm2/*` + `arm2_*`) and add one
   small re-publisher that serves the bare 1.0 names from arm 1. Removes R2 by construction — no
   unprefixed name exists anywhere — at the cost of one-time churn plus a new moving part.
3. **Alt 2** — physical-port udev naming plus a single generated geometry file. Addresses R1/R3, not
   R2: with arm 1 still bare, a stray unprefixed frame still merges the trees.

## Decision

**Alt 1.** Concretely:

- `ARM_NS=arm1` for the first arm (compose), like `arm2` for the second: topics `/arm1/joint_states`,
  `/arm1/joint_targets`, `/arm1/joint_states_rad`, `/arm1/robot_description`, joint names prefixed
  `arm1_` by `deg2rad`, frames `arm1_*`.
- `tools/make_cell_urdf.py` prefixes **both** arms (`--prefix1 arm1_`, `--prefix2 arm2_`), joins them
  with `arm1_mount` (`arm1_base → arm2_base` at the offset in `cell_geometry.json`) and **refuses to
  write** a model with a bare link or joint. The cell URDF's root is `arm1_base`.
- The static `base → arm2_base` transform is **gone** from `docker/ws/arm/entry.sh`: the fixed joint of
  the URDF is the only writer of that number, so there is no bare `base` frame and no second source.
- `docker/ws/arm/compat_bridge.py` (started by arm 1's `entry.sh`) republishes the 1.0 surface:
  `/arm1/joint_states → /joint_states`, `/arm1/joint_states_rad → /joint_states_rad`,
  `/arm1/robot_description → /robot_description` (latched), and `/joint_targets → /arm1/joint_targets`
  so a 1.0 client that publishes targets still moves arm 1. Messages are forwarded **verbatim** (same
  header stamp), not regenerated.
- **Frames are not aliased.** `base` stays gone. A 1.0-era consumer that pinned `base` (RViz fixed
  frame, `tf2_echo base …`) must say `arm1_base`; resurrecting `base` would put the collision back.
- Consumers are moved once: RViz fixed frame `arm1_base`, the pad (`ARMS`), the MuJoCo twin, both IK
  instances (`ARM_NS=arm1`/`arm2`), the hand-eye tools (`--arm-ns`, default `arm1`), the wrist cameras
  (`/arm1/gripper/*`, `/arm2/gripper/*`, the pair under `/cell/gripper/cameras/compressed`) and the
  wrist frame `arm1_gripper_color_optical_frame`.
- The rule is executable: `tools/check_namespaces.py` fails on any bare link/joint in the cell URDF,
  any static transform with a bare frame id, any topic literal outside `/arm1 /arm2 /oak /cell /ik`,
  and any use of the 1.0 names outside `compat_bridge.py`.

## Consequences

- **R2 is gone by construction**, and `check_namespaces.py` keeps it gone: a new file that types
  `/joint_states` fails the check instead of shipping.
- The compat bridge is the one new moving part. If it dies, the **1.0 surface** goes quiet while the
  cell keeps running (the containers talk to `/armN/*` directly). `tools/test_compat_bridge.py` pins
  its wiring, the message identity and the absence of TF publishing.
- Both arms are now symmetric in the Graph tab, in the pad and in the docs, so "which arm" is always
  the first segment of a topic.
- The released 1.0 tag keeps working for **topics** only; frame names changed, which is a documented
  break (`base` → `arm1_base`, `gripper_color_optical_frame` → `arm1_gripper_color_optical_frame`).
- MuJoCo model names (`base`, `arm2_base` in the generated MJCF) are **not** ROS frames and are left
  alone; the twin's geometry check keeps comparing those body names against `cell_geometry.json`.

## Verification

- `tools/check_namespaces.py` → `the invariant holds` (31 links+joints prefixed, mount joint
  `arm1_base → arm2_base` at `0 0.3 0`, 35 topic literals scanned, 1.0 names only in the bridge).
- `tools/test_compat_bridge.py` → 11 checks, including "the output IS the input (no copy, no
  re-stamp)" and "no /tf anywhere in the bridge".
- `tools/make_cell_urdf.py` → 16 links, 15 joints, root `arm1_base`, zero bare names.
- `tools/test_two_arm_switch.py` against a **fake rosbridge** (protocol v2 over WebSocket, two
  simulated arms): 40 of 41 checks — the pad finds `/arm1/joint_states`, switches with A, arms with B
  on the pose it reads, records with X and replays with Y. The single failure is the check that reads
  the real torque proxy over the LAN, which needs the cell.
- **On the Xavier, 2026-11-18** (deployed and recreated): `ros2 topic list` shows `/arm1/*`, `/arm2/*`,
  `/cell/*` and the four bare names, nothing else arm-related; `ros2 node list` shows
  `/arm1_arm_bridge`, `/arm1_deg2rad`, `/arm1_ik`, `/arm2_*`, `/compat_bridge`, `/joint_merge`;
  `tf_frames.py` reports **8 static + 12 moving frames, every one prefixed**, `arm2_base ← arm1_base
  (+0.000, +0.300, +0.000)`, and **no `base` frame** (the only orphan is `scene_camera_link`, the
  unmeasured hand-eye island); `/arm1/joint_states` and `/joint_states` carry the same six numbers at
  the same instant; a bare `/joint_targets` publish arrives on `/arm1/joint_targets` unchanged with
  torque off; `/arm1/gripper` streams with frame `arm1_gripper_color_optical_frame`; RViz renders
  from Xvfb `:10` with Fixed Frame `arm1_base` and no fixed-frame error in its log.
- **Rate**: the bridge keeps up when the board is calm (24.2 → 24.0 Hz) and loses ~20 % when it is
  loaded (27.0 → 22.0 Hz with RViz running) — it republishes synchronously. Arm 1's own 24–27 Hz
  against arm 2's 37–50 Hz is the joint server's read loop, unrelated to this change.
- **Found only on the bench**: `deg2rad.py` and `ik_node.py` named their nodes `arm2_*` for *any*
  namespace (arm 1 introduced itself as arm 2, and `ros2 node list` showed two `/arm2_ik`), and the
  cams container had no `/dev/arm1_wrist` mount so arm 1's wrist camera published nothing. Both fixed
  and re-verified; they are why "it passed headless" is not "it works".

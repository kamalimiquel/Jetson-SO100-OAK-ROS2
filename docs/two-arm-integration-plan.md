# Integrating a second arm and a second wrist camera — steps and tests

Plan for going from the 1.0 cell (one arm, one OAK, one wrist webcam) to a two-arm cell, without
breaking what already works. Every phase ends with tests that say pass or fail; nothing advances on
"it looks fine".

Four requirements drive the design and are the acceptance criteria at the end:

1. the **Cams** tab (today *Wristcam focus test*) lets you pick **which wrist camera** you are looking at;
2. **RViz** draws both arms **30 cm apart**;
3. the **MuJoCo twin** draws both arms 30 cm apart too;
4. the **F710** switches teleoperation from arm 1 to arm 2 by **holding A for 5 s**.

Naming decision, taken once and applied everywhere: **arm 1 keeps the topics it has today**
(`/joint_states`, `/joint_targets`, `/gripper/...`, frames `base…gripper`) so the released cell keeps
working, and **arm 2 lives under `/arm2/…`** with the TF prefix `arm2_`. Renaming arm 1 to `/arm1`
would be tidier and is a later option; it touches the web page, the twin, the pad and every doc, and
buys nothing today.

---

## Phase 0 — Names for the hardware (before any code)

The two adapters are the same chip, so `VID:PID` cannot tell them apart; the serials can. Mount the
mounting: arm 1 and its camera go to fixed ports and stay there.

**Work**
- Two udev rules keyed on `ATTRS{serial}` → `/dev/arm1_tty` (`5971081400`) and `/dev/arm2_tty`
  (`5A46085595`), replacing the single ambiguous `so100_arm` rule.
- Two rules for the cameras: by serial if the unit reports one, otherwise by physical port
  (`KERNELS=="1-2.2.2"`) → `/dev/arm1_wrist`, `/dev/arm2_wrist`.
- Label the cables and the hub ports; keep a photo of the hub in `docs/`.

**Tests**
| Test | How | Pass |
|---|---|---|
| T0.1 symlinks are stable | `udevadm trigger` ×5, then reboot ×2, checking `ls -l /dev/arm*_tty /dev/arm*_wrist` | the four links point at the same serials after every boot |
| T0.2 no leftover ambiguity | `ls -l /dev/so100_arm` | removed, or an alias only for arm 1 |
| T0.3 arms answer on their own link | `~/lerobot/.venv/bin/python tools/lerobot_bus_probe.py /dev/arm1_tty` and `… arm2_tty` | **6/6 motors** on each, no timeout |
| T0.4 cameras open on their own link | `~/lerobot/.venv/bin/python tools/two_cam_bandwidth.py /dev/arm1_wrist /dev/arm2_wrist` | both open, both ≥9 fps, 0 failed reads |

Fail signal: a link that moves between boots — stop here, the rest of the plan assumes stable names.

---

## Phase 1 — The second arm on the bus, no UI yet

**Work**
- `arm_bridge.py`, `deg2rad.py`, `torque_api.py`: take the namespace from the environment
  (`ARM_NS`, default empty) instead of hardcoding `/joint_states`, `/joint_targets`,
  `/robot_description`.
- New service `arm2` in compose: same image, `ARM_NS=/arm2`, `JOINT_PORT=8003`, `TORQUE_PORT=8095`
  (8092 is arm 1's proxy; 8093 and 8094 are the web tabs), device `/dev/arm2_tty:/dev/ttyACM0`,
  calibration id `so100_cell_b`.
- `robot_state_publisher` for arm 2: `frame_prefix: arm2_`, remap `joint_states:=/arm2/joint_states_rad`.
- Static transform `base → arm2_base` at **(0, 0.30, 0)** — the 30 cm of requirement 2, in metres,
  published once on `/tf_static` (same mechanism as the wrist camera, §10.7).

**Tests**
| Test | How | Pass |
|---|---|---|
| T1.1 the arm responds | `curl -s localhost:8003/status` | `{"connected": true, "calibrated": true}` |
| T1.2 read the pose | `curl -s localhost:8003/read` | six angles in degrees |
| T1.3 topics are separate | `ros2 topic list \| grep joint` | `/joint_states`, `/arm2/joint_states`, no duplicates overwritten |
| T1.4 rates | `ros2 topic hz /arm2/joint_states` | ~50 Hz, no gaps > 100 ms |
| T1.5 TF tree | `ros2 run tf2_ros tf2_echo base arm2_base` | translation `[0.000, 0.300, 0.000]`, constant |
| T1.6 frames are prefixed | `ros2 run tf2_tools view_frames` | `arm2_base → arm2_shoulder → …`, no `arm2_*` collisions with arm 1 |
| T1.7 arm 1 is untouched | `ros2 topic hz /joint_states`, `/read` on 8002 | still 50 Hz and answering |
| T1.8 no cross-talk | move arm 2 by hand, watch `/joint_states` | arm 1's topic does not move |

---

## Phase 2 — The second wrist camera

**Work**
- `wrist_cam_node.py`: `WRIST_TOPIC_NS` (default `/gripper`), `WRIST_FRAME`, `WRIST_DEVICE` from env.
  Second instance in `so100-cams` with `/dev/arm2_wrist`, publishing
  `/arm2/gripper/image/compressed`, frame `arm2_gripper_color_optical_frame`.
- Second static transform: `arm2_moving_jaw_so101_v1 → arm2_gripper_color_optical_frame`
  (same six nominal numbers as arm 1 until measured).

**Tests**
| Test | How | Pass |
|---|---|---|
| T2.1 both cameras publish | `ros2 topic hz /gripper/image/compressed /arm2/gripper/image/compressed` | ~10 Hz each, simultaneously |
| T2.2 both frames resolve | `tf2_echo arm2_base arm2_gripper_color_optical_frame` | a pose, not "frame does not exist" |
| T2.3 bandwidth holds with ROS running | `tools/two_cam_bandwidth.py` while both nodes run | ≥9 fps each, 0 failed reads (the measured budget: §3.5) |
| T2.4 a camera unplugged does not sink the other | `docker exec so100-cams python3 …` with one device removed | the surviving node keeps publishing at 10 Hz |

---

## Phase 3 — Cams tab: pick the camera (requirement 1)

**Work**
- Rename the tab **Wristcam focus test → Cams**.
- `wrist_focus.py`: subscribe to **both** topics; a selector (`/select?arm=1|2`, and `?arm=` in the
  URL) decides which one is streamed and scored; `/status` reports `arm`, `topic`, `score`, `peak`.
- CSV gains an `arm` column; rows are tagged with the arm and the camera.
- Keep the on-demand behaviour (start on entering the tab, stop on leaving, 45 s watchdog) —
  now it starts **one** subscriber, the selected one, not both.
- The page shows two buttons (`Arm 1` / `Arm 2`) and which topic is being watched.

**Tests**
| Test | How | Pass |
|---|---|---|
| T3.1 the selector switches the image | click `Arm 2` | the stream changes, `/status` says `arm=2` and the topic is `/arm2/gripper/...` |
| T3.2 the score follows the selection | point arm 1 at text and arm 2 at a wall, switch | score high on 1, low on 2, no stale values carried over |
| T3.3 marks land in the right rows | mark 12.5 cm on each arm | two CSV rows, `arm` column = 1 and 2 |
| T3.4 on-demand still holds | leave the tab, then `curl :8093/status` | `running: false`, both subscribers gone (`ros2 node list`) |
| T3.5 the deep link works | open `…/?tab=focus&arm=2` (or the tab's own `?arm=2`) | opens straight into arm 2 |

---

## Phase 4 — RViz with both arms 30 cm apart (requirement 2)

**Work**
- `rviz.rviz`: a second RobotModel display, `Description Topic: /arm2/robot_description`,
  `TF Prefix: arm2_`, and the same colours; keep `Fixed Frame: base`, which already sees
  `arm2_base` through the static transform.
- The second TF (arm2_*) comes from the arm 2 `robot_state_publisher`, not from RViz.

**Tests**
| Test | How | Pass |
|---|---|---|
| T4.1 two models, one scene | open the RViz tab | two arms visible, no overlapping meshes |
| T4.2 the separation is 30 cm | measure with the grid (1 m) and with `tf2_echo base arm2_base` | 0.300 m in the same axis, consistently |
| T4.3 both are live | move each arm by hand | each model follows its own arm, no jitter or swap |
| T4.4 no frame collisions | `view_frames` | `arm2_` prefix everywhere in arm 2's branch |
| T4.5 CPU stays sane | `docker stats so100-web` while RViz runs | the same order as before (one arm), no runaway |

---

## Phase 5 — F710: hold A to change arms (requirement 4)

**Work**
- `f710_teleop_ws.py`: an `ARMS` list with `(name, targets_topic, states_topic)`; a `active` index.
- Hold **A for 5 s** → switch. Implementation details that matter:
  - a 5 s hold needs a longer edge detector than `_edge()`; measure the press duration and fire once
    at 5 s (not on release), then require a release before the next switch;
  - **do not switch blind**: request the new arm's `/joint_states` first; if no feedback within 1 s,
    refuse the switch, rumble twice and stay on the current arm;
  - on switching: stop publishing to the old topic, re-seed the targets from the new arm's real pose
    (no jump), clamp the first delta, then resume;
  - rumble: one short buzz = switched to arm 1, two = arm 2, a long one = refused;
  - the web page shows the active arm (a small badge next to the torque button, fed by the client
    over the same rosbridge — or by the client printing it and the page reading a topic).
- Torque policy: **each arm keeps its own torque state**; switching does not energise anything.
  If the newly selected arm has torque off, the client refuses to publish (as it does today for arm 1).

**Tests**
| Test | How | Pass |
|---|---|---|
| T5.1 a short A does nothing | tap A | no switch, no message published, no rumble |
| T5.2 exactly 5 s switches | hold A 5 s | one switch, one rumble, arm 2 now moves with the sticks |
| T5.3 it cannot switch twice from one hold | keep A held 12 s | exactly one switch |
| T5.4 only the active arm moves | with arm 2 active, watch `/joint_states` | arm 1 static, arm 2 follows |
| T5.5 no jump on switch | hold A while pushing the stick | the new arm picks up from its own pose, movement ≤ one tick |
| T5.6 refusal when the other arm is dead | unplug arm 2's adapter, hold A | long rumble, stays on arm 1, `arm2` never receives `/arm2/joint_targets` |
| T5.7 torque is per arm | arm 1 energised, arm 2 off, switch | arm 2 does not move; the client reports "torque off" |
| T5.8 a lost pad stops everything | unplug the pad while moving | no topic is published after 0.5 s (existing behaviour, re-verified for both arms) |

---

## Phase 6 — MuJoCo with both arms 30 cm apart (requirement 3)

**Work**
- `tools/make_two_arm_scene.py`: generate a `scene_two_arms.xml` from the Menagerie arm model —
  one instance at the origin and a second **renamed `arm2_*`** and placed 0.30 m away (the same
  include cannot appear twice without renaming its bodies/joints).
- `mujoco_twin.py`: load that scene, subscribe to both `/joint_states` and `/arm2/joint_states`,
  map each to its own set of joint indices, and draw both.

**Tests**
| Test | How | Pass |
|---|---|---|
| T6.1 both arms in one view | run the twin | two models, visibly 30 cm apart |
| T6.2 the separation matches the cell | measure with `mujoco` coordinates and compare with `tf2_echo base arm2_base` | 0.300 m in both |
| T6.3 both mirror their arm | move arm 1, then arm 2 | each model follows only its own arm |
| T6.4 no joint swap | move `shoulder_pan` on arm 2 | only arm 2's shoulder turns |
| T6.5 the twin survives one arm down | stop arm 2's container | the twin keeps drawing arm 1 and logs the missing one |

Status 2026-11-14: **T6.1 and T6.2 done and measured** — the generated `scene_two_arms.xml` compiles
(`nq=12`, `nu=12`) and its two gripper bodies are 0.3000 m apart, the same number `tf2_echo base
arm2_base` reports; the twin's joint map resolves 6/6 per arm (qpos 0–5, 6–11) and rosbridge delivers
both `/joint_states` and `/arm2/joint_states`. T6.3–T6.5 need the physical arm being moved and the
window open on the Windows PC. The generator had to be fixed first: `scene.xml` only *includes*
`so101.xml`, so the tool now resolves includes, keeps the default classes and asset references shared,
and duplicates the six actuators alongside the arm.

---

## Phase 7 — IK for the second arm, and the small things that follow

**Work**
- A second `ik` instance: `ARM_NS=/arm2`, base frame `arm2_base`, topics `/arm2/ik/*`.
- Second `so100-web`-side pieces: nothing mandatory; the tabs already reach both arms through ROS.

**Tests**
| Test | How | Pass |
|---|---|---|
| T7.1 round trip per arm | `tools/ik_selftest.py` against each instance | median ≤ 1 mm, p95 ≤ 5 mm, on both |
| T7.2 the base frame is right | target on arm 2 in `arm2_base` | the arm reaches it (visually, ±1 cm) |
| T7.3 unresolvable is reported | 2 m target on arm 2 | `/arm2/ik/status` says it did not converge |

---

## Phase 8 — Regression on the whole cell

| Test | How | Pass |
|---|---|---|
| T8.1 every container healthy | `docker ps` + `docker logs --tail 20` | six (+`arm2`) up, no tracebacks |
| T8.2 the tabs all work | open each of the seven tabs | Terminal, Stereo Cam & Depth, Point Cloud, Cams (both arms), Xavier NX Temp, Graph, RViz |
| T8.3 Graph shows the new nodes | Graph tab | `/arm2/*` nodes and topics present |
| T8.4 temperatures | Xavier NX Temp under load | no zone near 95 °C; note the delta vs one arm |
| T8.5 USB | `lsusb -t` | OAK `5000M` alone; hub `480M` with four devices |
| T8.6 the 1.0 features still work | IK selftest, cloud, teleop, shutdown, torque | unchanged |
| T8.7 restart the host | reboot | all containers come back, the four symlinks are stable, both arms respond |

---

## Order, and what can run in parallel

1. **Phase 0** (naming) — blocks everything; ~1 h with a reboot.
2. **Phases 1 and 2** (arm 2 + camera 2) — sequential, they share the naming; ~half a day.
3. **Phase 3** (Cams tab) and **Phase 4** (RViz) — parallel, different files; ~half a day.
4. **Phase 5** (F710) — needs 1 and 2; ~half a day, mostly careful state handling.
5. **Phase 6** (MuJoCo) — independent of 5, needs 1; the XML renaming is the fiddly part.
6. **Phase 7** (IK) and **Phase 8** (regression) — the close.

## Risks worth naming now

- **Two identical adapters.** The serial rules are the only reliable key; if a rule is wrong the
  container talks to the wrong arm and the failure looks like a motor timeout.
- **TF collisions.** Any frame without the `arm2_` prefix silently merges the two trees.
- **The 30 cm.** It lives in exactly one place (the `base → arm2_base` static transform) and the twin
  reads it from the same number; if the two drift apart, the comparison with MuJoCo is meaningless.
- **Teleop safety on switch.** The dangerous moment is the transition: stop publishing, re-seed,
  clamp, and refuse when the far arm is not answering.
- **The Xavier's stability.** It dropped three times in an hour on 2026-11-11/13 with the board at
  50 °C. Nothing in this plan fixes that; the temperature tab is there to watch it under two arms.

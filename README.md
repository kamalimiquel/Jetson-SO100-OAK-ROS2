# Jetson-SO100-OAK-ROS2

Robotic manipulation cell: an **SO-100 arm** + **OAK-D POE camera (USB-C)** + an **OAK-D Lite wrist camera**, governed by a **Jetson Xavier NX 16 GB** (Ubuntu 20.04, JetPack 5.1.5) running **ROS2 Humble in Docker containers**.

A Windows 11 Home PC on the same LAN provides visualization (noVNC in Chromium), GraspGen inference, policy training (MuJoCo / Isaac Sim) and teleoperation with a Logitech F710.

![Data flow](docs/architecture-flow.png)

## Status

- [x] Phase 0: repo + engineering prompt (`docs/PROMPT.md`)
- [x] Phase 1: ROS2 Humble in containers, FastDDS, domain 42 (ADR-0001, ADR-0002)
- [x] Phase 2: Docker — per-domain containers, zero-copy, `--ipc=host` (ADR-0003…0005)
- [x] Phase 3: drivers — LeRobot (arm), depthai-core (both OAK-Ds) (ADR-0006…0008)
- [x] Phase 4: control — joint-space API, mounting, calibration, recording, GraspGen (ADR-0009…0013)
- [x] Phase 5: RViz — headless RViz2 + noVNC (ADR-0014)
- [x] Phase 6: Windows — noVNC in Chromium, MuJoCo+Isaac Sim, F710 C++ client (ADR-0015…0017)
- [x] Phase 7: web interface — noVNC tabs + ttyd (ADR-0018)
- [x] Phase 8: architecture doc + diagram, v0.1 tag
- [ ] Implementation on the hardware (see `docs/architecture.md` → open items)

## Implementation status

- [x] Arm: LeRobot SDK (`~/lerobot/.venv`, py3.12) + calibration on the Xavier; all 6 motors verified over the bus (ADR-0006 amended — no adapter flash needed)
- [x] depthai-core from source on the NX (ADR-0007) — core v2.33.0 + bindings installed on the Xavier; USB3 + wrist-Lite wiring check pending
- [x] Compose file + provisioning script for the **six** services (five cell domains + rosbridge)
- [x] MuJoCo twin live: rosbridge container on ws://:9090 (jammy image, ADR-0019); Windows twin mirrors /joint_states at 50 Hz — verified end-to-end 2026-11-08 (398 msgs / 8 s from a LAN PC)
- [~] Web interface: tabs page + ttyd live; RViz2 is built from source in the web image (no humble debs on focal) — the image build is in progress, see "Web image build (RViz2 from source)" (ADR-0014/0015/0018)
- [ ] ChArUco board + easy_handeye for both cameras (ADR-0011)
- [ ] Windows: ROS2 Humble + VS Build Tools + SDL2 + firewall, F710 client, GPU check (ADR-0015…0017)

## Jetson setup — uv + LeRobot SDK (arm)

Exact commands to install and use the LeRobot stack for the SO-100 arm on the Xavier NX (ADR-0006). Verified working; doubles as the record of the Jetson's current state.

**1. Install uv**

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Lands in `~/.local/bin/uv`. The installer script needs `curl`, which the full JetPack 5.1.5 image ships.

**2. Python 3.12 venv**

```bash
uv venv ~/lerobot/.venv --python 3.12
```

uv fetches a managed CPython 3.12 build — the system Python 3.8 on Ubuntu 20.04 is too old for LeRobot (needs ≥3.10).

**3. LeRobot + Feetech SDK**

```bash
uv pip install --python ~/lerobot/.venv/bin/python "lerobot[feetech]"
```

Pulls `lerobot` with `torch` (CPU aarch64 wheel — multi-GB download), `rustypot` (serial motor bus), `feetech-servo-sdk` and `opencv-python-headless`. If `/tmp` is tight, `export TMPDIR=~/tmp` first so pip unpacks on disk.

Note: `uv venv` creates no `pip` binary — always install through `uv pip install --python <venv>/bin/python`, never `<venv>/bin/pip`.

**4. Serial access to the arm**

The arm's bus adapter enumerates as `/dev/ttyACM0` (VID `1a86`, CH9102). Default udev rules create `ttyACM*` as `root:dialout` with mode 0660 — membership in the `dialout` group is all that's needed; no custom udev rule.

**5. Verify the bus**

```bash
~/lerobot/.venv/bin/python tools/lerobot_bus_probe.py /dev/ttyACM0
```

Expected output: one line per motor with its ID and raw position, then `OK 6/6`. Exit code 0 means the whole chain (wiring, 12 V supply, baudrate, motor IDs) is healthy.

The one-time `lerobot-setup-motors` step (writes unique IDs 1–6 + 1 M baud into each servo's EEPROM) is only needed when IDs don't match; this arm came with them already set.

**6. Calibration (physical, once per arm)**

```bash
~/lerobot/.venv/bin/lerobot-calibrate \
    --robot.type=so100_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=so100_cell
```

Two interactive steps with torque off — the operator must hold the arm:
1. pose every joint mid-range, Enter;
2. sweep every joint through its full range, Enter.

The result is written to `~/.cache/huggingface/lerobot/calibration/robots/so_follower/so100_cell.json`. Without it, normalized reads raise `has no calibration registered` (raw reads with `normalize=False` still work).

**Current state on the Xavier:** uv 0.13.7 in `~/.local/bin`; venv `~/lerobot/.venv` (CPython 3.12.16, lerobot + torch 2.15); arm calibrated as `so100_cell` (2026-11-07).

## Camera setup — depthai on the Xavier

**State:** scene OAK-D POE verified capturing color 1080p + depth 720p (on a USB2 link at the time — ~2.3 fps; USB3 wiring pending). Wrist OAK-D Lite pending connection. `depthai-core` v2.33.0 built from source and installed at `~/depthai`; `depthai-python` bindings built against it and installed into `~/oak/.venv` (version string `2.33.0.0.dev0+<commit>` — the source build, not the wheel).

1. **udev rule (once):** `/etc/udev/rules.d/80-oak.rules` → `SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"`, then `udevadm control --reload-rules && udevadm trigger --subsystem-match=usb`. Without it, a bootloader-state device can't be opened by non-root.
2. **Fallback venv (works today):** `uv venv ~/oak/.venv --python 3.12` + `uv pip install --python ~/oak/.venv/bin/python "depthai<3" opencv-python-headless` (wheel 2.33.0.0 — same API as the source build).
3. **Verify:** `~/oak/.venv/bin/python tools/oak_cams_probe.py` — lists every device (MXID, USB protocol) and captures color+depth per device into `~/oak_caps/`.
4. **depthai-core from source:** clone `--recurse-submodules --branch v2.33.0`, cmake with `-DCMAKE_POSITION_INDEPENDENT_CODE=ON -DDEPTHAI_BUILD_EXAMPLES=OFF -DDEPTHAI_BUILD_TESTS=OFF -DDEPTHAI_ENABLE_BACKWARD=OFF`, install to `~/depthai` — restores headers + `depthaiConfig.cmake` (the pip wheel ships neither), unblocking C++ nodes later.
5. **Bindings from source:** `depthai-python` tag v2.33.0.0, default add_subdirectory mode, with its PIC toolchain line changed to `set(CMAKE_POSITION_INDEPENDENT_CODE ON CACHE BOOL "" FORCE)` (the plain `set()` never reaches Hunter's external projects), Python pointed at the uv 3.12 headers, and a clean `~/.hunter` before configuring. **Install:** copy the built `depthai.cpython-312-aarch64-linux-gnu.so` over the wheel's module in `site-packages/`. The module statically links depthai-core (self-contained) and only needs libusb at runtime via rpath into `~/.hunter` — keep that tree on the host.
6. **USB3 check:** both OAKs must appear under a 5000M hub in `lsusb -t`; ~2–3 fps at 1080p+720p means the link fell back to USB2.

## Container stack — how the Xavier runs the robot

The Xavier runs the whole cell as **six Docker containers, one per job** (ADR-0003, ADR-0019). A container is a little boxed Linux with exactly what its job needs — nothing more. All six are started by docker compose, they share the Xavier's network and memory with the host (host network + `ipc: host`), and they talk to each other in ROS2 on domain 42 with FastDDS.

| Container | Job | What's inside |
|---|---|---|
| `so100-arm` | talks to the arm | the LeRobot SDK (shared read-only from the host venv) + the arm's USB port (`/dev/ttyACM0`) + its calibration file |
| `so100-cams` | reads both OAK-D cameras | its own depthai SDK (wheel, py3.12) + the computer's USB ports, so it can open the cameras |
| `so100-perception` | will turn images into grasp requests | plain ROS2 Humble image + a shared workspace folder |
| `so100-control` | will run the joint API + episode recording | plain ROS2 Humble image + the same shared workspace |
| `so100-web` | the browser tabs (RViz, graph, terminal) | headless RViz2 (built from source) streamed via noVNC at :6080 + rqt_graph (:6081) + ttyd (7681) + tabs page (port 80) |
| `so100-rosbridge` | WebSocket bridge for the MuJoCo twin | Ubuntu 22.04 ROS2 Humble image with rosbridge_suite (binary packages) at ws://:9090 — it needs Python ≥3.9, which the focal images don't have (ADR-0019) |

### How to create the containers

**The short way — the provisioning script (recommended).** Copy the `docker/` and `scripts/` folders onto the Xavier (e.g. into `~/so100/`), then run:

```bash
SUDO_PASSWORD=<your password> bash scripts/provision-jetson.sh
```

The script is safe to run again and again. It: installs the compose v2 plugin if missing (into your user, no sudo), makes sure the camera udev rule exists, builds the two custom images (cams, web), starts all five containers, and writes the Xavier's address into the tabs page. When it finishes, all five containers are running — `docker ps` shows them. The tabs page is then at http://<xavier-ip>/ (port 80).

**The manual way — one container at a time.** From the `docker/` folder on the Xavier:

1. `so100-arm` — ready-made image, nothing to build:
   ```bash
   docker compose up -d arm
   ```
2. `so100-cams` — builds its own image first (installs uv + the depthai wheel, a few minutes), then starts it:
   ```bash
   docker compose up -d --build cams
   ```
3. `so100-perception` — ready-made image:
   ```bash
   docker compose up -d perception
   ```
4. `so100-control` — ready-made image:
   ```bash
   docker compose up -d control
   ```
5. `so100-web` — builds its own image (installs noVNC + ttyd, a few minutes), then starts it:
   ```bash
   docker compose up -d --build web
   ```
6. `so100-rosbridge` — builds its own image (pulls the Ubuntu 22.04 ROS2 image + installs rosbridge, a few minutes), then starts it:
   ```bash
   docker compose up -d --build rosbridge
   ```

**Day-to-day:** `docker ps` shows the six `so100-*` containers; the containers come back up by themselves after a Xavier reboot (`restart: unless-stopped`). Open a shell inside one with `docker exec -it so100-arm bash`. To use ROS2 commands inside a shell, source the ROS environment first: `bash -c "source /opt/ros/humble/install/setup.bash && ros2 topic list"`.

Verified 2026-11-08: talker→listener across arm→control; arm bus 6/6 inside its container; OAK capture 1/1 inside cams; tabs page + ttyd live; rosbridge on :9090 serving /joint_states to a LAN PC at 50 Hz.

Notes (traps we hit — read before changing the stack):
- Compose v2 is required — v1 rejects `device_cgroup_rules` with a misleading error.
- dusty images ship an expired ROS apt key: `rm /etc/apt/sources.list.d/*ros*` before `apt-get update` in new Dockerfiles.
- Docker auto-creates missing bind-mount folders root-owned — pre-create them (the provision script does).

## Web image build (RViz2 from source)

Ubuntu 20.04 (focal) has no ROS2 Humble binary packages, so RViz2 must be compiled inside the web image — a multi-hour build (OGRE vendor alone ~2 h on the Xavier's 4 cores). `docker/web.Dockerfile` builds rviz2 + rqt_graph + noVNC/ttyd in stages.

The build has two paths, controlled by the presence of a tarball:

- **Fast path (prebuilt workspace):** drop `docker/prebuilt/rviz_ws.tar.gz` into the deploy folder (git-ignored). The builder stage extracts it instead of compiling. To produce it, build once in a persistent container and package the workspace:
  1. `docker run --name rviz-builder <builder image with deps> ... colcon build --packages-up-to rviz2 --install-base /opt/rviz_ws`
  2. fix any failing package **inside** that container (incremental colcon — no full rebuild)
  3. `docker cp rviz-builder:/opt/rviz_ws - | tar -czf docker/prebuilt/rviz_ws.tar.gz ...`
- **From-source fallback:** with no tarball present, the same Dockerfile stage runs the full colcon build — the reproducible recipe.

**Known state (2026-11-08):** the full from-source run fails in `rviz_default_plugins` (the last big package before rviz2 itself); OGRE and everything up to rviz_common build clean. The web image has not been rebuilt with RViz2 yet — the currently deployed `so100-web` container still runs the pre-RViz build (tabs page + ttyd live, RViz tab pending). The rqt_graph and rosbridge additions ride the same rebuild.

## Windows PC setup — teleop + visualization

Visualization needs nothing but a browser (ADR-0015): open http://<xavier-ip>/ — tabs are RViz (headless RViz2 via noVNC) and a Terminal (ttyd); the RViz stream alone is at http://<xavier-ip>:6080/vnc.html?autoconnect=true. Teleoperation with the F710 needs the native client (ADR-0017).

### MuJoCo (training, ADR-0016) — first milestone

MuJoCo is already installed on the Windows PC. To verify the SO-100 model chain before any training, in a cmd/PowerShell:

1. Install MuJoCo for Python (if not already):
   pip install mujoco
2. Get the SO-ARM100 model from MuJoCo Menagerie (clone or download zip):
   git clone https://github.com/google-deepmind/mujoco_menagerie
3. Run the repo's check script against the model:
   python windows\mujoco_check.py C:\path\to\mujoco_menagerie\trs_so_arm100\scene.xml

Expected output: "OK - model loaded, 6 joints, simulated 200 steps". That proves the sim can run the arm — the base for imitation-learning sessions later (Isaac Sim deferred for now by user decision).

### MuJoCo twin — live mirror of the real arm (rosbridge WebSocket, ADR-0019)

The Xavier's `so100-rosbridge` container serves a WebSocket at ws://<xavier-ip>:9090 that forwards any ROS2 topic as JSON. The Windows PC mirrors the arm's /joint_states into MuJoCo:

1. Install the two Python libraries — **mind the versions** (MuJoCo 4.x moved the classic viewer into MuJoCo Studio; the twin uses the 3.x viewer):
   ```
   pip install roslibpy "mujoco>=3,<4"
   ```
2. Run the twin (any Menagerie SO-100-family model works; `trs_so_arm100` is the exact SO-100, `robotstudio_so101` the SO-101):
   ```
   python windows\mujoco_twin.py C:\path\to\mujoco_menagerie\trs_so_arm100\scene.xml
   ```

A MuJoCo window opens and the simulated arm follows the real one live. Internals, each of which cost debugging time on 2026-11-08:

- **Kinematic mirror**: the script uses `viewer.launch_passive` (NOT `viewer.launch`, which blocks the main thread and steps its own physics — the model's position servos then hold the arm frozen at the initial pose). Each frame the script writes qpos from the latest message, runs `mj_forward` (kinematics only) and `sync`s the viewer. No physics, no fighting the real arm.
- **Degrees → radians**: /joint_states publishes degrees (LeRobot convention); MuJoCo qpos is radians.
- **Joint-name mapping**: Menagerie names its joints Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw — not the LeRobot names. The script has an alias table plus a positional fallback, and prints the mapping (`qpos[0] <- shoulder_pan ...`) and the first received message so problems are visible in the console.
- **Viewer import**: `from mujoco import viewer` must be explicit — in the 3.x line `import mujoco` alone does not expose it.

The rosbridge server itself: rosbridge_suite humble needs Python >=3.9, which the focal cell images don't have — hence its own Ubuntu 22.04 container (docker/rosbridge.Dockerfile, official binary packages). It starts with `docker compose up -d rosbridge` and its health check is `ros2 topic echo /joint_states --once` inside the container. It needs `ipc: host` like the rest of the stack: FastDDS shared-memory transport carries the data over /dev/shm, and without the host IPC namespace the container discovers topics but receives nothing.

Verified 2026-11-08 from a LAN PC: 398 /joint_states messages in 8 s (~50 Hz) through the WebSocket.

Twin troubleshooting:

| Symptom | Cause | Fix |
|---|---|---|
| `RosTimeoutError: Failed to connect` | rosbridge down on the Xavier | `docker compose up -d rosbridge`; check `ss -tln \| grep 9090` on the Xavier |
| window opens, arm never moves | old script or `viewer.launch` (blocking) | `git pull` — the current script uses `launch_passive` |
| `mujoco ... has no viewer module` | MuJoCo 4.x installed | `pip install "mujoco>=3,<4"` |
| `mapped 0/6 joints` warning | unknown model joint names | use the Menagerie SO-100/101 models; the alias table + fallback cover them |
| a joint moves backwards | sign convention differs between model and arm | report it — a per-joint sign flip in the script fixes it |

**One-time setup:**
1. ROS2 Humble (runtime only): the official Windows zip (ros2-humble-*-win64) from docs.ros.org → unpack to C:\dev\ros2_humble.
2. Visual Studio Build Tools 2022 with the "Desktop development with C++" workload.
3. SDL2 dev: SDL2-devel-2.30.x-VC.zip from libsdl.org → C:\dev\SDL2.
4. F710: back switch to X (XInput), plug the receiver in.
5. Firewall (elevated prompt): run windows/firewall-allow.cmd — opens UDP 7400–7450 for DDS on the private profile (the classic silent-failure point).

**Build + run:** windows/build.cmd produces build\Release\f710_teleop.exe. Run it with the ROS2 environment active (call C:\dev\ros2_humble\local_setup.bat first).

**Controls:** joint-step mode by default — D-pad L/R selects joint 0–5, left stick Y steps it, LB/RB = gripper close/open. START toggles cartesian-feel (left stick = pan/lift, right stick = elbow/wrist, triggers = gripper). X records the trajectory, B replays it.

**Safety:** the client publishes nothing until /joint_states arrives and starts from the arm's real position; deltas are clamped per tick and targets per joint limits — the Jetson bridge enforces the real limits regardless.

**GPU check (Isaac Sim viability):** run windows/check-gpu.cmd — needs an NVIDIA RTX-class GPU and driver ≥ 535 for Isaac Sim.

## ROS2 nodes — what each container runs and publishes

### so100-arm — the arm

Three processes inside the container:

1. **LeRobot joint server** (Python 3.12) — *not* a ROS node. The only process touching the arm's serial port: it reads the 6 motor positions and applies commanded angles, exposed as plain HTTP inside the container (port 8002). It exists because the LeRobot SDK needs Python ≥3.10 while the container's ROS2 libraries are built for Python 3.8 — the two halves talk over HTTP. API: `GET /read` (angles JSON), `GET /status`, `POST /write` (body `{"angles": [...]}`).
   ```bash
   docker exec so100-arm curl -s http://127.0.0.1:8002/read
   ```
2. **arm_bridge** (Python 3.8 + rclpy) — the ROS2 translator around that server.
3. **robot_state_publisher** — computes the arm's 3D skeleton (TF) from the URDF and the joint angles. Humble reads the URDF from the `robot_description` **parameter** (passed at launch), not from the topic.

Topics of this container, by node:

| Topic | Node | Direction | What it carries |
|---|---|---|---|
| /joint_states | /arm_bridge | out, 50 Hz | the arm's real position: 6 joint names + angles in degrees |
| /joint_targets | /arm_bridge | in | desired position: 6 angles — publishing here moves the arm |
| /robot_description | /arm_bridge | out, latched | the arm's URDF (geometry description) |
| /tf | /robot_state_publisher | out, ~17 Hz | the 3D transform of every moving link |
| /tf_static | /robot_state_publisher | out | fixed transforms (offsets that never move) |

**Utility:** this container alone turns the arm into a standard ROS2 device — anyone on domain 42 can see where it is (/joint_states), command it (/joint_targets) and render it in 3D (URDF + /tf), without ever knowing a serial port exists.

### so100-cams — the cameras

One node (/oak_cams), one depthai pipeline per device:

| Topic | Direction | What it carries |
|---|---|---|
| /oak/scene/color/compressed | out | scene color as JPEG (CompressedImage) |
| /oak/scene/depth | out | scene stereo depth, 16UC1 (mm once factory calibration exists) |
| /oak/scene/confidence | out | stereo confidence map, 8UC1 (per-pixel depth certainty) |
| /oak/scene/color/camera_info, /oak/scene/depth/camera_info | out | intrinsics — appear when calibration exists (Phase 4 easy_handeye) |
| /oak/scene/imu | out | accel + gyro + rotation vector — only on units that carry an IMU (this POE unit has none) |
| /oak/scene/points | out | point cloud (XYZ, meters) — LIVE with **placeholder** OAK-D POE intrinsics from compose env; shape is right, absolute scale is NOT (this unit has no factory calibration). Tune OAK_SCENE_DEPTH_SCALE until Phase 4 replaces the placeholders |
| /oak/gripper/* | out | the same set for the wrist Lite, once it is plugged in |

It also broadcasts /tf_static with the camera frame offsets (placeholders until easy_handeye).

### so100-web — the browser

RViz2 (built from source in the image — no humble debs exist for focal) runs headless on Xvfb and is streamed via noVNC at :6080; rqt_graph at :6081; a ttyd terminal at :7681; tabs page at :80. No nodes of its own — RViz2 consumes /robot_description, /tf, /joint_states and the /oak topics. (Until the RViz2 image rebuild lands, the deployed container runs the pre-RViz build — see "Web image build".)

### so100-rosbridge — the WebSocket bridge (MuJoCo twin, ADR-0019)

| Node | What it does |
|---|---|
| /rosbridge_websocket | the WebSocket server at ws://:9090 — subscribes to any topic on request (e.g. /joint_states) and forwards it as JSON to LAN clients |
| /rosapi | ROS introspection over the same socket (list topics/services) |

Its topics (/client_count, /connected_clients) are bookkeeping only. Any client on the LAN (roslibpy on Windows, a browser, another robot) can now consume the whole cell over one WebSocket.

so100-perception and so100-control carry no nodes yet (grasp pipeline and joint API + rosbag2 come in later phases).

## Troubleshooting

Traps this project has actually hit, with their fixes — in one place:

| Symptom | Cause | Fix |
|---|---|---|
| ROS container: `apt-get update` fails with EXPKEYSIG | dusty images ship an expired ROS key | `rm /etc/apt/sources.list.d/*ros*` before `apt-get update` in new Dockerfiles |
| compose rejects `device_cgroup_rules` with a misleading error | compose v1 | install the compose v2 plugin (user-level, no sudo — the provision script does it) |
| bind-mount folder appears but is unwritable | docker auto-created it root-owned | pre-create `tools/` and `ws/` before `up` (the provision script does) |
| container sees topics (`ros2 topic list`) but receives no data | missing `ipc: host` — FastDDS shared-memory transport needs the host IPC namespace | add `ipc: host` to the container (all six have it) |
| rosbridge: `TypeError: unsupported operand type(s) for \|` | rosbridge_suite humble is Python ≥3.9 code, cell images are py3.8 | run rosbridge in its own Ubuntu 22.04 container (docker/rosbridge.Dockerfile) — never in the focal images |
| rosbridge builds: missing package (rosbridge_test_msgs / rosapi_msgs / ament_cmake_mypy) | the suite's packages depend on each other and on ament_lint | build ALL of the cloned repos (`colcon build` without `--packages-select`) |
| MuJoCo: `module 'mujoco' has no attribute 'viewer'` | 4.x moved the viewer to Studio; 3.x loads it lazily | `pip install "mujoco>=3,<4"` and `from mujoco import viewer` |
| MuJoCo twin: window opens, arm frozen | `viewer.launch` blocks and steps its own physics | `viewer.launch_passive` + own qpos/mj_forward/sync loop |
| MuJoCo twin: `mapped 0/6 joints` | Menagerie joint names differ from LeRobot | alias table + positional fallback in mujoco_twin.py |
| OAK device: `open()` fails in bootloader state | missing udev rule | `SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"` (the provision script adds it) |
| depthai link errors about non-PIC libs | Hunter pulled a prebuilt non-PIC dependency | force `CMAKE_POSITION_INDEPENDENT_CODE ON` in the toolchain + clean `~/.hunter` |
| OAK ~2–3 fps at 1080p+720p | USB2 link (480 M) instead of USB3 | check `lsusb -t` for a 5000M hub; move to a blue port |
| `pip install depthai` on the cell image | no cp38 wheels above 2.30.0.0 | pin `depthai==2.30.0.0` in the cams container |

## How we work

Every architecture decision is proposed with 3 alternatives (pros, cons, recommended), chosen by the user, and recorded as an ADR in `docs/adr/`. The full workflow lives in `docs/PROMPT.md`.

## Documentation

- `docs/PROMPT.md` — engineering prompt and project phases
- `docs/architecture.md` — architecture overview, ADR index, open items
- `docs/adr/` — Architecture Decision Records (0001–0019)
- `docs/architecture-flow.svg` / `.png` — data-flow diagram

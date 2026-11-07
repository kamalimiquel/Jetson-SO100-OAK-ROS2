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
- [ ] Compose file + provisioning script for the five services
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

The Xavier runs the whole cell as **five Docker containers, one per job** (ADR-0003). A container is a little boxed Linux with exactly what its job needs — nothing more. All five are started by docker compose, they share the Xavier's network and memory with the host (host network + `ipc: host`), and they talk to each other in ROS2 on domain 42 with FastDDS.

| Container | Job | What's inside |
|---|---|---|
| `so100-arm` | talks to the arm | the LeRobot SDK (shared read-only from the host venv) + the arm's USB port (`/dev/ttyACM0`) + its calibration file |
| `so100-cams` | reads both OAK-D cameras | its own depthai SDK (wheel, py3.12) + the computer's USB ports, so it can open the cameras |
| `so100-perception` | will turn images into grasp requests | plain ROS2 Humble image + a shared workspace folder |
| `so100-control` | will run the joint API + episode recording | plain ROS2 Humble image + the same shared workspace |
| `so100-web` | the browser tabs (RViz, rqt_graph, terminal) | noVNC + ttyd + the tabs page, served on port 80 |

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

**Day-to-day:** `docker ps` shows the five `so100-*` containers; the containers come back up by themselves after a Xavier reboot (`restart: unless-stopped`). Open a shell inside one with `docker exec -it so100-arm bash`. To use ROS2 commands inside a shell, source the ROS environment first: `bash -c "source /opt/ros/humble/install/setup.bash && ros2 topic list"`.

Verified 2026-11-07: talker→listener across arm→control; arm bus 6/6 inside its container; OAK capture 1/1 inside cams; tabs page + ttyd live.

Notes (traps we hit — read before changing the stack):
- Compose v2 is required — v1 rejects `device_cgroup_rules` with a misleading error.
- dusty images ship an expired ROS apt key: `rm /etc/apt/sources.list.d/*ros*` before `apt-get update` in new Dockerfiles.
- Docker auto-creates missing bind-mount folders root-owned — pre-create them (the provision script does).

## Windows PC setup — teleop + visualization

Visualization needs nothing but Chromium (ADR-0015): open http://<xavier-ip>:6080/vnc.html once the RViz stream is up. Teleoperation with the F710 needs the native client (ADR-0017).

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

## How we work

Every architecture decision is proposed with 3 alternatives (pros, cons, recommended), chosen by the user, and recorded as an ADR in `docs/adr/`. The full workflow lives in `docs/PROMPT.md`.

## Documentation

- `docs/PROMPT.md` — engineering prompt and project phases
- `docs/architecture.md` — architecture overview, ADR index, open items
- `docs/adr/` — Architecture Decision Records (0001–0018)
- `docs/architecture-flow.svg` / `.png` — data-flow diagram

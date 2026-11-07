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
- [ ] depthai-core from source on the NX (ADR-0007)
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

## How we work

Every architecture decision is proposed with 3 alternatives (pros, cons, recommended), chosen by the user, and recorded as an ADR in `docs/adr/`. The full workflow lives in `docs/PROMPT.md`.

## Documentation

- `docs/PROMPT.md` — engineering prompt and project phases
- `docs/architecture.md` — architecture overview, ADR index, open items
- `docs/adr/` — Architecture Decision Records (0001–0018)
- `docs/architecture-flow.svg` / `.png` — data-flow diagram

#!/usr/bin/env bash
# Provisioning for the Xavier NX — run ON the Jetson as the dialout+docker user.
# Idempotent. Requires the sudo password via SUDO_PASSWORD env (never in the repo):
#   SUDO_PASSWORD=... bash scripts/provision-jetson.sh
set -euo pipefail

sudo_cmd() { echo "${SUDO_PASSWORD:?set SUDO_PASSWORD first}" | sudo -S -p "" "$@"; }

echo "== docker =="
docker --version || { echo "docker missing: install docker.io + nvidia-container-toolkit first"; exit 1; }

if ! docker compose version >/dev/null 2>&1; then
  echo "== installing compose v2 plugin (user-level, no sudo) =="
  mkdir -p ~/.docker/cli-plugins
  curl -L -o ~/.docker/cli-plugins/docker-compose \
    https://github.com/docker/compose/releases/latest/download/docker-compose-linux-aarch64
  chmod +x ~/.docker/cli-plugins/docker-compose
fi
COMPOSE=(docker compose)

echo "== udev: arm serial symlinks (two-arm plan, Phase 0) =="
# The two CH9102 adapters are the same chip; only the serial tells them apart (VID:PID would
# match both). arm1 keeps the legacy so100_arm alias so the released cell stays unchanged; the
# second arm gets arm2_tty. A wrong or missing rule makes a container talk to the wrong arm and
# the failure looks like a dead motor — this naming is gated by the reboot test (Phase 0).
ARM_VID="${ARM_VID:-1a86}"
ARM_PID="${ARM_PID:-55d3}"
ARM1_SERIAL="${ARM1_SERIAL:-5971081400}"
ARM2_SERIAL="${ARM2_SERIAL:-5A46085595}"
if [ ! -f /etc/udev/rules.d/81-so100-arm.rules ]; then
  printf '%s\n' \
    "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$ARM_VID\", ATTRS{idProduct}==\"$ARM_PID\", ATTRS{serial}==\"$ARM1_SERIAL\", SYMLINK+=\"arm1_tty\", SYMLINK+=\"so100_arm\"" \
    "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$ARM_VID\", ATTRS{idProduct}==\"$ARM_PID\", ATTRS{serial}==\"$ARM2_SERIAL\", SYMLINK+=\"arm2_tty\"" \
    | sudo_cmd tee /etc/udev/rules.d/81-so100-arm.rules >/dev/null
  sudo_cmd udevadm control --reload-rules
  sudo_cmd udevadm trigger --subsystem-match=tty
  echo "arm udev rules added ($ARM1_SERIAL -> arm1_tty + so100_arm; $ARM2_SERIAL -> arm2_tty)"
else
  echo "already present"
fi

echo "== udev: wrist camera rules (ADR-0022 / two-arm Phase 0) =="
# The wrist cameras are plain UVC webcams; the symlinks make the device paths stable, so a
# re-enumeration to /dev/video6 does not break a container mount. arm1 keeps the so100_wrist
# alias; the second camera gets /dev/arm2_wrist. Set WRIST_PID/WRIST_VID for arm1 (default: the
# 5M WebCam, the one with a manual focus control) and WRIST2_PID for arm2 (its own VID:PID —
# if the unit reports a serial, key on WRIST2_SERIAL instead of the port). Until WRIST2_PID is
# set the arm2 rule is skipped and this prints a hint; run the script again once you know it.
WRIST_VID="${WRIST_VID:-0bda}"
WRIST_PID="${WRIST_PID:-5649}"
WRIST2_VID="${WRIST2_VID:-$WRIST_VID}"
WRIST2_PID="${WRIST2_PID:-}"
WRIST2_SERIAL="${WRIST2_SERIAL:-}"
if [ ! -f /etc/udev/rules.d/81-wrist-cam.rules ]; then
  {
    printf '%s\n' \
      "SUBSYSTEM==\"video4linux\", ATTRS{idVendor}==\"$WRIST_VID\", ATTRS{idProduct}==\"$WRIST_PID\", ATTR{index}==\"0\", SYMLINK+=\"arm1_wrist\", SYMLINK+=\"so100_wrist\""
    if [ -n "$WRIST2_SERIAL" ]; then
      printf '%s\n' \
        "SUBSYSTEM==\"video4linux\", ATTRS{idVendor}==\"$WRIST2_VID\", ATTRS{idProduct}==\"$WRIST2_PID\", ATTRS{serial}==\"$WRIST2_SERIAL\", ATTR{index}==\"0\", SYMLINK+=\"arm2_wrist\""
    elif [ -n "$WRIST2_PID" ]; then
      printf '%s\n' \
        "SUBSYSTEM==\"video4linux\", ATTRS{idVendor}==\"$WRIST2_VID\", ATTRS{idProduct}==\"$WRIST2_PID\", ATTR{index}==\"0\", SYMLINK+=\"arm2_wrist\""
    fi
  } | sudo_cmd tee /etc/udev/rules.d/81-wrist-cam.rules >/dev/null
  sudo_cmd udevadm control --reload-rules
  sudo_cmd udevadm trigger --subsystem-match=video4linux
  echo "wrist udev rules added (arm1_wrist + so100_wrist${WRIST2_PID:+ ; arm2_wrist})"
  [ -z "$WRIST2_PID" ] && [ -z "$WRIST2_SERIAL" ] && echo "  (hint: set WRIST2_PID/WRIST2_SERIAL and rerun to write /dev/arm2_wrist)"
else
  echo "already present"
fi

echo "== udev: OAK rule (VID 03e7) =="
if [ ! -f /etc/udev/rules.d/80-oak.rules ]; then
  printf '%s\n' 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
    | sudo_cmd tee /etc/udev/rules.d/80-oak.rules >/dev/null
  sudo_cmd udevadm control --reload-rules
  sudo_cmd udevadm trigger --subsystem-match=usb
  echo "oak udev rule added"
else
  echo "already present"
fi

echo "== /dev/shm (host tmpfs, shared via ipc:host) =="
df -h /dev/shm | tail -1

echo "== stack =="
cd "$(dirname "$0")/../docker"
# Pre-create bind-mount sources: docker would otherwise auto-create them
# root-owned and the deploy user could never write them again.
mkdir -p ../tools ../ws
"${COMPOSE[@]}" up -d --build

echo "== tabs page (self-configuring: derives the host from its URL) =="

echo "== terminal tab -> host shell secret =="
# The web container's ttyd (tab 'Terminal') SSHes to the host so the terminal shows the
# XAVIER, not the inside of so100-web (user request). It needs maju's password, read from a
# 600 file on the host (never in the repo). Supply it at deploy time, like SUDO_PASSWORD:
#   TERMINAL_SECRET=<maju password> SUDO_PASSWORD=... bash scripts/provision-jetson.sh
: "${TERMINAL_SECRET:?set TERMINAL_SECRET (=maju host password) so the terminal tab can SSH to the host}"
umask 177
printf '%s\n' "$TERMINAL_SECRET" > /home/maju/.terminal-secret
echo "written /home/maju/.terminal-secret (600)"

echo "== status =="
docker ps --format "table {{.Names}}\t{{.Status}}"

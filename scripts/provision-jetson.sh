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

echo "== udev: arm serial symlink =="
# The CH9102 adapter comes back as ttyACM1 whenever something else is plugged into the USB
# tree, leaving the arm container bound to a dead node. A symlink makes the path stable.
ARM_VID="${ARM_VID:-1a86}"
ARM_PID="${ARM_PID:-55d3}"
if [ ! -f /etc/udev/rules.d/81-so100-arm.rules ]; then
  printf '%s\n' "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$ARM_VID\", ATTRS{idProduct}==\"$ARM_PID\", SYMLINK+=\"so100_arm\"" \
    | sudo_cmd tee /etc/udev/rules.d/81-so100-arm.rules >/dev/null
  sudo_cmd udevadm control --reload-rules
  sudo_cmd udevadm trigger --subsystem-match=tty
  echo "arm udev rule added ($ARM_VID:$ARM_PID -> /dev/so100_arm)"
else
  echo "already present"
fi

echo "== udev: wrist camera rule (ADR-0022) =="
# The wrist camera is a plain UVC webcam; the symlink makes the device path stable, so a
# re-enumeration to /dev/video6 does not break the container mount. Set WRIST_VID/WRIST_PID to
# the unit you mount (default: the 5M WebCam, the one with a manual focus control).
WRIST_VID="${WRIST_VID:-0bda}"
WRIST_PID="${WRIST_PID:-5649}"
if [ ! -f /etc/udev/rules.d/81-wrist-cam.rules ]; then
  printf '%s\n' "SUBSYSTEM==\"video4linux\", ATTRS{idVendor}==\"$WRIST_VID\", ATTRS{idProduct}==\"$WRIST_PID\", ATTR{index}==\"0\", SYMLINK+=\"so100_wrist\"" \
    | sudo_cmd tee /etc/udev/rules.d/81-wrist-cam.rules >/dev/null
  sudo_cmd udevadm control --reload-rules
  sudo_cmd udevadm trigger --subsystem-match=video4linux
  echo "wrist udev rule added ($WRIST_VID:$WRIST_PID -> /dev/so100_wrist)"
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

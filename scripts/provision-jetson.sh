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

echo "== tabs page: fill LAN address =="
LAN_IP=$(hostname -I | awk '{print $1}')
sed -i "s/__HOST__/${LAN_IP}/g" web/index.html
"${COMPOSE[@]}" restart web

echo "== status =="
docker ps --format "table {{.Names}}\t{{.Status}}"

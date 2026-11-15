#!/usr/bin/env bash
# Calibrate one arm on the Xavier and leave a copy of the profile in docs/calibration/arms.
#
#   ./scripts/calibrate.sh arm1      # so100_cell   on /dev/arm1_tty
#   ./scripts/calibrate.sh arm2      # so100_cell_b on /dev/arm2_tty
#
# Why a wrapper: the port cannot be open twice, so the container that owns the arm has to be stopped
# first — and if the calibration fails, or you press Ctrl+C halfway, the container must come back up
# anyway. The trap does that. Run this on the Xavier, from anywhere: it finds the repo itself.
set -euo pipefail

case "${1:-}" in
  arm1) svc=arm;  port=/dev/arm1_tty; arm_id=so100_cell;;
  arm2) svc=arm2; port=/dev/arm2_tty; arm_id=so100_cell_b;;
  *) echo "usage: $0 arm1|arm2" >&2; exit 2;;
esac

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose="$repo/docker/docker-compose.yaml"
lerobot="${LEROBOT_PY:-$HOME/lerobot/.venv/bin/lerobot-calibrate}"

[ -x "$lerobot" ] || { echo "no lerobot-calibrate at $lerobot — set LEROBOT_PY" >&2; exit 1; }
[ -e "$port" ]    || { echo "$port is missing — is the arm plugged in? (udev: 81-so100.rules)" >&2; exit 1; }

echo "== stopping so100-$svc (the serial port must be free)"
docker compose -f "$compose" stop "$svc" >/dev/null
restore() {
  echo "== starting so100-$svc again"
  docker compose -f "$compose" start "$svc" >/dev/null
  sleep 6
  printf '%s' "   :80$( [ "$svc" = arm ] && echo 02 || echo 03 )/status -> "
  curl -s -m 6 "http://127.0.0.1:80$( [ "$svc" = arm ] && echo 02 || echo 03 )/status" || echo "(no answer yet — docker logs so100-$svc)"
  echo
}
trap restore EXIT

echo "== calibrating $arm_id on $port — move each joint through its full range when asked"
"$lerobot" --robot.type=so100_follower --robot.port="$port" --robot.id="$arm_id"

echo "== archiving the profile"
python3 "$repo/tools/archive_calibration.py" --id "$arm_id" --out-dir "$repo/docs/calibration/arms"

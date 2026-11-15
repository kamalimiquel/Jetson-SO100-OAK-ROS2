#!/usr/bin/env bash
# Open a bash inside any of the cell's containers, with the ROS environment already sourced.
#
#   ./scripts/shell.sh                 # list the containers
#   ./scripts/shell.sh arm             # bash in so100-arm (arm 1)
#   ./scripts/shell.sh arm2            # bash in so100-arm2
#   ./scripts/shell.sh web             # bash in so100-web
#   ./scripts/shell.sh control "ros2 topic list"   # run one command and leave
#
# All seven images ship bash (Debian-based ROS, plus the two build images), so `docker exec -it
# so100-arm bash` works as it is — what it does NOT give you is ROS on the path. That is the whole
# point of this wrapper: same shells, environment ready, one short name to type.
set -euo pipefail

names="arm arm2 cams perception control web rosbridge"

usage() {
  echo "usage: $0 [container] [command...]" >&2
  echo "  containers: $names" >&2
  exit 2
}

if [ $# -eq 0 ]; then
  echo "running:"
  docker ps --filter name=so100- --format '{{.Names}}\t{{.Status}}' | sed 's/^so100-/  /'
  echo
  echo "usage: $0 [container] [command...]   (e.g. $0 arm2)"
  exit 0
fi

name="$1"; shift
case "$name" in
  arm1) name=arm;;
esac
case " $names " in
  *" $name "*) ;;
  *) echo "no container called '$name'" >&2; usage;;
esac

c="so100-$name"
docker ps --format '{{.Names}}' | grep -qx "$c" || {
  echo "$c is not running — docker compose -f docker/docker-compose.yaml up -d $name" >&2
  exit 1
}

# ROS 2 lives in /opt/ros/humble inside the four ROS images; web and rosbridge have no ROS and the
# source is a no-op there, which is fine.
# ROS is not in the same place in every image: the dustynv L4T images keep the workspace at
# /opt/ros/humble/install/ (that "overlay install" is why `ros2` is not on the PATH until it is
# sourced), the stock ros:humble image at /opt/ros/humble/. Try all of them, quietly.
env_setup='for f in /opt/ros/humble/setup.bash /opt/ros/humble/install/setup.bash /opt/ws/install/setup.bash; do if [ -f "$f" ]; then . "$f" >/dev/null 2>&1; fi; done; export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}; export PS1="[${HOSTNAME%%.*} \W]\$ "'

# -t only when there is a terminal (a cron job or a pipe must not fail on "the input device is not a TTY")
flags=-i; [ -t 0 ] && flags=-it

if [ $# -eq 0 ]; then
  exec docker exec $flags "$c" bash -c "$env_setup; exec bash"
fi
exec docker exec $flags "$c" bash -c "$env_setup; $*"

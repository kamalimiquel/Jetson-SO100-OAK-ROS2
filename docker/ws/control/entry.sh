#!/bin/bash
# control container: the joint-space API lives in the arm container; this one carries the
# pieces that reason about poses instead. IK only reports unless IK_APPLY=1.
# Two-arm (Plan Phase 7): a second instance with ARM_NS=arm2 solves for arm 2 from its own
# arm2_base frame (topics /arm2/ik/*, /arm2/joint_targets when IK_APPLY=1).
#
# Two more nodes here, both from the same review chapter:
#   arbiter.py   the command contract and the sole writer to each arm's /write port. Observing by
#                default (ARBITER_ENFORCE=0): it validates every command and publishes what it
#                would do on /cell/arbiter while arm_bridge keeps writing. With 1 it claims each
#                arm's lease and becomes the only writer — set ARM_WRITE_MODE=arbiter on the arm
#                containers at the same time, or two processes drive one bus.
#   recorder.py  an episode under $RECORD_DIR: per-arm state with its own stamp, the frames the
#                policy sees, the commands. It refuses to start with RECORD_DIR empty.
set -e

# The interface package (ros/so100_cell_interfaces). Building it at every boot costs minutes on this
# CPU and delays IK, arbiter and recorder behind it, so the install tree is cached on the HOST (mounted
# at /opt/iface_cache) and rebuilt only when the message definitions change. The stamp is the hash of
# the sources: same hash, no colcon.
IFACE_CACHE=${IFACE_CACHE:-/opt/iface_cache}
if [ -d /opt/ros_src/so100_cell_interfaces ]; then
  HASH=$(find /opt/ros_src/so100_cell_interfaces -type f -print0 | sort -z | xargs -0 sha256sum \
         | sha256sum | cut -c1-16)
  if [ -f "$IFACE_CACHE/install/setup.bash" ] && [ "$(cat "$IFACE_CACHE/stamp" 2>/dev/null)" = "$HASH" ]; then
    source "$IFACE_CACHE/install/setup.bash"
    echo "interfaces: cached ($HASH) — so100_cell_interfaces sourced"
  else
    echo "interfaces: building $HASH (first boot after a change takes minutes on this CPU)"
    mkdir -p "$IFACE_CACHE/src" && rm -rf "$IFACE_CACHE/src/so100_cell_interfaces"
    cp -r /opt/ros_src/so100_cell_interfaces "$IFACE_CACHE/src/"
    if ( cd "$IFACE_CACHE" && colcon build --packages-select so100_cell_interfaces \
           --cmake-args -DBUILD_TESTING=OFF >"$IFACE_CACHE/build.log" 2>&1 ); then
      echo "$HASH" > "$IFACE_CACHE/stamp"
      source "$IFACE_CACHE/install/setup.bash"
      echo "interfaces: built and sourced"
    else
      echo "interfaces: BUILD FAILED — see build.log in the cache; IK falls back to untyped topics"
    fi
  fi
else
  echo "interfaces: no /opt/ros_src/so100_cell_interfaces — untyped topics only"
fi

# ADR-0023: both IK instances are namespaced, like the arms they solve for.
ARM_NS=arm1 python3 /opt/ws/control/ik_node.py &
ARM_NS=arm2 python3 /opt/ws/control/ik_node.py &
python3 /opt/ws/control/arbiter.py &
if [ -n "${RECORD_DIR:-}" ]; then
  python3 /opt/ws/control/recorder.py &
else
  echo "recorder: RECORD_DIR is empty — not recording (set it to capture an episode)"
fi
wait

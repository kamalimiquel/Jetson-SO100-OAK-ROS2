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

#!/bin/bash
# control container: the joint-space API lives in the arm container; this one carries the
# pieces that reason about poses instead. IK only reports unless IK_APPLY=1.
# Two-arm (Plan Phase 7): a second instance with ARM_NS=arm2 solves for arm 2 from its own
# arm2_base frame (topics /arm2/ik/*, /arm2/joint_targets when IK_APPLY=1).
set -e
python3 /opt/ws/control/ik_node.py &
ARM_NS=arm2 python3 /opt/ws/control/ik_node.py &
wait

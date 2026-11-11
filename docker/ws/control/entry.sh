#!/bin/bash
# control container: the joint-space API lives in the arm container; this one carries the
# pieces that reason about poses instead. IK only reports unless IK_APPLY=1.
set -e
python3 /opt/ws/control/ik_node.py

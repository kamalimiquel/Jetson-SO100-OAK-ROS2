"""MuJoCo sanity check for the Windows PC (ADR-0016, Phase 6b).

Loads the SO-ARM100 model from MuJoCo Menagerie and steps the simulator, so we
know the MuJoCo install + model chain works before any training.

Usage:
    python windows/mujoco_check.py <path-to-menagerie>\trs_so_arm100\scene.xml

Expected: "OK — model loaded, N joints, simulated 200 steps"
"""

import sys

import mujoco

MODEL_PATH = sys.argv[1] if len(sys.argv) > 1 else "trs_so_arm100/scene.xml"

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# Light touch: actuate nothing, just simulate gravity holding the arm.
mujoco.mj_resetData(model, data)
for _ in range(200):
    mujoco.mj_step(model, data)

print(f"OK - model loaded, {model.nq} joints, simulated 200 steps")
print(f"final qpos (first 6): {[round(float(q), 4) for q in data.qpos[:6]]}")

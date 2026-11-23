#!/usr/bin/env python3
"""Why is this arm not falling? Answer with torques, not with guesses.

`frictionloss` is a torque threshold at the joint: if the gravity torque on a link exceeds it, the joint must
move. So "the friction holds it" is not an opinion — it is |qfrc_bias| <= frictionloss for every joint, at the
pose the model is actually in. This prints exactly that, per joint, and says which of the four things is true:

    HELD BY FRICTION   the gravity torque does not reach the joint's frictionloss
    SHOULD MOVE        it does: the arm must sag, and if it does not, something else is holding it
    AT LIMIT           the joint sits on its range bound, so the limit stops it
    GRAVCOMP           the body carries gravcomp, which cancels gravity outright

    python3 sim/why_held.py --model scene_bench.xml [--prefix arm1_]
"""
from __future__ import annotations

import argparse
import sys

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--prefix", default="")
    args = ap.parse_args(argv)

    try:
        import mujoco
    except Exception as exc:                                        # noqa: BLE001
        raise SystemExit("why_held: mujoco is not importable here (%s)" % exc)

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    print("model: %s  nq=%d nu=%d  gravity=%s  timestep=%.4f"
          % (args.model, model.nq, model.nu, list(model.opt.gravity), model.opt.timestep))
    print("\n%-16s %9s %9s %11s %11s %9s %s" %
          ("joint", "qpos", "qvel", "gravity N.m", "friction", "spring N.m", "verdict"))
    any_should_move = False
    rows = 0
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if not name or not name.startswith(args.prefix):
            continue
        dof = int(model.jnt_dofadr[j])
        adr = int(model.jnt_qposadr[j])
        loss = float(model.dof_frictionloss[dof])
        stiffness = float(model.jnt_stiffness[j]) if hasattr(model, "jnt_stiffness") else 0.0
        springref = float(model.qpos_spring[adr]) if hasattr(model, "qpos_spring") else 0.0
        spring_tau = stiffness * (qpos - springref) if stiffness else 0.0
        bias = float(data.qfrc_bias[dof])
        qpos, qvel = float(data.qpos[adr]), float(data.qvel[dof])
        limited = bool(model.jnt_limited[j])
        lo, hi = (float(model.jnt_range[j][0]), float(model.jnt_range[j][1])) if limited else (0.0, 0.0)
        at_limit = limited and (qpos <= lo + 1e-6 or qpos >= hi - 1e-6)
        gravcomp = ""
        body = int(model.jnt_bodyid[j])
        if hasattr(model, "body_gravcomp") and float(model.body_gravcomp[body]) > 0:
            gravcomp = " (body gravcomp %.2f)" % float(model.body_gravcomp[body])
        if gravcomp:
            verdict = "GRAVCOMP"
        elif at_limit:
            verdict = "AT LIMIT"
        elif stiffness and abs(bias + spring_tau) <= loss:
            verdict = "HELD BY SPRING (k=%.1f N.m/rad, q_ref=%.3f)" % (stiffness, springref)
        elif abs(bias) <= loss:
            verdict = "HELD BY FRICTION"
        else:
            verdict = "SHOULD MOVE (|tau| > friction)"
            any_should_move = True
        rows += 1
        print("%-16s %9.4f %9.4f %11.4f %11.4f %9.3f %s%s" %
              (name[len(args.prefix):], qpos, qvel, bias, loss, spring_tau, verdict, gravcomp))

    if rows == 0:
        raise SystemExit("why_held: no joint name starts with %r — an empty table proves nothing. Drop "
                         "--prefix to see all of them (arm 1 has no prefix; arm 2 uses arm2_)."
                         % args.prefix)

    print("\n%s" % ("Verdict: at least one joint must sag under gravity — if the viewer shows it still, the "
                    "model is not the one being drawn or the world is not stepping." if any_should_move
                    else "Verdict: friction (or limits) can hold this pose, so a still arm is correct physics."))
    return 0


if __name__ == "__main__":
    sys.exit(main())

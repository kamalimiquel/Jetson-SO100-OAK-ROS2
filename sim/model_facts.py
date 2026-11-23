#!/usr/bin/env python3
"""What does raw MuJoCo do with this model? No server, no threads, no ports.

The question "why do the arms not fall" has been answered three times with a wrong guess (gravcomp, friction,
springs) because each guess looked at one mechanism. This asks the physics directly: load the model, print what
could hold a joint, integrate for a second and print what actually moved.

    python3 sim/model_facts.py --model scene_bench.xml [--seconds 1.0]

If qpos does not change here, the model holds itself and the server is innocent. If it does change here but not
in world_server, the fault is in the server's loop.
"""
from __future__ import annotations

import argparse
import sys


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--seconds", type=float, default=1.0)
    args = ap.parse_args(argv)

    try:
        import mujoco
    except Exception as exc:                                        # noqa: BLE001
        raise SystemExit("model_facts: mujoco is not importable here (%s)" % exc)

    m = mujoco.MjModel.from_xml_path(args.model)
    d = mujoco.MjData(m)

    flags = m.opt.disableflags
    print("model: %s" % args.model)
    print("  nq=%d nv=%d nu=%d  gravity=%s  timestep=%.4f  integrator=%d"
          % (m.nq, m.nv, m.nu, [round(float(g), 3) for g in m.opt.gravity], m.opt.timestep,
             int(m.opt.integrator)))
    print("  disableflags=%d (gravity off=%s, contact off=%s, limit off=%s)"
          % (int(flags), bool(flags & mujoco.mjtDisableBit.mjDSBL_GRAVITY),
             bool(flags & mujoco.mjtDisableBit.mjDSBL_CONTACT),
             bool(flags & mujoco.mjtDisableBit.mjDSBL_LIMIT)))

    print("\n%-18s %6s %8s %9s %9s %9s %9s %9s" %
          ("joint", "type", "range", "stiff", "damp", "armat", "fricloss", "qpos"))
    for j in range(m.njnt):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or "?"
        if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            continue
        dof, adr = int(m.jnt_dofadr[j]), int(m.jnt_qposadr[j])
        rng = "%.2f..%.2f" % (m.jnt_range[j][0], m.jnt_range[j][1]) if m.jnt_limited[j] else "free"
        print("%-18s %6d %8s %9.3f %9.3f %9.4f %9.3f %9.4f"
              % (name, int(m.jnt_type[j]), rng, m.jnt_stiffness[j], m.dof_damping[dof],
                 m.dof_armature[dof], m.dof_frictionloss[dof], d.qpos[adr]))

    q0 = d.qpos.copy()
    steps = int(round(args.seconds / m.opt.timestep))
    for _ in range(steps):
        mujoco.mj_step(m, d)
    delta = d.qpos - q0

    print("\nafter %.1f s (%d steps):" % (args.seconds, steps))
    moved = 0
    for j in range(m.njnt):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or "?"
        adr = int(m.jnt_qposadr[j])
        change = float(delta[adr]) if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE else 0.0
        if abs(change) > 1e-6:
            moved += 1
            print("  %-18s %+0.4f rad  (%+.2f deg)" % (name, change, change * 57.2958))
    print("\n%s" % ("VERDICT: %d joint(s) move in raw MuJoCo — the model does fall, so a still arm in the "
                    "server is the server's fault." % moved if moved else
                    "VERDICT: nothing moves in raw MuJoCo either — the model holds itself; the server is "
                    "innocent and the mechanism is in the XML."))
    return 0


if __name__ == "__main__":
    sys.exit(main())

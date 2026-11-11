"""Self-test for the IK node — no hardware, no rclpy spin.

Three checks:
  1. FK at the zero pose puts the jaw where the model says it is ([0.317, 0.018, 0.255] m,
     agreed with the MuJoCo twin when the URDF was generated).
  2. Round trip: random joint vectors inside the limits -> FK -> IK -> FK again. Measures how
     far the returned solution puts the tool from the target.
  3. An unreachable target must be reported as such, not silently "solved".

Run inside a container that has rclpy (so100-control):

    python3 /opt/tools/ik_selftest.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.environ.get("IK_DIR", "/opt/ws/control"))
from ik_node import Chain, solve, pose_error  # noqa: E402

LIMITS_DEG = [(math.degrees(lo), math.degrees(hi)) for lo, hi in Chain().limits]
TOL_MM = 2.0
TOL_DEG = 2.0


def main():
    chain = Chain()
    print("chain: %d joints, IK solves %d (%s) %s"
          % (chain.n, chain.dof, ", ".join(chain.links[:chain.dof]), chain.links[chain.dof:]))

    # 1 -- zero-pose jaw position
    T0 = chain.fk(np.zeros(chain.n))
    jaw = T0[:3, 3] * 1000.0
    print("FK at zero pose: jaw at [%.1f, %.1f, %.1f] mm" % tuple(jaw))
    ref = np.array([317.0, 18.0, 255.0])
    fk_err = float(np.linalg.norm(jaw - ref))
    print("  vs the documented model [317, 18, 255] mm -> %.1f mm %s"
          % (fk_err, "OK" if fk_err < 5.0 else "MISMATCH"))

    # 2 -- round trip
    rng = np.random.default_rng(7)
    pos_errs, ori_errs, joint_errs, worst = [], [], [], None
    n = 200
    for _ in range(n):
        q = np.array([rng.uniform(math.radians(lo), math.radians(hi))
                      for lo, hi in chain.limits[:chain.dof]])
        T = chain.fk(q, jaw=0.0)
        seed = np.array([0.0, -20.0, 40.0, 0.0, 0.0][:chain.dof])
        q2_deg, iters, pos_m, ori_deg = solve(chain, T, seed)
        T2 = chain.fk(np.radians(q2_deg), jaw=0.0)
        dp, w = pose_error(T2, T)
        pos_errs.append(np.linalg.norm(dp) * 1000.0)
        ori_errs.append(math.degrees(np.linalg.norm(w)))
        joint_errs.append(float(np.max(np.abs(np.degrees(q) - q2_deg))))
        if worst is None or pos_errs[-1] > worst[0]:
            worst = (pos_errs[-1], ori_errs[-1], iters, q2_deg)
    pos_errs = np.array(pos_errs)
    ori_errs = np.array(ori_errs)
    print("\nround trip over %d random poses:" % n)
    print("  position:  median %.2f mm  p95 %.2f mm  max %.2f mm"
          % (np.median(pos_errs), np.percentile(pos_errs, 95), pos_errs.max()))
    print("  orientation: median %.2f deg  max %.2f deg"
          % (np.median(ori_errs), ori_errs.max()))
    print("  joint space (how far the solution is from the pose that made the target): max %.1f deg"
          % max(joint_errs))
    print("  worst case: %.1f mm / %.1f deg in %d iterations" % worst[:3])

    # 3 -- unreachable
    far = np.eye(4)
    far[:3, 3] = [2.0, 0.0, 0.0]
    q3, it3, pos_m3, ori3 = solve(chain, far, np.zeros(chain.dof))
    ok = pos_m3 * 1000.0 <= TOL_MM
    print("\nunreachable target 2 m away: reported %.0f mm / %.0f deg -> ok=%s %s"
          % (pos_m3 * 1000.0, ori3, ok, "OK (correctly refused)" if not ok else "WRONG"))

    good = (fk_err < 5.0 and np.percentile(pos_errs, 95) < TOL_MM and not ok)
    print("\nSELFTEST %s" % ("PASS" if good else "FAIL"))
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())

# ADR-0025: the joint units are stated in config and asserted at boot

- Status: Accepted (the assertion and the audit table are implemented and verified on both arms;
  the external-reference column of the table is still empty — no instrument has been used yet)
- Date: 2026-11-19
- Decided by: the user, acting on the MCP specification §4 (joint and gripper measurements)

## Context

`SO100FollowerConfig` chooses the unit system of the five body joints:

- `use_degrees=True` → `MotorNormMode.DEGREES`: the value is an angle, converted with
  `(tick − mid) · 360 / 4095` around the midpoint of the joint's calibrated range.
- `use_degrees=False` → `MotorNormMode.RANGE_M100_100`: the value is a fraction of that range.

The jaw is always `RANGE_0_100` (percent). The server never passed the flag, so it ran on the SDK
default. Verified on the installed 0.7.0: the default *is* `True`, and all six motors report the
expected mode — but that is a property of a package version, not of this cell. Nothing in the code
would notice a change: `arm_bridge` copies the numbers, `deg2rad` multiplies by π/180, the model
draws, and every number downstream is wrong by a factor with no exception anywhere.

The same class of silent failure covers a reordered bus map (the array carries no names), a changed
motor id, and a package that drops the `normalize` keyword that the raw-tick audit depends on.

## Alternatives

1. **Leave the default alone and document it.** Cheapest; the audit table then has to trust a
   library default that a `pip install -U` can change.
2. **Alt 1 (this ADR)** — state `use_degrees=True` in the config, derive a *manifest* from the
   running SDK at boot (motor names, ids, models, normalisation modes, method signatures), compare it
   with an expectation kept in `docker/ws/lib/units_manifest.py`, and **refuse to start** on any
   mismatch. Also expose the resolved manifest and raw ticks over HTTP so the audit table can be
   produced without a second serial connection.
3. **Convert at every boundary** (store degrees in the cell, convert in each consumer). More code in
   more places, and the same question — "which unit is this?" — has to be answered at each of them.

## Decision

**Alt 1.** Concretely:

- `SO100FollowerConfig(..., use_degrees=True)` is written in the server. The default is no longer the
  thing that decides the unit.
- `units_manifest.JOINTS` holds the expectation: six joints in wire order, ids 1–6, model `sts3215`,
  `DEGREES` for the five body joints and `RANGE_0_100` for the jaw.
- At boot the server builds the observed manifest (`observed_from`) and runs `check()`. Errors are
  printed one per line and the process exits **2** before it binds the port; warnings (SDK version
  drift, an uninspected signature) are printed and do not stop it.
- `GET /manifest` returns the resolved manifest plus the calibration path and its sha256;
  `GET /read_raw` returns raw encoder ticks next to the SDK value for every motor.
- `tools/joint_units.py` writes the audit table (run **on the Xavier**: the servers bind loopback);
  `tools/jaw_curve.py` sweeps the jaw in both directions for the aperture curve.

## Consequences

- A wrong unit system, a reordered map or a changed id is a **bench that does not come up**, which is
  the failure mode we want: loud, early, and before any motion.
- Two more HTTP endpoints exist on a loopback port that already exposes `/read`; they are read-only
  and are named in the README's API list.
- The check runs in the LeRobot interpreter (py3.12) because that is where the bus lives; the ROS side
  is unchanged and keeps seeing the same numbers.
- A package upgrade now needs a deliberate edit of the manifest, i.e. a reviewed change rather than an
  inherited default.

## Verification

- `tools/test_units_manifest.py` — 18 checks: correct manifest passes; `use_degrees=False`, a body
  joint in the wrong mode, a jaw in degrees, a missing/extra motor, a reordered map, a changed id and a
  `read()` without `normalize` are each refused with a sentence; version drift warns only.
- `tools/test_joint_server.py` — 40 checks against the real handler, now including `/manifest`
  (degrees for the five, percent for the jaw, order as the contract) and `/read_raw` (ticks differ
  from the normalised value).
- `tools/check_namespaces.py` — asserts the wiring: `use_degrees=True`, `um.check(manifest)`,
  `raise SystemExit(2)`, and both endpoints present.
- **On both arms, 2026-11-19**: boot logs `units OK — sdk 0.7.0, degrees=True` and the two calibration
  shas (`0051a9d8…`, `d9cccea7…`). The audit table (`docs/qualification/baseline/2026-11-19-6d63efc/`)
  reproduces, for all 12 motors, `SDK value == DEGREES/RANGE formula(raw tick)` with **delta 0.000**,
  and `ROS value == SDK value` with **delta 0.000**.
- **External reference, 2026-11-19**: `shoulder_lift` slope **0.990** over 15.2 deg, `elbow_flex`
  **0.976** over 7.5 deg (fit on shoulder+elbow: the instrument on the forearm measures absolute
  inclination), about **1 deg of backlash** on reversal, and a joint that does not reach its command
  under load (-10 deg commanded arrived as -6.9 on the shoulder and -4.8 on the elbow).
- **Jaw, 2026-11-19**: `J(0 %) = 16 mm`, `1.346 mm/%` opening and `1.384 mm/%` closing over **0-50 %**,
  hysteresis **4 mm at 25 %**. Above ~50 % the fingers cross (the SO-100 jaw **rotates**, it is not
  parallel), so no millimetre aperture is claimed there — which also means `deg2rad`'s per-percent
  proxy is wrong in *structure*, not only in scale.
- Still not measured: `shoulder_pan`, `wrist_flex`, `wrist_roll`, repeatability, and any point under
  load. Until those are filled phase 1 is not signed off.

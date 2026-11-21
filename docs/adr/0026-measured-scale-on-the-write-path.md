# ADR 0026 — the measured scale is applied on the write path, opt-in and declared

**Status:** accepted, 2026-11-19. Applies to `lerobot_joint_server.py`, `docker/ws/lib/joint_scale.{py,json}`.

## Context

Phase 1 measured every joint against an instrument outside the robot and found that the SDK's degrees
are off by a *different* amount on each one: `shoulder_pan` 1.059 (5.9 % fast), `shoulder_lift` 0.990,
`elbow_flex` 0.976, `wrist_flex` 0.94, `wrist_roll` 0.877 (12.3 % short). A commanded 90° therefore
lands somewhere between 79° and 95° depending on the joint, and nothing raises: the number is finite, in
range, and the model agrees with itself.

The numbers are now evidence-backed (CSV per joint, in
`docs/qualification/baseline/2026-11-19-6d63efc/`), so the question is not whether to use them but
where, and how to keep them from becoming folklore.

## Decision

1. **The scale is data, not code.** `docker/ws/lib/joint_scale.json` carries the five angles and their
   uncertainty, the session and the evidence directory, and a `gripper` entry whose scale is **null**:
   a percentage is not an angle, and the jaw's millimetres live in a curve that is only valid below
   ~50 %. `joint_scale.py` validates the table (six joints, no extras, scale within 0.5–1.5, gripper
   null) and refuses anything else with a sentence. It also carries the arm it was measured on, and the
   server refuses to *apply* a table whose arm is not its own: arm 2 has its own calibration file and its
   own servos, so borrowing arm 1's scales would be a guess dressed as a measurement. Verified live —
   `JOINT_SCALE_APPLY=1` on arm 2 exits with that sentence.
2. **The conversion happens at the joint server's write path, before the range check.**
   `POST /write` with `{"units": "physical"}` takes degrees the caller means physically and divides by
   the scale of each joint; the SDK-range fuse is then applied to the number that actually goes on the
   wire. The contract is unchanged for the ROS topic: the arbiter keeps sending SDK values, because a
   topic cannot say which unit it carries.
3. **It is opt-in and announced.** `JOINT_SCALE_APPLY` defaults to `0`. Off, every number is SDK
   degrees, exactly as today, and asking for `"units": "physical"` is a 400 that says why. With it on,
   the boot log prints the table, its hash and `APPLIED`, `/scale` serves it, and `/manifest` records
   its sha256 — so a run can be tied to the calibration it used.
4. **Reads keep both.** `/read_raw` still returns the encoder tick and the SDK value, and adds
   `physical`, computed with the same table. Nothing is silently rescaled: a closed loop can use either
   as long as it says which.

## Consequences

- A policy or a script that commands physical degrees gets what it means, and the 12 % on `wrist_roll`
  stops being a hidden bias in every grasp.
- Turning it on **changes what a request means**. That is why it is a flag and not a default, and why
  the response echoes the conversion (`shoulder_pan 95.0 physical -> 89.7 sdk`) instead of hiding it.
- The table can go stale. `tools/test_joint_scale.py` fails if the numbers stop matching the CSVs they
  name — a cheap guard against a calibration drifting away from its evidence — but a re-calibration
  still has to be written down; the hash is what makes the drift visible.
- `wrist_roll`'s 0.877 is the least certain of the five (±0.03, one mounting, one direction of travel).
  It is applied like the others, and the uncertainty is in the file next to it.

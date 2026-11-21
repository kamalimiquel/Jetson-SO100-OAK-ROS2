# Qualification evidence — the acceptance ledger

This directory is the **acceptance ledger**: the record of what was measured, on which machine, with
which software, and what happened. It is deliberately **not** the demonstration recorder
(`docker/ws/control/recorder.py`, `RECORD_DIR=/episodes`): that one produces training episodes, this
one produces the answer to "may this configuration be accepted?".

They are kept apart for four reasons:

1. **Contamination** — an acceptance run must never become training data by accident.
2. **Deletion** — episodes are rotated when disk fills; evidence is not.
3. **Lifetime** — the recorder is optional and is switched off during qualification; the ledger exists
   for every attempt, including the ones that fail.
4. **Reader** — the ledger is read by a person (the independent scorer); the recorder by a training
   script.

## Layout

```
docs/qualification/
  README.md                         this file: rules and schema
  baseline/<date>-<sha>/            what the system was, frozen
    inventory.yaml                  host, images, devices, versions, resolved modes
    tests.txt                       the headless checks and their result at that SHA
    joint_units.csv                 the unit table (phase 1)
  runs/<RUN-ID>.json                one record per attempt, append-only
```

`RUN-ID` is `YYYYMMDD-<stage>-<nn>` (e.g. `20261119-BENCH-01`). Never reuse an ID, never edit a
record after it is written: append a new one and reference the old.

## Rules

- **Append-only.** A record that is wrong gets a new record that says so; nothing is deleted.
- **All started attempts count.** An aborted run, an operator takeover or a timeout is a recorded
  outcome, not a missing row.
- **No stitching.** A mission passes only if one run ID covers every stage.
- **Evidence vocabulary** (from the specification): *reviewed* = source inspected, *reproduced* = run
  here, *reported* = a dated project measurement, *proposed* = design or trial target. Every number in
  a record says which one it is.
- **The scorer is blind to the controller** where an A/B comparison is being scored.
- **Hashes, not names.** Config, calibration, adapter, weights and skill revisions are recorded by
  hash; "the current version" is not evidence.

## Run record schema

`runs/<RUN-ID>.json`:

| Field | Meaning |
|---|---|
| `run_id` | `YYYYMMDD-<stage>-<nn>` |
| `stage` | `BENCH`, `DOCK`, `MISSION`, `SIM-A/B`, `UNITS`, `HANDOFF`… |
| `started`, `ended` | ISO wall time, and monotonic duration in seconds |
| `operator`, `scorer` | who was at the bench, who scored |
| `robot` | serials of both arms, boot epoch, calibration hashes |
| `config` | hashes of the deployed configuration (images, adapter, skill, weights) |
| `layout` | layout ID, prop dimensions, positions, lighting, camera IDs |
| `stage_results` | per stage: `pass` / `fail` / `unverified` / `not_run`, with the reason code |
| `first_failure` | the first stage that did not pass, and its contributing factor |
| `interventions` | count and kind (takeover, clarification, stop) |
| `evidence` | relative paths of frames, encoder logs, snapshots kept for this run |
| `notes` | anything a reader needs; no interpretations of success |

## What never goes here

- Demonstration episodes (`/episodes`), raw bags, or anything the training pipeline consumes.
- Secrets, tokens or certificates.
- A conclusion without the numbers that support it.

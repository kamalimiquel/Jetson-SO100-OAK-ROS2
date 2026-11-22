"""One definition of the cell's epoch, shared by the supervisor and the MCP manifest.

It was written twice — the supervisor appended the calibration digest, the manifest prefixed it with the arm
name — and the two produced different strings for the same cell. The effect was not subtle: every request the
endpoint sent was refused with "epoch mismatch" against a cell that had not changed at all. Two copies of a
fingerprint are two fingerprints.

    epoch(boot_id, {"so100_cell": table, ...}) -> "c05d9607be6b9327"
"""
from __future__ import annotations

import hashlib


def epoch(boot_id, tables):
    """Fingerprint of (host boot, calibration revisions). Changes when either changes; equal means same cell."""
    if not boot_id:
        raise ValueError("epoch needs the host boot id — without it a restart is invisible")
    parts = [str(boot_id)]
    for arm in sorted(tables):
        parts.append("%s:%s" % (arm, tables[arm].get("sha256") or "no-sha"))
    return hashlib.sha256("|".join(parts).encode("utf-8")).hexdigest()[:16]


def boot_id(path="/proc/sys/kernel/random/boot_id"):
    try:
        with open(path) as fh:
            return fh.read().strip()
    except OSError:
        return "no-boot-id"

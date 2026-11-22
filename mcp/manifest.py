#!/usr/bin/env python3
"""What an agent is allowed to believe about this cell: the manifest behind `capabilities`/`get_state`.

The endpoint lives outside the containers, so it cannot ask ROS anything. It gets a small, explicit snapshot
and builds one document from it; the parts that must never be guessed (which robot, which calibration, what
a joint's scale and deadband are) come from the same files the arms load, so a manifest cannot describe a
robot that is not the one on the bench.

Stdlib only, no ROS, no sockets: `tools/test_manifest.py` runs it headless.

    from manifest import build
    m = build(identity={"robot": "bench-02", "serial": "so100_cell", "profile": "robot"},
              arms=[...], epoch=epoch("boot-1", tables), images_lock="abc123")

Two decisions worth knowing:

  - **The epoch is a fingerprint, not a counter.** It is derived from the host boot id and the calibration
    revisions, so it changes whenever either changes. An agent that planned against epoch E and asks with E
    after a restart is refused, which is the point.
  - **An unreachable arm is reported, not omitted.** A manifest that silently drops an arm is a manifest an
    agent will plan around.
"""
from __future__ import annotations

import hashlib
import json
import os
import sys

SCHEMA = "so100.cell.manifest/1"
PROFILES = ("robot", "sim")
SKILLS = ("move_joint", "move_pose", "park", "hold", "open_jaw", "close_jaw")


class ManifestError(ValueError):
    """The inputs do not describe a cell. The message says which field and what to fix."""


def epoch(boot_id, tables):
    """A fingerprint of (host boot, calibration revisions), computed by the cell's one implementation.

    Imported rather than written again: it *was* written twice, the two copies disagreed, and every request the
    endpoint sent was refused against a cell that had not changed.
    """
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docker", "ws", "lib"))
    import cell_epoch
    try:
        return cell_epoch.epoch(boot_id, tables)
    except ValueError as exc:
        raise ManifestError(str(exc))


def read_images_lock(path):
    """The digest of what is actually running, so a manifest can be tied to an image set."""
    if not os.path.exists(path):
        return None
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()[:16]


def _joints(table):
    """The physical facts an agent needs per joint, from that arm's own measured table."""
    out = {}
    for name, entry in (table.get("joints") or {}).items():
        row = {}
        if entry.get("scale") is not None:
            row["scale"] = entry["scale"]
            row["uncertainty"] = entry.get("uncertainty")
        dead = entry.get("deadband_deg")
        row["deadband_deg"] = dead            # None is "not measured", never 0
        if entry.get("curve"):
            row["curve"] = entry["curve"]
        out[name] = row
    return out


def build(identity, arms, epoch_value, images_lock=None, limits=None):
    """The one document an agent gets. Raises rather than inventing a field."""
    if not isinstance(identity, dict):
        raise ManifestError("identity must be an object")
    for key in ("robot", "serial", "profile"):
        if not identity.get(key):
            raise ManifestError("identity.%s is missing" % key)
    if identity["profile"] not in PROFILES:
        raise ManifestError("identity.profile must be one of %s" % (PROFILES,))
    if not epoch_value:
        raise ManifestError("an epoch is required: without it a restarted arm looks like the same one")
    if not isinstance(arms, list) or not arms:
        raise ManifestError("arms must be a non-empty list — a cell with no arm is not a cell")

    out_arms = []
    for arm in arms:
        if not isinstance(arm, dict) or not arm.get("arm"):
            raise ManifestError("each arm needs an 'arm' id")
        if not arm.get("table"):
            raise ManifestError("arm %r has no measured table: its scales would be a guess" % arm["arm"])
        out_arms.append({
            "arm": arm["arm"],
            "reachable": bool(arm.get("reachable")),
            "owner": arm.get("owner"),
            "torque": arm.get("torque"),
            "calibration_sha256": (arm["table"].get("sha256") or "")[:16],
            "session": arm["table"].get("session"),
            "joints": _joints(arm["table"]),
            "error": arm.get("error"),
        })

    return {
        "schema": SCHEMA,
        "identity": {"robot": identity["robot"], "serial": identity["serial"],
                     "profile": identity["profile"], "epoch": epoch_value},
        "images": {"lock_sha256": images_lock},
        "skills": list(SKILLS),
        "limits": limits or {},
        "arms": out_arms,
        "note": ("deadband_deg is the step below which that joint does not move and reports no error; "
                 "null means it was never measured, which is not zero"),
    }


def to_json(manifest):
    return json.dumps(manifest, indent=2, sort_keys=False)

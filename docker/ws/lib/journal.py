#!/usr/bin/env python3
"""The mission journal: what was asked, what was decided, and the ticket that came back.

Phase 4 of docs/design/mcp-endpoint-plan.md. An append-only JSONL file, one line per /skill
outcome. It is what makes a retry idempotent: an accepted mission_id is answered with its
original ticket instead of being written a second time, and what the agent is told happened is
what the file says happened.

    append(entry)                      one line
    read(mission_id=None, limit=50)    oldest first
    find_accepted(mission_id)          the accepted writing entry for that mission, or None
    last_for(mission_id)               the latest entry for that mission, or None

Nothing here talks to ROS or to an arm: it is a file and a few pure functions.
"""
from __future__ import annotations

import json
import os

# /episodes is mounted from the host (docker-compose `control`), so the journal survives the
# container and is readable from outside it. Without it, a writable /tmp fallback, resolved the
# same way for reads and writes — a split fallback would write entries nobody could read.
DEFAULT = "/episodes/journal.jsonl"
# Skills that change the world. solve_ik does not, so a retried report is not "already done".
WRITING = ("move_joint", "move_pose", "hold", "open_jaw", "close_jaw")


def path():
    return _open(os.environ.get("JOURNAL_PATH", DEFAULT))


def _open(p):
    try:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        return p
    except OSError:
        os.makedirs("/tmp/journal", exist_ok=True)
        return os.path.join("/tmp/journal", os.path.basename(p))


def append(entry, p=None):
    p = _open(p or path())
    with open(p, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(entry, sort_keys=True) + "\n")
        fh.flush()
    return entry


def read(p=None, mission_id=None, limit=50):
    p = p or path()
    try:
        with open(p, encoding="utf-8") as fh:
            lines = fh.readlines()
    except OSError:
        return []
    out = []
    for line in lines[-max(1, int(limit)) * 4:]:
        try:
            entry = json.loads(line)
        except ValueError:
            continue
        if mission_id and entry.get("mission_id") != mission_id:
            continue
        out.append(entry)
    return out[-max(1, int(limit)):]


def find_accepted(mission_id, epoch=None, p=None):
    """The entry that already wrote for this mission **in this epoch**, so a retry does not write twice.

    Scoped by epoch on purpose (spec 6-8, "mission_id idempotence + epoch"): after the cell restarts
    or is recalibrated, the same mission id is a new decision, not a replay of the old one. Answering
    it with a stale ticket would claim a motion that happened to a different robot.
    """
    if not mission_id:
        return None
    for entry in read(p, mission_id=mission_id, limit=200):
        if not entry.get("accepted") or entry.get("skill") not in WRITING:
            continue
        if epoch is not None and entry.get("epoch") and entry.get("epoch") != epoch:
            continue
        return entry
    return None


def last_for(mission_id, p=None):
    rows = read(p, mission_id=mission_id, limit=200)
    return rows[-1] if rows else None

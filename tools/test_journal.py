#!/usr/bin/env python3
"""The journal, without a cell: order, idempotence scope, and a missing file.

    python3 tools/test_journal.py
"""
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "docker", "ws", "lib"))
import journal as jr  # noqa: E402

ok = fail = 0


def check(label, got, want):
    global ok, fail
    if got == want:
        ok += 1
        print("ok   %-58s %r" % (label, got))
    else:
        fail += 1
        print("FAIL %-58s got %r want %r" % (label, got, want))


tmp = tempfile.mkdtemp()
p = os.path.join(tmp, "journal.jsonl")

check("a missing file reads as empty", jr.read(p), [])
check("and has no accepted mission", jr.find_accepted("m1", p), None)

jr.append({"mission_id": "m1", "skill": "move_joint", "accepted": True, "ticket": "t1"}, p)
jr.append({"mission_id": "m1", "skill": "solve_ik", "accepted": True, "ticket": "t2"}, p)
jr.append({"mission_id": "m2", "skill": "move_joint", "accepted": False, "ticket": ""}, p)
jr.append({"mission_id": "m3", "skill": "move_pose", "accepted": True, "ticket": "t3"}, p)

check("entries come back oldest first", [e["ticket"] for e in jr.read(p)], ["t1", "t2", "", "t3"])
check("a mission is answered from its own rows",
      [e["skill"] for e in jr.read(p, mission_id="m1")], ["move_joint", "solve_ik"])
check("the accepted write is the one found", jr.find_accepted("m1", p)["ticket"], "t1")
check("a refused mission has nothing accepted", jr.find_accepted("m2", p), None)
check("last_for gives the latest row", jr.last_for("m1", p)["skill"], "solve_ik")
check("last_for of an unknown mission is None", jr.last_for("nope", p), None)

# A report-only solve is not a write: it must not make a later motion look "already done".
jr.append({"mission_id": "m4", "skill": "solve_ik", "accepted": True, "ticket": "t4"}, p)
check("solve_ik alone does not block a retry", jr.find_accepted("m4", p), None)
jr.append({"mission_id": "m4", "skill": "move_joint", "accepted": True, "ticket": "t5"}, p)
check("...but the motion does", jr.find_accepted("m4", p)["ticket"], "t5")

# The limit keeps the tail, not the head.
for i in range(60):
    jr.append({"mission_id": "many", "skill": "move_joint", "accepted": True, "ticket": "m%d" % i}, p)
check("read keeps the newest rows", jr.read(p, mission_id="many", limit=3)[-1]["ticket"], "m59")
check("read caps the count", len(jr.read(p, mission_id="many", limit=3)), 3)

print("\n%s" % ("all checks pass" if not fail else "%d check(s) FAILED" % fail))
sys.exit(1 if fail else 0)

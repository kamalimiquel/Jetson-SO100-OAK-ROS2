#!/usr/bin/env python3
"""The virtual arm's contract, checked with a fake model — physics is not needed to test an interface.

Everything here is a rule the real arm also enforces, and every one of them failing silently would be worse
than a wrong number: a write without the lease, a write with torque off, five values with no `hold`, a NaN.

    python3 tools/test_sim_arm.py
"""
import json
import os
import sys
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim"))
import arm_server as sim  # noqa: E402

ok = fail = 0


def check(label, got, want):
    global ok, fail
    if got == want:
        ok += 1
        print("ok   %-56s %r" % (label, got))
    else:
        fail += 1
        print("FAIL %-56s got %r want %r" % (label, got, want))


class FakeModel:
    actuated = True
    model = type("M", (), {"nq": 6})()

    def __init__(self):
        self.last = {}

    def present_deg(self):
        return {j: 0.0 for j in sim.JOINT_ORDER}

    def command_deg(self, angles):
        self.last = dict(angles)
        return dict(angles)


fake = FakeModel()
state = sim.State("so100_cell", fake, "fake")
srv = ThreadingHTTPServer(("127.0.0.1", 0), sim.make_handler(state))
port = srv.server_address[1]
threading.Thread(target=srv.serve_forever, daemon=True).start()
BASE = "http://127.0.0.1:%d" % port


def call(method, path, body=None, owner=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(BASE + path, data=data, method=method)
    if data:
        req.add_header("Content-Type", "application/json")
    if owner:
        req.add_header("X-Owner", owner)
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            return r.status, json.loads(r.read().decode())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode())


SIX = [1, 2, 3, 4, 5, 6]

check("status says which mode it is", call("GET", "/status")[1]["mode"], "fake")
check("no lease to start with", call("GET", "/owner")[1]["owner"], None)
check("claiming gives the arm", call("POST", "/claim", {"owner": "A", "ttl_s": 5})[1]["owner"], "A")
code, body = call("POST", "/claim", {"owner": "B", "ttl_s": 5})
check("a second claimant gets 423", code, 423)
check("...and is told who has it", "leased to 'A'" in body["error"], True)
check("a write without the lease gets 423", call("POST", "/write", {"angles": SIX}, owner=None)[0], 423)
check("a write with a stranger's token gets 423", call("POST", "/write", {"angles": SIX}, owner="B")[0], 423)
check("torque is off to start with", call("GET", "/torque")[1]["torque"], False)
check("a write with the lease but torque off is 409",
      call("POST", "/write", {"angles": SIX}, owner="A")[0], 409)
check("the operator's switch needs no lease", call("POST", "/torque", {"on": True})[1]["torque"], True)
code, body = call("POST", "/write", {"angles": SIX}, owner="A")
check("with the lease and torque on, the write lands", (code, body["status"]), (200, "ok"))
check("...and the six values reach the model in JOINT_ORDER",
      [fake.last[j] for j in sim.JOINT_ORDER], [float(v) for v in SIX])
code, body = call("POST", "/write", {"angles": [7, 8, 9, 10, 11], "hold": ["gripper"]}, owner="A")
check("five values with hold are accepted", code, 200)
check("...and the jaw is NOT driven", "gripper" in fake.last, False)
check("five values without hold are refused",
      call("POST", "/write", {"angles": [1, 2, 3, 4, 5]}, owner="A")[0], 400)
check("a NaN is refused", call("POST", "/write", {"angles": [1, 2, 3, 4, 5, float("nan")]}, owner="A")[0], 400)
check("an implausible angle is refused",
      call("POST", "/write", {"angles": [1, 2, 3, 4, 5, 900]}, owner="A")[0], 400)
check("an unknown unit system is refused",
      call("POST", "/write", {"angles": SIX, "units": "furlongs"}, owner="A")[0], 400)
check("release frees the arm", call("POST", "/release", {"owner": "A"})[1]["owner"], None)
check("after release a write needs no token", call("POST", "/write", {"angles": SIX})[0], 200)
code, body = call("GET", "/read")
check("read carries six named joints", (len(body["angles"]), body["names"]), (6, list(sim.JOINT_ORDER)))

srv.shutdown()
print("\n%s — %d checks, %d failed" % ("PASS" if not fail else "FAIL", ok + fail, fail))
sys.exit(1 if fail else 0)

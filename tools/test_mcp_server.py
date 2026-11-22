#!/usr/bin/env python3
"""The endpoint's behaviour, without a cell: JSON-RPC shape, tool list, and the sim guard.

    python3 tools/test_mcp_server.py
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mcp"))
sys.path.insert(0, os.path.join(HERE, "..", "docker", "ws", "lib"))
import joint_scale as js  # noqa: E402
import manifest as mf  # noqa: E402
import server as srv  # noqa: E402

ok = fail = 0


def check(label, got, want):
    global ok, fail
    if got == want:
        ok += 1
        print("ok   %-58s %r" % (label, got))
    else:
        fail += 1
        print("FAIL %-58s got %r want %r" % (label, got, want))


class Provider:
    def __init__(self, sim=True):
        self.sim = sim

    skills = []

    def skill(self, tool, arguments):
        self.skills.append((tool, dict(arguments)))
        if arguments.get("arm") == "nope":
            return {"accepted": False, "reason": "arm must be one of ['so100_cell', 'so100_cell_b']", "ticket": ""}
        return {"accepted": True, "reason": "ok", "ticket": "m1-so100_cell-1"}

    http = []

    def supervisor_http(self, path, body=None):
        self.http.append((path, body))
        if path.startswith("/goal"):
            return {"mission_id": "m1", "goal": {"skill": "move_joint", "accepted": True, "ticket": "t1"}}
        if path == "/cancel":
            return {"cancelled": (body or {}).get("mission_id"), "dropped": 0}
        if path == "/stop":
            return {"stopped": True, "dropped": 0}
        return {"error": "unexpected %r" % path}

    def sim_available(self):
        return {"available": self.sim, "reason": "" if self.sim else "no simulator listening on 8004"}

    def manifest(self):
        t1, t2 = js.load(js.path_for("so100_cell")), js.load(js.path_for("so100_cell_b"))
        arms = [{"arm": "so100_cell", "reachable": True, "owner": None, "torque": False, "table": t1},
                {"arm": "so100_cell_b", "reachable": True, "owner": "supervisor", "torque": True, "table": t2}]
        return mf.build({"robot": "bench-02", "serial": "so100_cell", "profile": "robot"}, arms,
                        mf.epoch("boot-1", {"so100_cell": t1, "so100_cell_b": t2}))


ep = srv.Endpoint(Provider(), "robot")


def rpc(method, params=None, rid=1):
    payload = {"jsonrpc": "2.0", "id": rid, "method": method}
    if params is not None:
        payload["params"] = params
    return ep.handle(payload)


# Phase 4: the journal and the local stop path.
p = Provider()
ep4 = srv.Endpoint(p, "robot")
r4 = ep4.handle({"jsonrpc": "2.0", "id": 9, "method": "tools/call",
                 "params": {"name": "get_goal", "arguments": {"mission_id": "m1"}}})
check("get_goal asks the supervisor for that mission",
      json.loads(r4["result"]["content"][0]["text"])["goal"]["ticket"], "t1")
r5 = ep4.handle({"jsonrpc": "2.0", "id": 10, "method": "tools/call",
                 "params": {"name": "get_goal", "arguments": {}}})
check("get_goal without a mission is refused", r5["error"]["code"], -32602)
r6 = ep4.handle({"jsonrpc": "2.0", "id": 11, "method": "tools/call",
                 "params": {"name": "cancel_goal", "arguments": {"mission_id": "m1"}}})
check("cancel_goal reaches /cancel", json.loads(r6["result"]["content"][0]["text"])["cancelled"], "m1")
r7 = ep4.handle({"jsonrpc": "2.0", "id": 12, "method": "tools/call",
                 "params": {"name": "stop", "arguments": {}}})
check("stop reaches /stop", json.loads(r7["result"]["content"][0]["text"])["stopped"], True)
check("and is a POST, not a queued skill", p.http[-1][0], "/stop")
check("stop takes no arguments",
      ep4.handle({"jsonrpc": "2.0", "id": 13, "method": "tools/call",
                  "params": {"name": "stop", "arguments": {"arm": "arm1"}}})["error"]["code"], -32602)

r = rpc("tools/list")
check("tools/list answers", r["result"]["tools"][0]["name"] in ("capabilities", "get_state"), True)
check("the ten tools are listed", sorted(t["name"] for t in r["result"]["tools"]),
      ["camera_snapshot", "cancel_goal", "capabilities", "detect_objects", "get_goal", "get_state",
       "move_joint_bounded", "set_gripper_aperture", "solve_ik", "stop"])
check("the moving tools declare their required fields",
      r["result"]["tools"][2]["inputSchema"].get("required"), ["arm", "angles"])
check("each tool declares an input schema", all("inputSchema" in t for t in r["result"]["tools"]), True)

r = rpc("tools/call", {"name": "capabilities", "arguments": {}})
body = json.loads(r["result"]["content"][0]["text"])
check("capabilities returns the identity", body["identity"]["robot"], "bench-02")
check("...with the epoch", len(body["identity"]["epoch"]), 16)
check("...and the skills", "move_joint" in body["skills"], True)
check("capabilities does not carry the whole arms table", "arms" in body, False)

r = rpc("tools/call", {"name": "get_state", "arguments": {}})
full = json.loads(r["result"]["content"][0]["text"])
check("get_state carries both arms", len(full["arms"]), 2)
check("...with each arm's deadband", full["arms"][1]["joints"]["shoulder_pan"]["deadband_deg"], 2.0)
check("...and who owns it", full["arms"][1]["owner"], "supervisor")

r = rpc("tools/call", {"name": "nope", "arguments": {}})
check("an unknown tool is a JSON-RPC error", r["error"]["code"], srv.METHOD_NOT_FOUND)
check("...that lists what exists", "capabilities" in r["error"]["message"], True)
r = rpc("tools/call", {"name": "get_state", "arguments": {"arm": "x"}})
check("arguments to a no-argument tool are refused", r["error"]["code"], srv.INVALID_PARAMS)
r = rpc("fly")
check("an unknown method is refused", r["error"]["code"], srv.METHOD_NOT_FOUND)
check("the id is echoed back", r["id"], 1)
check("a non-JSON-RPC payload is refused", ep.handle({"id": 1})["error"]["code"], srv.INVALID_REQUEST)

# ---- the sim profile cannot answer for the physical cell
ep_sim_bad = srv.Endpoint(Provider(sim=False), "sim")
r = ep_sim_bad.handle({"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                       "params": {"name": "get_state", "arguments": {}}})
check("sim without a simulator is a configuration error", r["error"]["code"], srv.UNAVAILABLE)
check("...and says why", "no simulator" in r["error"]["message"], True)
ep_sim_ok = srv.Endpoint(Provider(sim=True), "sim")
r = ep_sim_ok.handle({"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                      "params": {"name": "capabilities", "arguments": {}}})
check("sim with a simulator answers", "result" in r, True)

# ---- phase 2: the endpoint hands the request over and reports what came back
P = Provider()
ep2 = srv.Endpoint(P, "robot")
r = ep2.handle({"jsonrpc": "2.0", "id": 9, "method": "tools/call",
                "params": {"name": "move_joint_bounded", "arguments": {"arm": "so100_cell",
                                                                      "angles": [1, 2, 3, 4, 5, 6]}}})
check("a joint move is accepted when the cell says so", r["result"].get("isError"), False)
check("the epoch was added by the endpoint, not the caller", "epoch" in P.skills[-1][1], True)
check("...and a deadline too", P.skills[-1][1].get("deadline_s"), 1.0)
r = ep2.handle({"jsonrpc": "2.0", "id": 10, "method": "tools/call",
                "params": {"name": "move_joint_bounded", "arguments": {"arm": "nope", "angles": [0]}}})
check("a refusal from the cell is an error result, not a silent ok", r["result"]["isError"], True)
r = ep2.handle({"jsonrpc": "2.0", "id": 11, "method": "tools/call",
                "params": {"name": "move_joint_bounded", "arguments": {"arm": "so100_cell"}}})
check("missing angles are refused before the call", r["error"]["code"], srv.INVALID_PARAMS)
r = ep2.handle({"jsonrpc": "2.0", "id": 12, "method": "tools/call",
                "params": {"name": "set_gripper_aperture",
                           "arguments": {"arm": "so100_cell", "mm": 40, "speeds": 2}}})
check("an unknown argument is refused", r["error"]["code"], srv.INVALID_PARAMS)
r = ep2.handle({"jsonrpc": "2.0", "id": 13, "method": "tools/call",
                "params": {"name": "set_gripper_aperture", "arguments": {"arm": "so100_cell", "mm": 40}}})
check("a jaw request reaches the cell", P.skills[-1][0], "set_gripper_aperture")
check("...carrying the millimetres", P.skills[-1][1].get("mm"), 40)

print("\n%s — %d checks, %d failed" % ("PASS" if not fail else "FAIL", ok + fail, fail))
sys.exit(1 if fail else 0)

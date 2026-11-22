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


r = rpc("tools/list")
check("tools/list answers", r["result"]["tools"][0]["name"] in ("capabilities", "get_state"), True)
check("both tools are listed", sorted(t["name"] for t in r["result"]["tools"]), ["capabilities", "get_state"])
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

print("\n%s — %d checks, %d failed" % ("PASS" if not fail else "FAIL", ok + fail, fail))
sys.exit(1 if fail else 0)

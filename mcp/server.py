#!/usr/bin/env python3
"""The endpoint an agent talks to: JSON-RPC over HTTP, tools listed and called, nothing else.

Phase 1 of docs/design/mcp-endpoint-plan.md. It is deliberately *not* a ROS bridge and not a second writer:
the only tools it can ever expose are read-only until phase 2 wires `move_joint_bounded` through the
supervisor, and even then the supervisor decides.

    POST /mcp   {"jsonrpc":"2.0","id":1,"method":"tools/list"}
    POST /mcp   {"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"get_state","arguments":{}}}

The provider is injected, so the tests and the cell are the same code path: the provider returns a manifest
(or raises) and this file only knows about JSON, methods and errors.

Two rules enforced here rather than documented:

  - **A sim profile cannot answer if its simulator is not there.** `profile="sim"` requires the provider to
    say `sim_available: true`; otherwise every call fails with a configuration error. A silent fallback to the
    physical cell is the one bug that would be unrecoverable.
  - **Errors are data, not silence.** Unknown tool, missing parameters and provider refusals come back as a
    JSON-RPC error with a code the agent can branch on.
"""
from __future__ import annotations

import argparse
import json
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

TOOLS = {
    "capabilities": {"description": "What this cell can do: skills, profiles and versions.",
                     "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
    "get_state": {"description": "Identity, epoch, per-joint scale and deadband, and each arm's real state.",
                  "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
}

PARSE_ERROR, INVALID_REQUEST, METHOD_NOT_FOUND = -32700, -32600, -32601
INVALID_PARAMS, INTERNAL_ERROR = -32602, -32603
UNAVAILABLE = -32000            # our own: the cell is not in a state to answer


class Endpoint:
    def __init__(self, provider, profile):
        self.provider, self.profile = provider, profile

    def _guard(self):
        """Called before any tool. The sim profile must never answer for the physical cell."""
        if self.profile == "sim":
            state = self.provider.sim_available()
            if not state.get("available"):
                raise LookupError("profile 'sim' has no simulator: %s" % state.get("reason", "not running"))

    def list_tools(self):
        return {"tools": [dict(name=n, **spec) for n, spec in TOOLS.items()]}

    def call(self, name, arguments):
        if name not in TOOLS:
            raise KeyError(name)
        if arguments not in (None, {}):
            raise ValueError("%s takes no arguments" % name)
        self._guard()
        manifest = self.provider.manifest()
        if arguments is None:
            arguments = {}
        if name == "capabilities":
            return {"content": [{"type": "text", "text": json.dumps({
                "schema": manifest["schema"], "identity": manifest["identity"],
                "skills": manifest["skills"], "profiles": ["robot", "sim"]}, indent=2)}]}
        return {"content": [{"type": "text", "text": json.dumps(manifest, indent=2)}]}

    def handle(self, payload):
        if not isinstance(payload, dict) or payload.get("jsonrpc") != "2.0":
            return {"jsonrpc": "2.0", "id": None,
                    "error": {"code": INVALID_REQUEST, "message": "expected a JSON-RPC 2.0 object"}}
        rid, method = payload.get("id"), payload.get("method")
        try:
            if method == "tools/list":
                return {"jsonrpc": "2.0", "id": rid, "result": self.list_tools()}
            if method == "tools/call":
                params = payload.get("params") or {}
                try:
                    result = self.call(params.get("name"), params.get("arguments"))
                except KeyError:
                    return {"jsonrpc": "2.0", "id": rid,
                            "error": {"code": METHOD_NOT_FOUND,
                                      "message": "unknown tool %r; known: %s"
                                                 % (params.get("name"), sorted(TOOLS))}}
                except ValueError as exc:
                    return {"jsonrpc": "2.0", "id": rid,
                            "error": {"code": INVALID_PARAMS, "message": str(exc)}}
                except LookupError as exc:
                    return {"jsonrpc": "2.0", "id": rid,
                            "error": {"code": UNAVAILABLE, "message": str(exc)}}
                return {"jsonrpc": "2.0", "id": rid, "result": result}
            return {"jsonrpc": "2.0", "id": rid,
                    "error": {"code": METHOD_NOT_FOUND, "message": "unknown method %r" % method}}
        except Exception as exc:                                     # noqa: BLE001
            return {"jsonrpc": "2.0", "id": rid,
                    "error": {"code": INTERNAL_ERROR, "message": "%s: %s" % (type(exc).__name__, exc)}}


def make_handler(endpoint):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, payload):
            raw = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(raw)

        def do_POST(self):
            if self.path.rstrip("/") != "/mcp":
                return self._send(404, {"error": "not found — the endpoint is /mcp"})
            n = int(self.headers.get("Content-Length") or 0)
            try:
                payload = json.loads(self.rfile.read(n).decode() or "{}")
            except ValueError:
                return self._send(200, {"jsonrpc": "2.0", "id": None,
                                        "error": {"code": PARSE_ERROR, "message": "body is not JSON"}})
            return self._send(200, endpoint.handle(payload))

        def do_GET(self):
            return self._send(405, {"error": "POST JSON-RPC to /mcp"})

    return Handler


class PortProvider:
    """Builds the manifest from the arms as they really are, over their own HTTP ports."""

    def __init__(self, identity, ports, lib_dir):
        self.identity, self.ports, self.lib_dir = identity, ports, lib_dir

    def _get(self, port, path):
        import urllib.request
        try:
            with urllib.request.urlopen("http://127.0.0.1:%d%s" % (port, path), timeout=2) as r:
                return json.loads(r.read().decode()), None
        except Exception as exc:                                     # noqa: BLE001
            return None, "%s" % exc

    def sim_available(self):
        port = self.ports.get("sim")
        if not port:
            return {"available": False, "reason": "no sim port configured"}
        _, err = self._get(port, "/status")
        return {"available": err is None, "reason": err or ""}

    def manifest(self):
        sys.path.insert(0, self.lib_dir)
        import joint_scale as js
        import manifest as mf
        arms = []
        for arm, port in self.ports.items():
            if arm == "sim":
                continue
            owner, err = self._get(port, "/owner")
            torque, _ = self._get(port, "/torque")
            arms.append({"arm": arm, "reachable": err is None,
                         "owner": (owner or {}).get("owner"), "torque": (torque or {}).get("torque"),
                         "table": js.load(js.path_for(arm)), "error": err})
        tables = {a["arm"]: a["table"] for a in arms}
        boot = ""
        try:
            with open("/proc/sys/kernel/random/boot_id") as fh:
                boot = fh.read().strip()
        except OSError:
            boot = "no-boot-id"
        return mf.build(self.identity, arms, mf.epoch(boot, tables))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--robot", default="bench-02")
    ap.add_argument("--serial", default="so100_cell")
    ap.add_argument("--profile", default="robot", choices=("robot", "sim"))
    ap.add_argument("--sim-port", type=int, default=0)
    ap.add_argument("--lib", default="/opt/ws/lib")
    args = ap.parse_args(argv)

    ports = {"so100_cell": 8002, "so100_cell_b": 8003}
    if args.sim_port:
        ports["sim"] = args.sim_port
    provider = PortProvider({"robot": args.robot, "serial": args.serial, "profile": args.profile},
                            ports, args.lib)
    srv = ThreadingHTTPServer((args.host, args.port), make_handler(Endpoint(provider, args.profile)))
    print("mcp: http://%s:%d/mcp — profile %s" % (args.host, args.port, args.profile), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())

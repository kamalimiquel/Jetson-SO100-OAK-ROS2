#!/usr/bin/env python3
"""One glance at the cell: who owns each arm, what the scales say, and whether the supervisor is up.

Runs wherever the arms' loopback ports are reachable — inside the control container, or on the Xavier
itself. Its whole job is to be the data layer a page can render: it talks to nothing it cannot see, and says
"unreachable" instead of inventing a value.

    python3 tools/cell_status.py                # table, for a terminal
    python3 tools/cell_status.py --json         # same data, for a page or a log
    python3 tools/cell_status.py --port so100_cell=8002 --port so100_cell_b=8003 --skill-port 8000

Exit code is 0 when every arm answered, 1 when at least one did not: a monitoring script can tell "all quiet"
from "one arm is deaf" without parsing the text.
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import urllib.error
import urllib.request

DEFAULT_PORTS = {"so100_cell": 8002, "so100_cell_b": 8003}
TIMEOUT_S = 1.5


def get(port, path):
    """A tiny GET that never raises: the answer is (payload, error)."""
    url = "http://127.0.0.1:%d%s" % (port, path)
    try:
        with urllib.request.urlopen(url, timeout=TIMEOUT_S) as r:
            return json.loads(r.read().decode("utf-8")), None
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", "replace").strip()
        return None, "HTTP %d: %s" % (exc.code, body[:120])
    except (urllib.error.URLError, socket.timeout, ValueError) as exc:
        return None, "unreachable (%s)" % getattr(exc, "reason", exc)


def arm_status(name, port):
    """What a page wants to know about one arm, one field at a time."""
    row = {"arm": name, "port": port, "reachable": False}
    owner, err = get(port, "/owner")
    if err:
        row["error"] = err
        return row
    row["reachable"] = True
    row["owner"] = (owner or {}).get("owner")
    status, err = get(port, "/status")
    if err:
        row["error"] = err
    else:
        row["connected"] = (status or {}).get("connected")
        row["calibrated"] = (status or {}).get("calibrated")
    scale, err = get(port, "/scale")
    if err:
        row["scale"] = None
    else:
        scale = scale or {}
        row["scale"] = {"arm": scale.get("arm"), "applied": scale.get("applied"),
                        "sha": (scale.get("sha256") or "")[:8], "session": scale.get("session")}
    return row


def supervisor(skill_port):
    """The supervisor is optional by design: it is not started by entry.sh, so silence is normal."""
    payload, err = get(skill_port, "/status")
    if err:
        return {"listening": False, "note": "not running (normal unless a mission is up)"}
    return {"listening": True, "detail": payload}


def render(rows, sup):
    out = []
    out.append("%-14s %-5s %-6s %-8s %-8s %-6s %-10s %s"
               % ("arm", "port", "up", "owner", "torque", "fault", "scale", "session"))
    for r in rows:
        out.append("%-14s %-5d %-6s %-8s %-8s %-6s %-10s %s"
                   % (r["arm"], r["port"],
                      "yes" if r["reachable"] else "NO",
                      r.get("owner") or "-",
                      ("on" if r.get("torque") else "off") if "torque" in r else "-",
                      (r.get("fault") or "-") if "fault" in r else "-",
                      ("%s%s" % (r["scale"]["sha"], "" if r["scale"]["applied"] else " (off)"))
                      if r.get("scale") else "-",
                      (r.get("scale") or {}).get("session") or "-"))
        if r.get("error"):
            out.append("               └ %s" % r["error"])
    out.append("")
    out.append("supervisor: %s" % ("up" if sup["listening"] else sup["note"]))
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", action="append", default=[], metavar="ARM=PORT",
                    help="repeatable; defaults to %s" % DEFAULT_PORTS)
    ap.add_argument("--skill-port", type=int, default=8000,
                    help="the supervisor's HTTP status port (optional; silence is normal)")
    ap.add_argument("--json", action="store_true", help="the same data, machine-readable")
    args = ap.parse_args(argv)

    ports = dict(DEFAULT_PORTS)
    if args.port:
        ports = {}
        for item in args.port:
            name, _, number = item.partition("=")
            ports[name] = int(number)

    rows = [arm_status(name, port) for name, port in ports.items()]
    sup = supervisor(args.skill_port)
    ok = all(r["reachable"] for r in rows)

    if args.json:
        print(json.dumps({"arms": rows, "supervisor": sup, "all_reachable": ok}, indent=2))
    else:
        print(render(rows, sup))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Headless test of the joint server's HTTP boundary — no ROS, no serial bus, no lerobot.

The server is where a command becomes motion, and every failure here used to be silent: a five-value
array left the jaw alone, seven dropped the last, one write per motor could leave a partial pose, and
an out-of-range value was clamped to the limit and answered 200. This test runs the real handler on a
loopback port against a fake bus, and asserts the status codes and what reached the bus.

    python3 tools/test_joint_server.py
"""

import json
import os
import sys
import threading
import time
import types
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
SERVER = os.path.join(os.path.dirname(HERE), "docker", "ws", "arm", "lerobot_joint_server.py")
PORT = 8791
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

fails = []


def check(name, got, want):
    ok = got == want
    print(("  ok   " if ok else "  FAIL ") + "%s: %r" % (name, got) +
          ("" if ok else " (expected %r)" % (want,)))
    if not ok:
        fails.append(name)


class FakeMotor:
    """What the SDK reports per motor: the unit system the server now asserts at boot."""

    def __init__(self, mid, name):
        self.id = mid
        self.model = "sts3215"
        self.norm_mode = types.SimpleNamespace(name="RANGE_0_100" if name == "gripper" else "DEGREES")


class FakeBus:
    def __init__(self):
        self.motors = {n: FakeMotor(i + 1, n) for i, n in enumerate(NAMES)}
        self.present = dict(zip(NAMES, [0.0, -10.0, 20.0, 0.0, 0.0, 40.0]))
        self.sync_calls = []
        self.single_writes = []
        self.torque = False
        self.fail_next_sync = None

    def read(self, what, name, *, normalize=True):   # normalize= is what the audit table needs
        return self.present[name] if normalize else int(self.present[name] * 10) + 2048

    def write(self, what, name, value, *, normalize=True):   # must never be used by /write
        self.single_writes.append((name, value))

    def sync_write(self, what, values, *, normalize=True):
        self.sync_calls.append(dict(values))
        if self.fail_next_sync:
            err, apply = self.fail_next_sync
            self.fail_next_sync = None
            if apply:
                self.present.update({k: v for k, v in values.items()})
            raise RuntimeError(err)
        self.present.update({k: v for k, v in values.items()})

    def enable_torque(self):
        self.torque = True

    def disable_torque(self):
        self.torque = False


BUS = FakeBus()


def install_fake_lerobot():
    arm = types.SimpleNamespace(bus=BUS, is_calibrated=True, is_connected=True,
                                config=types.SimpleNamespace(use_degrees=True),
                                calibration_fpath=None)
    arm.connect = lambda: None
    arm.disconnect = lambda: None
    mod = types.ModuleType("lerobot.robots.so_follower")
    mod.SO100Follower = lambda cfg: arm
    # a namespace, not a dict: the server reads config.use_degrees back out of it
    mod.SO100FollowerConfig = lambda **kw: types.SimpleNamespace(**kw)
    for name in ("lerobot", "lerobot.robots", "lerobot.robots.so_follower"):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules["lerobot.robots.so_follower"] = mod
    sys.modules["lerobot.robots"].so_follower = mod


def call(method, path, body=None, owner=None, timeout=2.0):
    req = urllib.request.Request("http://127.0.0.1:%d%s" % (PORT, path), method=method,
                                 data=json.dumps(body).encode() if body is not None else None,
                                 headers={"Content-Type": "application/json"})
    if owner:
        req.add_header("X-Owner", owner)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, json.loads(resp.read() or b"{}")
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read() or b"{}")


def main():
    os.environ["PORT"] = str(PORT)
    os.environ["SO100_PORT"] = "/dev/null"
    install_fake_lerobot()
    import importlib.util
    spec = importlib.util.spec_from_file_location("joint_server", SERVER)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["joint_server"] = mod
    spec.loader.exec_module(mod)

    thread = threading.Thread(target=mod.main, daemon=True)
    thread.start()
    deadline = time.time() + 10
    while time.time() < deadline:
        try:
            code, _ = call("GET", "/status")
            if code == 200:
                break
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.05)
    else:
        print("  FAIL the server never came up")
        return 1

    six = [1.0, -20.0, 30.0, -40.0, 50.0, 60.0]

    print("the shape of the command")
    code, answer = call("POST", "/write", {"angles": six[:5]})
    check("five values with no hold: 400, not 500", code, 400)
    check("and the sentence says how", "hold" in answer.get("error", ""), True)
    code, answer = call("POST", "/write", {"angles": six + [0.0]})
    check("seven values: 400", code, 400)
    code, answer = call("POST", "/write", {"angles": [1.0, 2.0, float("nan"), 4.0, 5.0, 6.0]})
    check("NaN on an arm joint: 400", code, 400)
    code, answer = call("POST", "/write", {"angles": [190.0] + six[1:]})
    check("out of range: 400 naming the joint", (code, "shoulder_pan" in answer.get("error", "")),
          (400, True))
    check("nothing reached the bus for any of those", BUS.sync_calls, [])

    print("torque is still the gate")
    code, _ = call("POST", "/write", {"angles": six})
    check("torque off: 409", code, 409)
    code, _ = call("POST", "/torque", {"on": True})
    check("arming works with no lease held", code, 200)
    check("arming seeded the goals from the present positions", len(BUS.sync_calls), 1)

    print("one packet, six joints")
    BUS.sync_calls.clear()
    code, answer = call("POST", "/write", {"angles": six})
    check("a valid command: 200", code, 200)
    check("exactly one sync_write", len(BUS.sync_calls), 1)
    check("with all six joints in it", sorted(BUS.sync_calls[0]), sorted(NAMES))
    check("and no per-motor write at all", BUS.single_writes, [])

    BUS.sync_calls.clear()
    code, answer = call("POST", "/write", {"angles": six[:5], "hold": ["gripper"]})
    check("five + hold: 200", code, 200)
    check("the jaw is not in the packet", "gripper" in BUS.sync_calls[0], False)
    check("the five arm joints are", len(BUS.sync_calls[0]), 5)
    check("the answer says what was held", answer.get("held"), ["gripper"])

    print("ownership at the port")
    code, answer = call("POST", "/claim", {"owner": "policy", "ttl_s": 30.0})
    check("a supervisor can claim the arm", (code, answer.get("owner")), (200, "policy"))
    code, answer = call("POST", "/write", {"angles": six})
    check("another writer without the header: 423", code, 423)
    code, answer = call("POST", "/write", {"angles": six}, owner="policy")
    check("the owner's write: 200", code, 200)
    code, answer = call("POST", "/write", {"angles": six}, owner="someone_else")
    check("a wrong owner: 423", code, 423)
    code, answer = call("POST", "/torque", {"on": True})
    check("torque stays the operator's switch while a lease is held (no X-Owner needed)", code, 200)
    code, answer = call("POST", "/write", {"angles": six})
    check("but motion still belongs to the owner", code, 423)
    code, answer = call("POST", "/write", {"angles": six}, owner="policy")
    check("and the owner can still drive it", code, 200)
    code, _ = call("POST", "/torque", {"on": True}, owner="policy")
    check("the owner may arm as well", code, 200)

    code, _ = call("POST", "/release", {}, owner="policy")
    check("release gives the path back", code, 200)
    code, _ = call("POST", "/write", {"angles": six})
    check("and an unclaimed arm writes again", code, 200)

    print("units and the audit table")
    code, man = call("GET", "/manifest")
    check("the server reports its unit manifest", code, 200)
    check("body joints in degrees, jaw in percent",
          [j["norm"] for j in man["joints"]],
          ["DEGREES"] * 5 + ["RANGE_0_100"])
    check("and says so explicitly", man["use_degrees"], True)
    check("the joint order is the contract", [j["name"] for j in man["joints"]], NAMES)
    code, raw = call("GET", "/read_raw")
    check("raw ticks are readable next to the SDK value", code, 200)
    check("both numbers for every motor", sorted(raw["ticks"]), sorted(NAMES))
    check("the raw value is not the normalised one",
          raw["ticks"]["shoulder_pan"] != raw["normalized"]["shoulder_pan"], True)

    print("a failed packet is measured, not trusted")
    BUS.present.update({"wrist_roll": 0.0})
    BUS.fail_next_sync = ("Checksum error on motor 4", False)
    code, answer = call("POST", "/write", {"angles": six})
    check("the failure is a 502 with the measured pose", (code, "measured" in answer), (502, True))
    code, answer = call("POST", "/write", {"angles": six})
    check("and the path stays held: 409 with the fault", (code, "fault" in answer), (409, True))
    code, _ = call("POST", "/torque", {"on": False})
    check("cycling torque is the way out", code, 200)
    code, _ = call("POST", "/torque", {"on": True})
    code, _ = call("POST", "/write", {"angles": six})
    check("after torque, writes flow again", code, 200)

    BUS.fail_next_sync = ("Checksum error on motor 4", True)   # the packet DID land
    code, answer = call("POST", "/write", {"angles": six})
    check("a packet that failed but landed counts as delivered", (code, answer.get("status")),
          (200, "ok"))
    check("and says why in the answer", "bus error ignored" in answer.get("note", ""), True)

    print("\n%s" % ("all checks pass" if not fails else "%d check(s) FAILED" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

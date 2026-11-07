#!/usr/bin/env python
"""Phase 1 verification probe: talk to the SO-100 Feetech bus with the LeRobot SDK.

Reads every motor's ID and Present_Position over the serial bus and prints them.
No calibration needed. Run on the Jetson with the lerobot venv:

    ~/.local/bin/uv run --python ~/lerobot/.venv/bin/python tools/lerobot_bus_probe.py [PORT]

Exit codes: 0 = all six motors answered; 1 = bus read failed (wiring, power,
baud or ID mismatch).
"""

import sys

from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"


def main() -> int:
    cfg = SO100FollowerConfig(port=PORT, id="so100_cell")
    arm = SO100Follower(cfg)
    arm.connect(calibrate=False)
    bus = arm.bus
    names = list(bus.motors.keys())
    print(f"bus={type(bus).__name__} port={PORT} motors={names}")

    ok = 0
    for name in names:
        try:
            pos = bus.read("Present_Position", name, normalize=False)
            print(f"{name}: id={bus.motors[name].id} pos={pos}")
            ok += 1
        except Exception as exc:  # noqa: BLE001
            print(f"{name}: READ ERROR {type(exc).__name__}: {exc}")

    arm.disconnect()
    print(f"OK {ok}/{len(names)}")
    return 0 if ok == len(names) else 1


if __name__ == "__main__":
    sys.exit(main())

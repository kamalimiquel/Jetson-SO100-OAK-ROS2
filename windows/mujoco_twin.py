"""MuJoCo digital twin of the SO-100 (ADR-0016) via rosbridge WebSocket.

The Xavier's web container runs rosbridge (port 9090) which streams
/joint_states as JSON over WebSocket. This script subscribes, converts the
arm's joint angles (degrees, as published) into the MuJoCo model's qpos
(radians), and mirrors the arm in the MuJoCo viewer. Physics is NOT stepped —
it is a kinematic mirror, so the sim cannot fight the real arm.

Usage (on the Windows PC):
    pip install roslibpy mujoco
    python windows\\mujoco_twin.py C:\\path\\to\\mujoco_menagerie\\trs_so_arm100\\scene.xml

Optional: --host 192.168.1.185 --port 9090
"""

import argparse
import math

import mujoco
import roslibpy

try:
    from mujoco import viewer
except ImportError:
    raise SystemExit(
        "mujoco %s has no viewer module. The classic viewer exists in mujoco 3.x: "
        "pip install \"mujoco>=3,<4\"" % mujoco.__version__
    )

WANTED = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# Menagerie trs_so_arm100 names the joints differently than LeRobot.
ALIASES = {
    "shoulder_pan": ["rotation"],
    "shoulder_lift": ["pitch"],
    "elbow_flex": ["elbow"],
    "wrist_flex": ["wrist_pitch"],
    "wrist_roll": ["wrist_roll"],
    "gripper": ["jaw"],
}


def build_map(model):
    """WANTED joint name -> qpos index in the MuJoCo model."""
    names = [
        (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or "").lower()
        for i in range(model.njnt)
    ]
    mapping = {}
    used = set()

    # 1) direct name match
    for want in WANTED:
        for i, name in enumerate(names):
            if i not in used and want in name:
                mapping[want] = i
                used.add(i)
                break
    # 2) menagerie aliases
    for want in WANTED:
        if want in mapping:
            continue
        for alias in ALIASES.get(want, []):
            for i, name in enumerate(names):
                if i not in used and alias in name:
                    mapping[want] = i
                    used.add(i)
                    break
            if want in mapping:
                break
    # 3) positional fallback when the model has exactly 6 qpos
    if len(mapping) < len(WANTED) and model.nq == len(WANTED):
        for j, want in enumerate(WANTED):
            if want not in mapping and j not in mapping.values():
                mapping[want] = j
    missing = [w for w in WANTED if w not in mapping]
    if missing:
        print(f"WARNING: joints not found in model: {missing}")
    return mapping


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("model", help="path to trs_so_arm100/scene.xml")
    parser.add_argument("--host", default="192.168.1.185")
    parser.add_argument("--port", type=int, default=9090)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    mapping = build_map(model)
    print(f"model: {model.nq} qpos; mapped {len(mapping)}/{len(WANTED)} joints")
    for want, i in sorted(mapping.items(), key=lambda kv: kv[1]):
        print(f"  qpos[{i}] <- {want}")

    client = roslibpy.Ros(host=args.host, port=args.port)
    client.run()
    sub = roslibpy.Topic(client, "/joint_states", "sensor_msgs/JointState")
    latest = {}
    first = [True]

    def on_message(msg):
        names = msg.get("name", [])
        positions = msg.get("position", [])
        if first[0]:
            first[0] = False
            print(f"first /joint_states: {len(names)} joints, positions (deg): "
                  f"{[round(p, 2) for p in positions]}")
        for want, i in mapping.items():
            if want in names:
                latest[i] = math.radians(positions[names.index(want)])

    sub.subscribe(on_message)
    print("subscribed to /joint_states — waiting for data...")

    handle = viewer.launch(model, data)
    try:
        while handle.is_running():
            for i, angle in latest.items():
                data.qpos[i] = angle
            mujoco.mj_forward(model, data)
            handle.sync()
    finally:
        sub.unsubscribe()
        client.terminate()
        handle.close()


if __name__ == "__main__":
    main()

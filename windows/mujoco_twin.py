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

if not hasattr(mujoco, "viewer"):
    raise SystemExit(
        "mujoco %s has no viewer module — upgrade:  pip install -U mujoco" % mujoco.__version__
    )

WANTED = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def build_map(model):
    """WANTED joint name -> qpos index in the MuJoCo model (fuzzy by name)."""
    names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or ""
        for i in range(model.njnt)
    ]
    mapping = {}
    for want in WANTED:
        for i, name in enumerate(names):
            if want in name.lower().replace(" ", "_"):
                mapping[want] = i
                break
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
    print(f"model: {model.nq} qpos; mapped {len(mapping)}/6 joints")

    client = roslibpy.Ros(host=args.host, port=args.port)
    client.run()
    sub = roslibpy.Topic(client, "/joint_states", "sensor_msgs/JointState")
    latest = {}

    def on_message(msg):
        names = msg.get("name", [])
        positions = msg.get("position", [])
        for want, i in mapping.items():
            if want in names:
                latest[i] = math.radians(positions[names.index(want)])

    sub.subscribe(on_message)
    print("subscribed to /joint_states — waiting for data...")

    viewer = mujoco.viewer.launch(model, data)
    try:
        while viewer.is_running():
            for i, angle in latest.items():
                data.qpos[i] = angle
            mujoco.mj_forward(model, data)
            viewer.sync()
    finally:
        sub.unsubscribe()
        client.terminate()
        viewer.close()


if __name__ == "__main__":
    main()

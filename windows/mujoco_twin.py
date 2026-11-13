"""MuJoCo digital twin of the SO-100 (ADR-0016) via rosbridge WebSocket.

The Xavier's web container runs rosbridge (port 9090) which streams the arm's
/joint_states as JSON over WebSocket. This script subscribes, converts the
angles (degrees, as published) into the MuJoCo model's qpos (radians), and
mirrors the arm in the MuJoCo viewer. Physics is NOT stepped — it is a
kinematic mirror, so the sim cannot fight the real arm.

Two-arm (Plan Phase 6): pass the generated scene_two_arms.xml and --both-arms.
It then subscribes to both /joint_states (arm 1) and /arm2/joint_states
(arm 2), maps each into its own (arm2_-prefixed) qpos tree, and draws both
arms 0.30 m apart — the same number as the base -> arm2_base transform.

Usage (on the Windows PC):
    pip install roslibpy mujoco
    python windows\\mujoco_twin.py C:\\path\\to\\mujoco_menagerie\\trs_so_arm100\\scene.xml
    python windows\\mujoco_twin.py C:\\path\\to\\mujoco_menagerie\\robotstudio_so101\\scene_two_arms.xml --both-arms
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
PREFIX = "arm2_"

# Menagerie trs_so_arm100 names the joints differently than LeRobot.
ALIASES = {
    "shoulder_pan": ["rotation"],
    "shoulder_lift": ["pitch"],
    "elbow_flex": ["elbow"],
    "wrist_flex": ["wrist_pitch"],
    "wrist_roll": ["wrist_roll"],
    "gripper": ["jaw"],
}


def build_map(model, arm2=False):
    """WANTED joint -> qpos index for ONE arm's tree (arm2 = arm2_-prefixed ids)."""
    all_names = [
        (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or "").lower()
        for i in range(model.njnt)
    ]
    if arm2:
        idx = [i for i, n in enumerate(all_names) if n.startswith(PREFIX)]
        names = [all_names[i][len(PREFIX):] for i in idx]
    else:
        idx = [i for i, n in enumerate(all_names) if not n.startswith(PREFIX)]
        names = [all_names[i] for i in idx]

    mapping = {}
    used = set()

    def pick(want):
        for j, name in enumerate(names):
            if j not in used and want in name:
                used.add(j)
                return idx[j]
        for alias in ALIASES.get(want, []):
            for j, name in enumerate(names):
                if j not in used and alias in name:
                    used.add(j)
                    return idx[j]
        if len(names) == len(WANTED) and len(mapping) < len(WANTED):   # positional fallback
            for j, name in enumerate(names):
                if j not in used:
                    used.add(j)
                    return idx[j]
        return None

    for want in WANTED:
        m = pick(want)
        if m is not None:
            mapping[want] = m
    missing = [w for w in WANTED if w not in mapping]
    if missing:
        print(f"WARNING: joints not found in {'arm2' if arm2 else 'arm1'} tree: {missing}")
    return mapping


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("model", help="path to the MuJoCo scene.xml (single- or two-arm)")
    parser.add_argument("--host", default="192.168.1.185")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--both-arms", action="store_true",
                        help="the scene holds two arms: subscribe to /joint_states and /arm2/joint_states")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    scopes = [False]
    if args.both_arms:
        scopes = [False, True]
    mappings = [build_map(model, arm2) for arm2 in scopes]
    for i, m in enumerate(mappings):
        label = ["arm 1", "arm 2"][i]
        print(f"{label}: mapped {len(m)}/{len(WANTED)} joints"
              + ("" if m else "  (none — is the scene two-arm? did the tf prefix match?)"))
        for want, q in sorted(m.items(), key=lambda kv: kv[1]):
            print(f"  qpos[{q}] <- {label} {want}")
    print(f"model: {model.nq} qpos")

    client = roslibpy.Ros(host=args.host, port=args.port)
    client.run()
    topics = [("/joint_states", 0)]
    if args.both_arms:
        topics.append(("/arm2/joint_states", 1))
    latest = [{}, {}]                  # [arm][qpos_idx] = radians

    def on_message(msg, arm):
        names = msg.get("name", [])
        positions = msg.get("position", [])
        for want, q in mappings[arm].items():
            if want in names:
                latest[arm][q] = math.radians(positions[names.index(want)])

    for tname, arm in topics:
        sub = roslibpy.Topic(client, tname, "sensor_msgs/JointState")
        sub.subscribe(lambda msg, arm=arm: on_message(msg, arm))
    print("subscribed: " + ", ".join(t[0] for t in topics))

    handle = viewer.launch_passive(model, data)
    try:
        while handle.is_running():
            for arm in range(len(mappings)):
                for i, angle in latest[arm].items():
                    data.qpos[i] = angle
            mujoco.mj_forward(model, data)
            handle.sync()
    finally:
        client.terminate()
        handle.close()


if __name__ == "__main__":
    main()

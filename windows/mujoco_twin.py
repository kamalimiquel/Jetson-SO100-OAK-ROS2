"""MuJoCo digital twin of the SO-100 (ADR-0016) via rosbridge WebSocket.

The Xavier's rosbridge container (port 9090) streams the arm's
/arm1/joint_states as JSON over WebSocket (ADR-0023: both arms are namespaced). This script subscribes, converts the
angles (degrees, as published) into the MuJoCo model's qpos (radians), and
mirrors the arm in the MuJoCo viewer. Physics is NOT stepped — it is a
kinematic mirror, so the sim cannot fight the real arm.

Two-arm (Plan Phase 6): pass the generated scene_two_arms.xml and --both-arms.
It then subscribes to both /arm1/joint_states and /arm2/joint_states, maps each
into its own (arm2_-prefixed) qpos tree, and draws both
arms 0.30 m apart — the same number as the base -> arm2_base transform.

Usage (on the Windows PC):
    pip install roslibpy mujoco
    python windows\\mujoco_twin.py C:\\path\\to\\mujoco_menagerie\\trs_so_arm100\\scene.xml
    python windows\\mujoco_twin.py C:\\path\\to\\mujoco_menagerie\\robotstudio_so101\\scene_two_arms.xml --both-arms
"""

import argparse
import json
import math
import os
import time

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


def check_geometry(model, data, path):
    """Compare the scene's arm separation with the one file that says where arm 2 stands.

    Returns True when they agree. The twin keeps running either way: this is a viewer, and stopping
    it would hide the very thing the operator opened it to look at.
    """
    try:
        with open(path, encoding="utf-8") as fh:
            off = json.load(fh)["arm2_offset_m"]
    except FileNotFoundError:
        print(f"geometry check skipped: {path} not found")
        return None
    want = (off["x"] ** 2 + off["y"] ** 2 + off["z"] ** 2) ** 0.5
    b1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    b2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, PREFIX + "base")
    if min(b1, b2) < 0:
        print("geometry check: the scene has no 'base' / '%sbase' bodies to compare" % PREFIX)
        return None
    mujoco.mj_forward(model, data)
    d = data.xpos[b2] - data.xpos[b1]
    got = float((d @ d) ** 0.5)
    ok = abs(got - want) <= 1e-6
    print(f"geometry check: scene {'OK' if ok else 'MISMATCH'} — arms {got:.4f} m apart, "
          f"{path} says {want:.4f} m")
    if not ok:
        print("  !! sim and bench disagree: regenerate the scene with tools/make_two_arm_scene.py, "
              "or fix cell_geometry.json — nothing else will notice")
    return ok


def safe_terminate(client):
    """roslibpy 2.x raises AttributeError on terminate() when the Twisted loop never started; that
    traceback at exit hides the real reason a run ended, so swallow it."""
    try:
        client.terminate()
    except Exception as exc:                                   # noqa: BLE001
        print(f"(rosbridge connection closed: {type(exc).__name__})")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("model", help="path to the MuJoCo scene.xml (single- or two-arm)")
    parser.add_argument("--host", default="192.168.1.185")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--both-arms", action="store_true",
                        help="the scene holds two arms: subscribe to /arm1/joint_states and /arm2/joint_states")
    parser.add_argument("--cell-geometry", default=os.path.join(
                            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "docker", "ws", "cell_geometry.json"),
                        help="the file that says where arm 2 stands (README §15 R3)")
    parser.add_argument("--no-geometry-check", action="store_true",
                        help="do not compare the scene's separation with the cell geometry file")
    parser.add_argument("--fps", type=float, default=60.0,
                        help="how often to redraw (default 60). The loop used to run flat out, which "
                             "pinned a core and made every other window on the PC stutter")
    parser.add_argument("--headless", action="store_true",
                        help="no viewer window: subscribe and print the rates only (use it next to the "
                             "pad, or to check the bridge without opening a second window)")
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

    # R3 lives in one file. The scene is generated from it, but a scene can be older than the file
    # (or come from another bench), and a mismatch has no symptom at all: both models just move.
    if args.both_arms and not args.no_geometry_check:
        check_geometry(model, data, args.cell_geometry)

    client = roslibpy.Ros(host=args.host, port=args.port)
    client.run()
    topics = [("/arm1/joint_states", 0)]
    if args.both_arms:
        topics.append(("/arm2/joint_states", 1))
    latest = [{}, {}]                  # [arm][qpos_idx] = radians

    def on_message(msg, arm):
        names = msg.get("name", [])
        positions = msg.get("position", [])
        for want, q in mappings[arm].items():
            if want in names:
                latest[arm][q] = math.radians(positions[names.index(want)])

    subs = []                          # keep the Topic objects: roslibpy does not require it, but a
    for tname, arm in topics:          # subscription you cannot see is one you cannot unsubscribe
        sub = roslibpy.Topic(client, tname, "sensor_msgs/JointState")
        sub.subscribe(lambda msg, arm=arm: on_message(msg, arm))
        subs.append(sub)
    print("subscribed: " + ", ".join(t[0] for t in topics))

    if args.headless:
        period = 1.0 / max(args.fps, 1.0)
        t0, n0 = time.time(), 0
        print("headless: no viewer, Ctrl+C to stop")
        try:
            while True:
                time.sleep(period)
                n0 += 1
                if time.time() - t0 >= 5.0:                      # a heartbeat, so silence means dead
                    print("  %.1f Hz drawing, %d of %d joints seen"
                          % (n0 / (time.time() - t0), len(latest[0]) + len(latest[1]),
                             len(mappings[0]) + len(mappings[1])), flush=True)
                    t0, n0 = time.time(), 0
        except KeyboardInterrupt:
            pass
        finally:
            safe_terminate(client)
        return

    handle = viewer.launch_passive(model, data)
    period = 1.0 / max(args.fps, 1.0)
    frames = 0
    t_report = time.time()
    try:
        while handle.is_running():
            tick = time.time()
            for arm in range(len(mappings)):
                for i, angle in latest[arm].items():
                    data.qpos[i] = angle
            mujoco.mj_forward(model, data)
            handle.sync()
            frames += 1
            if time.time() - t_report >= 5.0:                     # if this line stops, the loop is stuck
                print("  %.0f fps, %d of %d joints seen"
                      % (frames / (time.time() - t_report), len(latest[0]) + len(latest[1]),
                         len(mappings[0]) + len(mappings[1])), flush=True)
                frames, t_report = 0, time.time()
            rest = period - (time.time() - tick)                  # the window had no throttle at all:
            if rest > 0:                                          # a busy loop here starves the pad
                time.sleep(rest)                                  # on a small PC
    finally:
        safe_terminate(client)
        handle.close()


if __name__ == "__main__":
    main()

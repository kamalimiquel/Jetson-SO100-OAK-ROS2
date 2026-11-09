# Scene camera calibration — status

> **2026-11-09, later:** the arm turned out to be an **SO-101**, not an SO-100 (ADR-0021).
> With the SO-101 URDF the closed-form residual dropped from **431 mm to 29 mm** on the
> first attempt (best kept subset 25 mm / 9°), and the bundle adjustment was re-run on
> `datasets/2026-11-09-so101`. The intrinsics below still stand. Everything further down
> describes the diagnosis against the wrong model and is kept as a record.

Dataset for the SO-101 run: `docs/calibration/datasets/2026-11-09-so101/` (12 shots,
`poses.json` + JPEGs, the operator's manual poses). Every step can be re-run from it.

Last touched: 2026-11-09. **Not finished: the extrinsic is not usable yet.**

| File | State |
|---|---|
| `scene_cam_intrinsics.yaml` | **good** — rms 0.17–0.24 px over 12–13 views; matches the factory EEPROM values (fx 1545 / cx 948) within 2 %, so it can be trusted |
| `scene_cam_extrinsic.yaml` | **invalid** — every solve leaves a residual of hundreds of mm; kept only as a record of what was tried |

## Method

Eye-to-hand (ADR-0011 amended): camera fixed to the base, ChArUco board
(`docs/charuco-board.pdf`, 5×5, 18 mm / 13.5 mm, DICT_4X4_50) mounted on the gripper.
The operator moves the arm by hand, `tools/handeye_grab.py` captures pose + frame,
`tools/handeye_solve.py` solves, `tools/handeye_refine.py` refines jointly.

Nothing ever commands joint 6: `/joint_targets` may carry **5 values**, and the joint
server then leaves the gripper servo untouched.

## What has been ruled out (with data, not opinion)

| Hypothesis | Evidence |
|---|---|
| degrees vs radians | fixed earlier: RSP gets `/joint_states_rad`; TF and `fk()` agree to the millimetre |
| joint order in `/joint_states` | verified: pan, lift, elbow, flex, roll, gripper |
| sign / permutation errors | 2⁵ signs × 5! permutations = 3 840 solves, best still 141 mm |
| bad intrinsics | factory EEPROM: fx 1545.4, cx 948.0 vs ours 1545–1571 / 941–948 |
| stale frame paired with fresh angles | grabber now requires two frames 1.2 s apart to agree within 2 px; unchanged residuals |
| board seen edge-on / too small | restricting to the 5 best-facing shots: still 213 mm |
| `/joint_states` carrying targets instead of positions | server reads `Present_Position` |
| eye-to-hand vs eye-in-hand convention | all four combinations tried, 400–1000 mm |
| camera moving with the arm | owner confirms the camera is fixed |

## Diagnosis

No rigid configuration explains the images together with the joint angles — even a
12-parameter bundle adjustment (camera pose + board mount, both free) bottoms out at
**64 px** of reprojection error, where a healthy calibration is under 1 px.

Single-joint pairs point at the model itself:

- `wrist_roll` turns the board **~10 % less** than the model predicts (26.8° vs 32.4°,
  67.4° vs 74.4°),
- the `shoulder_pan` axis is **28°** away from the model's axis.

The URDF is aligned frame-by-frame with the MuJoCo Menagerie model, so model and
simulator agree with each other — they just do not describe this physical unit well
enough for a millimetre-level extrinsic.

## Next step (when there is appetite for it)

Capture 12 shots moving **one joint at a time** (3 pan, 3 lift, 3 elbow, 3 wrist),
then:

1. estimate each joint's real rotation axis and scale from the image motion,
2. write the corrections into `docker/ws/arm/urdf/so100.urdf` (axis + a scale on the
   reported angle),
3. repeat a normal 12-pose sweep — the extrinsic should land in the millimetre range.

The captured datasets are in `~/calib/scene`, `~/calib/scene3` on the desktop
(`poses.json` + JPEGs); every step can be re-run from them without touching the robot.

## Working commands

```
uv run --with roslibpy --with "opencv-contrib-python-headless<5" --with numpy \
    python tools/handeye_grab.py                 # one sample, with guards
uv run --with "opencv-contrib-python-headless<5" --with numpy python tools/handeye_solve.py
uv run --with roslibpy --with "opencv-contrib-python-headless<5" --with numpy --with scipy \
    python tools/handeye_refine.py
```

Live camera view for framing: `http://<jetson-ip>:8090/` (grid + `/snapshot`).

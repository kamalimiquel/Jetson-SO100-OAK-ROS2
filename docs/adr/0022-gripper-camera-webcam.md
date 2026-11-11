# ADR-0022: Gripper camera — USB RGB webcam focused at 10-15 cm

- Status: Accepted — **amended 2026-11-11**: the webcam is mounted on the moving jaw and
  streams at 10 Hz (`/gripper/image/compressed`, 640x480) with its frame in the TF tree —
  `moving_jaw_so101_v1 → gripper_color_optical_frame` published once on `/tf_static` (nominal
  bracket offsets, `WRIST_TF_*` in compose, README §10.5). Still open: the *measured* focus and the
  measured offsets replacing the nominal ones
- Date: 2026-11-11
- Decided by: user
- Supersedes: ADR-0008 (wrist OAK-D Lite)
- Amends: ADR-0010 (mounting), ADR-0011 (calibration scope)

## Context

ADR-0008 chose an OAK-D Lite on the wrist for depth at the jaws. The working distance was assumed
to be compatible with its stereo range; it is not. The jaws and the object sit **10-15 cm** in front
of the lens, the Lite's minimum depth is well beyond that, and its lens is focused further out still.

The measurements in README §10.5 make the size of the problem concrete: two fixed-focus USB cameras
on this bench came out focused at opposite ends of the range — one sharpest at 10 cm or closer (444
at 10 cm, 75 at 15 cm), the other still improving at 15 cm (268 → 408) — and neither covered
10-15 cm. A fixed lens can only be mounted *at* its plane, so with hardware already in the drawer
the question is not which camera is "good" but where its plane happens to be.

So the requirement is not "a depth camera on the wrist". It is "a camera that is sharp at
10-15 cm", and that is a **focus** property, not a stereo property.

## Alternatives

1. **OAK-D Lite on the shared depthai node (ADR-0008 as it stood)** — Pros: depth at the wrist for
   servoing; one driver stack with the scene camera. Cons: fixed lens focused far beyond 10 cm, and a
   stereo minimum beyond the wrist distance as well, so at 10-15 cm neither the RGB nor the depth is
   usable. Rejected.
2. **Fixed-focus USB RGB webcam** — Pros: cheapest, one V4L2 node, UVC everywhere, lighter than the
   Lite. Cons: measured — units land anywhere in the range (one was sharp at ≤10 cm, another only
   beyond 15 cm); you cannot order the right one by price or by the words "fixed focus", and moving
   the mount invalidates the choice. Rejected as a design rule.
3. **USB RGB webcam with manual focus, set and locked at the working distance** — Pros: the focus
   plane goes exactly where the work happens (≈12.5 cm, mid-band), and it is verified with a number
   before the mount is built (README §10.5); plain UVC, no depthai, no USB3 budget, far lighter on a
   weak wrist servo. Cons: no depth at the wrist, so grasp refinement falls back to the scene cloud;
   focus can drift when the camera is handled and must be re-checked after mounting.

## Decision

**A USB RGB webcam with manual focus, mounted on the gripper and focused at 10-15 cm — set at
≈12.5 cm and locked** — driven by plain V4L2/UVC instead of depthai. The focus is proven with
`tools/camfocus.py` before the camera is mounted (README §10.5).

## Consequences

- `so100-cams` keeps depthai for the scene OAK-D only. The wrist feed is a separate UVC node
  publishing RGB; there is no depth, confidence or point cloud at the wrist.
- The `/oak/gripper/*` topics of `oak_ros_node.py` become **unused**: the multi-device code stays in
  the node for a possible second OAK, but nothing is published there any more.
- GraspGen keeps the **scene** cloud as its primary input (ADR-0013). The wrist image is for seeing
  the jaws and for pre-grasp verification — a 2D view, not a 3D sensor.
- Focus is a maintenance item: set it, record the score at 10 / 12.5 / 15 cm, and re-run the check
  whenever the camera is re-mounted, knocked or replaced.
- No wrist extrinsic is required for an RGB view. If the wrist image is ever used to localise an
  object (eye-in-hand), the procedure of ADR-0011 applies to this camera too.
- Simpler mount and cabling than ADR-0008's Lite + bracket, and no USB3 link to verify.

# SO-101 scene-camera dataset (2026-11-09)

12 hand-posed shots taken with `tools/handeye_grab.py` right after the URDF was
regenerated from the SO-101 MJCF (ADR-0021). Each shot passed the capture guards:
joints stationary, stream caught up, board facing the camera and large enough, and
no second copy of the board in view.

- `poses.json` — board spec, joint values (degrees), FK `base -> jaw`, corner count
- `pose_NN.jpg` — 1920x1080 colour frame from the scene OAK-D

Reproduce:

```
uv run --with "opencv-contrib-python-headless<5" --with numpy \
    python tools/handeye_solve.py --dir docs/calibration/datasets/2026-11-09-so101
uv run --with roslibpy --with "opencv-contrib-python-headless<5" --with numpy --with scipy \
    python tools/handeye_refine.py --dir docs/calibration/datasets/2026-11-09-so101
```

Results on this set: closed-form `base2gripper` 29.1 mm / 8.7 deg (best subset 25.4 mm);
intrinsics rms 0.18 px. Not yet good enough to grasp with — the remaining error is
spread across all shots, which points at the arm sagging under its own weight (it hangs
inverted) rather than at the camera.

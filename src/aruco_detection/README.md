# aruco_detection — legacy, not built

Superseded by [`perceptron_docking`](../perceptron_docking/). Kept for
reference only.

A `COLCON_IGNORE` file in this directory keeps `colcon` from building it, which
is why `colcon build` reports six packages and not seven. Do not remove that
file unless you intend to revive the package.

## Why it is not used

- Written for the Jetson's Python 3.6 / older OpenCV; the checked-in `build/`
  and `install/` directories are `python3.6` artefacts that will not work on
  Humble's Python 3.10.
- `arUcoPosePublisher.py` hardcodes a camera calibration path that only exists
  on one machine:
  `/mnt/usbdrive/jetson-home/camera/calibration_data/intrinsics_2.yml`.
- It publishes marker poses on `/aruco/pose` in the **camera frame**, with no TF
  into the robot frame — the caller has to know the camera geometry.
- It has no notion of the marker's normal, so it cannot support a square
  approach to the dock.
- It bundles its own copy of `transformations.py` rather than depending on
  `tf_transformations`.

## What replaced it

`perceptron_docking/aruco_detector_node.py` does the same job and adds:

- Pose published in `base_footprint` via TF, so consumers need no camera
  geometry, and withheld entirely if the transform is unavailable rather than
  published in the wrong frame.
- Intrinsics from `/camera/camera_info` instead of a file path.
- Sub-pixel corner refinement, a single reused detector object, an annotated
  debug image, and a per-frame detection flag.
- Everything configurable through `docking_params.yaml`.

## If you want something from it

`arUcoBase.py` holds the original standalone `ArucoCamera` capture-and-detect
class, which is a reasonable starting point for a non-ROS test script on the
Jetson. Read it, do not build it.

## Removing it

Once you are sure nothing here is needed:

```bash
rm -rf src/aruco_detection
```

Nothing else in the workspace depends on it.

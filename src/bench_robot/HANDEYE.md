# Automated eye-in-hand calibration

`handeye_auto` moves the arm through board-centred views, captures observations,
and calls the existing multi-method solver. The camera must be rigidly attached
to the arm; the ChArUco board and robot base must stay fixed for the whole run.

## Setup

1. Start the normal robot driver, joint states, robot TF, camera, and MoveIt
   `/move_action` and `/execute_trajectory` servers. Stop other arm-commanding
   scanners/managers for this run. Camera and robot timestamps must share a clock.
2. Fix the board in a clear reachable area, with the board/support and nearby
   obstacles represented in the MoveIt planning scene. Collision checking covers
   the modelled scene. The 20 cm displacement limit bounds target positions, not
   every intermediate link position along a planned trajectory.
3. Position the arm once so the complete board is visible with space around its
   edges. Alternatively place the board in the stationary camera's existing view.
   All subsequent calibration positions and orientations are automatic.
4. Specify the **actual printed board** dimensions. Defaults are 7 × 5 squares,
   30 mm squares, 22 mm markers, `DICT_4X4_50`. Use calibrated color CameraInfo
   matching the image resolution and optical frame; plumb-bob and rational
   distortion models are supported. OpenCV needs `aruco.CharucoDetector`.

Build and source from the workspace:

```bash
cd /home/thiwa/CEAbot
colcon build --packages-select bench_robot --symlink-install
source install/setup.bash
```

`arm_controlling` must also already be built and sourced. If needed, build it
along with `bench_robot` using the workspace's usual build procedure.

## Camera frame in this repository

The capture defaults use `/gemini336/color/image_raw` and
`/gemini336/color/camera_info`. The current arm URDF mounts
`gemini305_color_optical_frame`; the old Gemini 336 mounting joint is commented
out in `gen3_macro.xacro`.

By default the routine looks for an approximate TF from `end_effector_link` to
the image's optical frame. If the driver and URDF use different names for the
**same physical arm-mounted color camera**, supply
`--camera-mount-frame gemini305_color_optical_frame`. Only use this override when
that URDF frame approximates the installed camera's position and optical-axis
orientation. Otherwise correct the approximate mounting TF first. This TF is
used to aim the camera; it is not used as the measured hand-eye answer.

Preview target poses without moving the arm (requires live stationary robot,
TF, and camera; saves one seed observation plus `auto_run.yaml`):

```bash
ros2 run bench_robot handeye_auto \
  --camera-mount-frame gemini305_color_optical_frame \
  --squares-x 7 --squares-y 5 --square-length 0.030 --marker-length 0.022
```

The preview checks predicted board framing and displacement. It does **not**
plan trajectories or establish reachability. An execution run obtains a new seed
and plans each target from the current robot state immediately before moving.

Run automatic movement, capture, and solve:

```bash
ros2 run bench_robot handeye_auto --execute \
  --camera-mount-frame gemini305_color_optical_frame \
  --squares-x 7 --squares-y 5 --square-length 0.030 --marker-length 0.022 \
  --samples 20 --candidates 32
```

Omit `--camera-mount-frame` when the image frame already has the correct
approximate mount TF. For a driver actually publishing under `/gemini305`, also
set `--image-topic /gemini305/color/image_raw` and
`--camera-info-topic /gemini305/color/camera_info`.

## Capture behaviour and tuning

- Views orbit the observed board centre, changing tilt about multiple axes,
  roll, and distance. Defaults: 15° tilt, ±12° roll, ±8% distance variation,
  0.20 m maximum end-effector target displacement from the seed, and 10% velocity
  and acceleration scaling. Use `--tilt-deg`, `--roll-deg`,
  `--distance-variation`, `--max-displacement`, `--velocity-scaling`, and
  `--acceleration-scaling` to tune these.
- Each successful movement must finish and show stable, fresh measured joint
  positions for `--settle-time` (1 s). Missing velocities do not bypass this
  position-based check. Missing/stale feedback or execution failure stops the run.
- Capture requires three distinct fresh frames, at least eight ChArUco corners,
  the estimated complete board perimeter at least `--image-margin` (30 px) inside
  the image, and reprojection RMSE ≤1 px. This checks geometric framing and
  detected corners; it cannot guarantee every square is unoccluded.
- The image timestamp must follow settling, and capture uses exact timestamped
  robot TF. There is no latest-TF fallback in automatic mode. Tune
  `--max-sensor-age` (0.5 s), `--capture-timeout` (10 s), and
  `--max-reprojection-rmse` when needed. Do not loosen freshness to compensate
  for unsynchronised clocks.
- Unreachable or out-of-view targets are skipped. If too few remain, reposition
  the board/seed or reduce tilt and try again. The solver requires at least ten
  usable observations and rotation diversity. If the requested count is not
  reached, it warns and attempts a solve with the available observations.
- Board visibility is checked at capture poses. MoveIt may turn the camera away
  during travel. Visibility at every future pose cannot be guaranteed from an
  approximate mount transform; failed views are never saved as calibration data.
- Ctrl-C requests cancellation of active execution. Keep the robot's physical
  stop accessible. On completion the arm stays at the last pose.

## Results

Each run creates `/home/thiwa/scan_data/handeye_calibration/handeye_<timestamp>/`
with capture PNGs, `observations.yaml`, `auto_run.yaml` (targets and outcomes),
and, after solving, `handeye_solution.yaml`. `--output-dir` changes the root.

The best result is `methods[best_method].end_effector_from_camera`: the camera
**optical frame expressed in `end_effector_link`**, in metres and xyzw quaternion.
At any arm pose, `base_from_camera = base_from_end_effector @
end_effector_from_camera`. This estimates the optical frame, not the camera
housing or its mounting screw. The run does not modify URDF or publish a new TF.

Exit codes: 0 = preview completed or solver accepted; 1 = run/solver failure;
2 = solved but residual thresholds failed; 130 = interrupted. An acceptance uses
fixed-board consistency across the calibration observations (defaults 5 mm and
1° RMS). It is not an independent accuracy measurement. Validate with separate
observations before applying the transform. The existing manual
`handeye_capture` and `handeye_solve SESSION_DIR` commands remain available.

The view diversity check follows OpenCV's requirement for rotations about at
least two nonparallel axes:
[OpenCV hand-eye calibration](https://docs.opencv.org/4.12.0/d9/d0c/group__calib3d.html).

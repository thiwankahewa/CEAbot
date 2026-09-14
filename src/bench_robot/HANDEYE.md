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

## End-effector camera connected to the Raspberry Pi

The latest terminal sequence is saved in [Test/commands.txt](Test/commands.txt),
under “ChArUco calibration: latest terminal commands (2026-09-09)”. The current
printed board was detected as **5 × 7**, `DICT_4X4_50` (17 markers, 24 corners).
The 30 mm square and 22 mm marker lengths still need physical verification.
Manual HTTP capture uses a 1 s image-age limit in that sequence because measured
transfer plus detection was approximately 0.48 s; timestamped TF is still required.
The last session was blocked by a crashed `ros2_control_node` following a
`BaseCyclicClient::Refresh` timeout. Restart the existing robot launch and verify
`base_link` to `end_effector_link` TF before retrying capture.

For manual calibration with minimal network/encoding load, use **capture on
demand**. After copying and starting the bridge as described below, run on Jetson:

```bash
source /home/thiwa/CEAbot/install/setup.bash
ros2 run bench_robot handeye_capture --rpi-url http://10.20.0.200:8081 --no-preview --remote-max-age 1.0 \
  --squares-x 5 --squares-y 7 --square-length 0.030 --marker-length 0.022
```

Keep the board fixed and arm stationary, then press **Enter** in the terminal.
Each Enter makes one `/capture` request. The Pi waits up to one second for an
image stamped after the request, encodes it, and returns it. The Jetson detects
ChArUco and saves only if corners, freshness, and exact timestamped robot TF
checks pass. **q + Enter** quits. There is no window or background polling.
Use the printed board's actual dimensions. Both machines still need synchronized
clocks. The Pi camera driver still runs; on-demand mode reduces encoding and
network traffic, not the driver's acquisition load.

**If an older bridge is already running, recopy `handeye_remote.py` and restart
that process** before using `--no-preview`. The updated bridge encodes only on
HTTP requests, even when a preview client is used. Stop other preview clients
to eliminate their requests. `handeye_auto` retains repeated frame checks and
does not accept this manual-only option.

The generic HTTP 503 wording about high load is not a load measurement. This
bridge returns 503 when matching fresh RGB/CameraInfo is unavailable or its
encoder is busy. Verify the Pi's `/gemini336/color/image_raw` and
`/gemini336/color/camera_info`, their matching dimensions/frame IDs, and camera
timestamps if captures continue returning 503.

The current bridge reports the specific rejection reason in the Pi terminal
and in the Jetson's 503 warning. For diagnostics without requesting or encoding
an image, run on the Pi (omit the Authorization header if no token is configured):

```bash
curl -sS --max-time 3 -H "Authorization: Bearer $CEABOT_CAPTURE_TOKEN" \
  http://127.0.0.1:8081/status | python3 -m json.tool
```

This reports received image count, actual topics, frame IDs, resolutions, and
image age relative to the Pi ROS clock. Missing images or CameraInfo require
checking the camera driver and ROS environment/domain. A large positive or
negative image age requires checking the camera timestamp source and Pi clock;
do not hide it by substituting HTTP arrival time for image acquisition time.
Frame/resolution mismatches require CameraInfo from the same color stream.

The normal Pi capture service on port 8080 transfers scan archives; it does not
provide live images to hand-eye capture. Use the separate RGB preview bridge on
port **8081**. It subscribes to the Pi's existing ROS camera driver, so leave
`orbbec-camera.service` running. No second camera driver or cross-machine DDS
configuration is needed. The Gemini 305 currently uses `/gemini336` topic names
on the Pi; the bridge preserves the image's actual optical frame ID.

Copy the standalone bridge from the Jetson to the Pi (requires SSH access):

```bash
scp /home/thiwa/CEAbot/src/bench_robot/bench_robot/handeye_remote.py \
  thiwa@10.20.0.200:/home/thiwa/handeye_remote.py
```

On the **Pi**, source its ROS environment and start the bridge:

```bash
source /home/thiwa/CEAbot_Rpi/systemd_boot_setup/ros_jazzy.bash
python3 /home/thiwa/handeye_remote.py \
  --image-topic /gemini336/color/image_raw \
  --camera-info-topic /gemini336/color/camera_info
```

The bridge needs the Pi's ROS `rclpy`, `sensor_msgs`, `cv_bridge`, NumPy, and
OpenCV; it does not need the full robot workspace or OpenCV ChArUco support.
If `CEABOT_CAPTURE_TOKEN` is set in its terminal, set the same environment
variable in the Jetson terminal. Credentials are not passed as command arguments.
The preview bridge reads this variable independently of the capture service.
Pi and Jetson clocks must be synchronized, and camera timestamps must be in that
same clock domain. The client keeps original acquisition stamps, rejects stale
or future frames, and requires exact timestamped robot TF even in manual mode.

On the **Jetson**, with robot joint states and TF running:

```bash
source /home/thiwa/CEAbot/install/setup.bash
ros2 run bench_robot handeye_capture --rpi-url http://10.20.0.200:8081 \
  --squares-x 5 --squares-y 7 --square-length 0.030 --marker-length 0.022 \
  --dictionary DICT_4X4_50
```

Use the dimensions of the actual printed board. Keep it fixed, wait for the arm
to stop, and press **Space** in the Jetson preview window to save an observation;
**Q** quits. READY indicates detected corners and a fresh image with intrinsics;
Space can still be rejected if timestamped robot TF is missing. Files go into
the usual Jetson hand-eye session directory. `handeye_solve SESSION_DIR` works
unchanged. The bridge supplies RGB only; **C** cloud validation is unavailable
in HTTP mode. Omit `--rpi-url` to use the existing ROS image/cloud input.

For an automatic **preview without movement**, use:

```bash
ros2 run bench_robot handeye_auto --rpi-url http://10.20.0.200:8081 \
  --camera-mount-frame gemini305_color_optical_frame \
  --squares-x 5 --squares-y 7 --square-length 0.030 --marker-length 0.022
```

After checking the preview and the setup requirements above, adding `--execute`
enables automatic motion. The Pi input uses the same freshness, settling,
visibility, and timestamped TF checks as ROS input.

The bridge encodes full-resolution lossless PNGs only on request, with preview
encoding capped at 5 Hz by default; a preview client polls at 5 Hz. Use server
`--rate` and client `--remote-rate` to tune preview load. Manual `/capture`
requests bypass the preview cache and rate limit to obtain a new frame.
`--remote-timeout` defaults to 2 s and `--remote-max-age` to 0.5 s. Automatic mode
also applies `--max-sensor-age` (0.5 s). A stalled feed is shown as NOT READY and
cannot be saved. Check connectivity, authentication, camera topics, and clock
synchronization before increasing age limits. No scan archives are created by
preview requests.

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
  --squares-x 5 --squares-y 7 --square-length 0.030 --marker-length 0.022
```

The preview checks predicted board framing and displacement. It does **not**
plan trajectories or establish reachability. An execution run obtains a new seed
and plans each target from the current robot state immediately before moving.

Run automatic movement, capture, and solve:

```bash
ros2 run bench_robot handeye_auto --execute \
  --camera-mount-frame gemini305_color_optical_frame \
  --squares-x 5 --squares-y 7 --square-length 0.030 --marker-length 0.022 \
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

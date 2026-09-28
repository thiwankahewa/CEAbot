# Plant row detection

`bench_robot/launch/robot.launch.py` starts `plant_row_coordinates` with
`bench_robot/config/plant_row_coordinates.yaml`. After saving a top-view capture,
`top_scan` requests the `plant_row_coordinates` state. The detector must already
be running to receive the capture topics and state trigger.

The configured YOLO model is `models/plant_top_v1.pt`, trained with Ultralytics
8.4.47. The ROS executable uses `/usr/bin/python3`; installing YOLO in another
virtual environment does not install it for this executable. `colcon build`
does not install these optional ML dependencies.

## Jetson Orin / JetPack 6.1 setup

This setup uses Python 3.10, CUDA 12.6, NumPy 1.26.4 and the existing
`opencv-contrib-python` installation. Keep NumPy below 2 for this ROS Humble
`cv_bridge` environment. OpenCV contrib already provides `cv2`, so install
Ultralytics without dependencies to avoid installing a competing OpenCV wheel.

Install the Jetson wheels and supporting packages:

```bash
/usr/bin/python3 -m pip install --user 'numpy==1.26.4' 'sympy==1.13.1' \
  'https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-2.5.0a0%2B872d972e41.nv24.08-cp310-cp310-linux_aarch64.whl' \
  'https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.20.0a0%2Bafc54f7-cp310-cp310-linux_aarch64.whl' \
  'nvidia-cusparselt-cu12==0.6.2' 'polars==1.44.2' 'ultralytics-thop==2.2.1'
/usr/bin/python3 -m pip install --user --no-deps 'ultralytics==8.4.47'
```

The Jetson PyTorch wheel also needs `libcusparseLt.so.0`. Expose the user-installed
NVIDIA runtime through PyTorch's existing library search path:

```bash
/usr/bin/python3 - <<'PYTHON'
from importlib.metadata import distribution
from pathlib import Path
runtime = distribution('nvidia-cusparselt-cu12')
library = next(runtime.locate_file(f) for f in runtime.files
               if str(f).endswith('/libcusparseLt.so.0'))
link = Path(distribution('torch').locate_file('torch/lib/libcusparseLt.so.0'))
if not link.exists():
    link.symlink_to(library)
print(link, '->', link.resolve())
PYTHON
```

Repeat the link step if reinstalling PyTorch removes the link. This leaves the
system CUDA installation unchanged. The other Ultralytics runtime dependencies
(matplotlib, Pillow, PyYAML, requests, SciPy and psutil) are already installed on
this robot.

Verify the runtime before launching the robot:

```bash
/usr/bin/python3 -c 'import torch, torchvision, ultralytics; print(torch.__version__, torchvision.__version__, ultralytics.__version__); print(torch.cuda.is_available())'
```

Expected: PyTorch `2.5.0a0+872d972e41.nv24.08`, torchvision
`0.20.0a0+afc54f7`, Ultralytics `8.4.47`, and CUDA available `True`.

Check node startup without joining the robot's ROS domain:

```bash
cd /home/thiwa/CEAbot
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 ros2 run CEAbot_phenotyping plant_row_coordinates \
  --ros-args --params-file src/bench_robot/config/plant_row_coordinates.yaml
```

Look for `Row segmentation backend: yolo` and `Loaded row YOLO model:` with the
configured path. Stop this isolated check with Ctrl+C. Restart the normal robot
launch after fixing dependencies; a detector that exited at startup does not
restart itself. Capture a new top view because the original capture topics are
not retained for a newly started detector.

Setup references: [Ultralytics Jetson guide](https://docs.ultralytics.com/guides/nvidia-jetson/)
and [NVIDIA PyTorch for Jetson installation](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html).

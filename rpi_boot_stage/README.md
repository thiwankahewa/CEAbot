# Gemini 336 capture spool over the Gen3 expansion Ethernet

The Pi captures and compresses every view locally. It does **not** transfer a
view during the scan. After `/individual_scan_done` is published, the Jetson
downloads all pending archives in resumable chunks and installs them in the
original scan folders. The next capture automatically pauses transfer; the next
row-complete signal resumes it.

Each installed view contains:

```text
<run_dir>/plant_<ID>/<view>/
├── color.png
├── depth.npy
├── cloud_xyzrgb.npy
└── meta.yaml
```

`depth.npy` preserves the original `uint16` depth values. The canonical point
cloud is `cloud_xyzrgb.npy`; create a PLY later only when an external viewer
needs one, avoiding duplicate storage and transfer during acquisition.

## Raspberry Pi

Use Ubuntu Server ARM64 and synchronize the Pi and Jetson clocks with
chrony/NTP. Configure `eth0` as `10.20.0.200/24` and install the Orbbec ROS 2
wrapper, `cv_bridge`, `message_filters`, OpenCV, NumPy and PyYAML.

## Verified physical wiring

The installed 20-pin FFC reverses its pin order:

```text
Kinova pin 1  -> breakout pin 20
Kinova pin 20 -> breakout pin 1
breakout pin = 21 - Kinova pin
```

For a T568B Ethernet cable whose RJ45 end plugs into the Raspberry Pi:

| RJ45 pin | Cable color | Kinova signal | Kinova pin | Breakout pin |
|---:|---|---|---:|---:|
| 1 | white/orange | ETH_RX_P | 5 | 16 |
| 2 | orange | ETH_RX_N | 6 | 15 |
| 3 | white/green | ETH_TX_P | 8 | 13 |
| 6 | green | ETH_TX_N | 9 | 12 |

The blue and brown pairs are unused and individually insulated. The Pi is
powered independently; Kinova +24 V is not connected to it. Before reconnecting
after any wiring work, verify the mapping and absence of shorts with power off.

The working link reports:

```text
Speed: 100Mb/s
Duplex: Full
Auto-negotiation: off
Link detected: yes
```

The Kinova Web App expansion connection is configured for 100 Mbps/full duplex,
so the Pi is intentionally forced to the matching mode.

## Verified network topology

```text
Jetson:          192.168.1.2   (enP1p1s0)
Kinova external: 192.168.1.10
Kinova EXP:      10.20.0.1
Raspberry Pi:    10.20.0.200   (eth0)
```

Temporary commands used during commissioning were:

```bash
# Raspberry Pi
sudo ethtool -s eth0 speed 100 duplex full autoneg off
sudo ip address replace 10.20.0.200/24 dev eth0
sudo ip link set eth0 up
sudo ip route replace 192.168.1.0/24 via 10.20.0.1 dev eth0

# Jetson
sudo ip route replace 10.20.0.0/24 via 192.168.1.10 dev enP1p1s0
```

Successful commissioning checks were:

```bash
# Pi -> Kinova EXP
ping -I eth0 -c 4 10.20.0.1

# Jetson -> Pi through Kinova
ping -c 4 10.20.0.200
```

Both achieved 0% packet loss. These `ip` and `ethtool` commands are temporary;
install the boot services below to restore them automatically after reboot.

## Make both ends persistent across reboot

On the Raspberry Pi:

```bash
cd /home/thiwa/CEAbot_Rpi/systemd_boot_setup
sudo cp ceabot-expansion-network.service /etc/systemd/system/
sudo cp orbbec-camera.service /etc/systemd/system/
sudo cp ceabot-capture.service /etc/systemd/system/

sudo mkdir -p /var/lib/ceabot-captures
sudo chown thiwa:thiwa /var/lib/ceabot-captures
sudo chmod 750 /var/lib/ceabot-captures

sudo systemctl daemon-reload
sudo systemctl enable --now ceabot-expansion-network.service
sudo systemctl enable --now orbbec-camera.service
sudo systemctl enable --now ceabot-capture.service
```

On the Jetson:

```bash
cd /home/thiwa/CEAbot_Rpi/systemd_boot_setup
sudo cp jetson-ceabot-rpi-route.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now jetson-ceabot-rpi-route.service
```

After rebooting both machines, verify:

```bash
# Pi
ip -br address show eth0
sudo ethtool eth0 | grep -E "Speed|Duplex|Auto-negotiation|Link detected"
systemctl --no-pager --full status ceabot-expansion-network orbbec-camera ceabot-capture

# Jetson
ip route get 10.20.0.200
ping -c 4 10.20.0.200
curl -H "Authorization: Bearer $CEABOT_CAPTURE_TOKEN" \
  http://10.20.0.200:8080/health
```

The expected health response has `ok: true`. A positive `frames_received`
value confirms the Orbbec driver is publishing synchronized RGB, depth and
registered colored point-cloud messages.

The registered colored point-cloud topic is required:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/orbbec_ws/install/setup.bash

ros2 launch orbbec_camera gemini_330_series.launch.py \
  camera_name:=gemini336 \
  depth_registration:=true \
  align_mode:=SW \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=true
```

Create a writable spool directory. An NVMe SSD is strongly preferred because
RGB-D point-cloud archives are large:

```bash
sudo mkdir -p /var/lib/ceabot-captures
sudo chown "$USER":"$USER" /var/lib/ceabot-captures
```

Start the server:

```bash
export CEABOT_CAPTURE_TOKEN='replace-with-a-long-random-token'
python3 /home/thiwa/CEAbot_Rpi/rpi_orbbec_capture_server.py \
  --bind 10.20.0.200 \
  --spool-dir /var/lib/ceabot-captures \
  --depth-scale-m-per-unit 0.001
```

Health check from the Jetson:

```bash
curl -H "Authorization: Bearer $CEABOT_CAPTURE_TOKEN" \
  http://10.20.0.200:8080/health
```

## Jetson

Disable the Jetson-side Gemini 336 launch and the old `orbbec_test_scan` node.
Build and run the proxy:

```bash
cd /home/thiwa/CEAbot
colcon build --packages-select arm_controlling
source install/setup.bash

export CEABOT_CAPTURE_TOKEN='replace-with-the-same-token'
ros2 run arm_controlling remote_orbbec_capture --ros-args \
  -p rpi_url:=http://10.20.0.200:8080 \
  -p auth_token:="$CEABOT_CAPTURE_TOKEN" \
  -p chunk_bytes:=1048576
```

`plant_view_scanner` continues using `/orbbec_test_scan/capture_view`. A capture
returns after the Pi has safely written its compressed archive. The Jetson
creates `meta.yaml` immediately so timestamped TF/pose metadata can still be
recorded; it merges that metadata when the full archive arrives.

The proxy automatically starts/resumes transfer when it receives
`/individual_scan_done`. Manual controls are also available:

```bash
ros2 service call /remote_orbbec_transfer/start std_srvs/srv/Trigger '{}'
ros2 service call /remote_orbbec_transfer/pause std_srvs/srv/Trigger '{}'
ros2 service call /remote_orbbec_transfer/status std_srvs/srv/Trigger '{}'
```

Pi archives are deleted only after the Jetson verifies the complete archive,
extracts all files, merges metadata and acknowledges successful installation.
If the connection fails, the partial archive remains on both machines and the
next start/row-complete command resumes it.

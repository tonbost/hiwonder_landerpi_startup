# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Remote setup tool for HiWonder LanderPi robot on Raspberry Pi 5. Uses Fabric to execute commands over SSH, with state tracking to resume interrupted deployments.

## Commands

```bash
# Install dependencies
uv sync

# Run comprehensive health check (recommended first)
uv run python checkLanderPi.py check --host <IP> --user <USER> --password <PASS>

# Quick health check (critical systems only)
uv run python checkLanderPi.py quick --host <IP> --user <USER> --password <PASS>

# Test SSH connection to Pi
uv run python setup_landerpi.py connect --host <IP> --user <USER> --password <PASS>

# Deploy full setup (resumes if interrupted)
uv run python setup_landerpi.py deploy --host <IP> --user <USER> --password <PASS>

# Dry run (preview commands without executing)
uv run python setup_landerpi.py deploy --host <IP> --user <USER> --dry-run

# Reset state and start over
uv run python setup_landerpi.py deploy --host <IP> --user <USER> --reset

# Skip Docker/ROS2 installation
uv run python setup_landerpi.py deploy --host <IP> --user <USER> --skip-docker

# Migrate SD card to NVMe drive (faster boot and I/O)
uv run python migrate_to_nvme.py status              # Check NVMe readiness and current state
uv run python migrate_to_nvme.py clone --yes         # Install rpi-clone and clone SD to NVMe
uv run python migrate_to_nvme.py configure --yes     # Set boot order and enable PCIe Gen 3
uv run python migrate_to_nvme.py verify              # Verify NVMe boot after reboot
uv run python migrate_to_nvme.py update-eeprom --yes # Update bootloader firmware (optional)

# Test chassis motion - DIRECT (no ROS2 needed, uses config.json)
uv run python validation/test_chassis_direct.py test --direction all --duration 2 --yes  # Full test (6 directions)
uv run python validation/test_chassis_direct.py test --duration 3 --yes  # Forward + backward only
uv run python validation/test_chassis_direct.py test --direction turn_right --duration 2 --yes
uv run python validation/test_chassis_direct.py test --direction strafe_left --duration 2 --yes

# Test individual motors (for debugging motor mapping)
uv run python validation/test_chassis_direct.py motor-test
uv run python validation/test_chassis_direct.py motor-test --motor 1

# Test chassis motion - via ROS2 (requires deployed stack, uses config.json)
uv run python validation/test_chassis_motion_ros2.py test --direction all --distance 1.0 --yes  # Full test (all 6 movements)
uv run python validation/test_chassis_motion_ros2.py test --direction forward --distance 0.5 --yes
uv run python validation/test_chassis_motion_ros2.py test --direction strafe_left --distance 0.5 --yes
uv run python validation/test_chassis_motion_ros2.py test --direction turn_right --yes

# Emergency stop
uv run python validation/test_chassis_direct.py stop

# Get robot status (battery, serial port)
uv run python validation/test_chassis_direct.py status

# Test lidar - check device, Docker, and ROS2 connectivity
uv run python validation/test_lidar.py check

# Start lidar driver in Docker container
uv run python validation/test_lidar.py start-driver

# Read lidar scan data (requires driver running)
uv run python validation/test_lidar.py scan --samples 5

# Stop lidar driver container
uv run python validation/test_lidar.py stop-driver

# Test lidar modes (robot will MOVE! requires HiWonder ROS2 workspace)
uv run python validation/test_lidar.py test --mode 1 --duration 10 --yes  # Obstacle avoidance
uv run python validation/test_lidar.py test --mode 2 --duration 10 --yes  # Tracking (35cm following)
uv run python validation/test_lidar.py test --mode 3 --duration 10 --yes  # Guard (face objects)

# Stop all lidar functionality
uv run python validation/test_lidar.py stop

# Test robotic arm - DIRECT (no ROS2 needed, uses config.json)
uv run python validation/test_arm.py test --yes  # Full arm test (positions + gripper)
uv run python validation/test_arm.py test --duration 3 --yes  # Custom movement duration

# Get arm servo status (positions, voltages, temperatures)
uv run python validation/test_arm.py status

# Move arm to home position (all servos at 500)
uv run python validation/test_arm.py home --yes

# Test individual servos
uv run python validation/test_arm.py servo-test --servo 1 --yes  # Test servo 1
uv run python validation/test_arm.py servo-test --yes  # Test all servos (1-5, 10)

# Emergency stop (disable all arm servos)
uv run python validation/test_arm.py stop

# Deploy persistent ROS2 stack (uses config.json, survives reboots)
uv run python deploy_ros2_stack.py deploy    # Upload files and start stack
uv run python deploy_ros2_stack.py stop      # Stop the ROS2 stack
uv run python deploy_ros2_stack.py logs      # View container logs
uv run python deploy_ros2_stack.py logs -f   # Follow logs in real-time

# Test sensors via ROS2 stack (requires deploy_ros2_stack.py deploy)
uv run python validation/test_arm_ros2.py test --yes        # Arm via ROS2 topics
uv run python validation/test_arm_ros2.py home --yes        # Home position
uv run python validation/test_arm_ros2.py status            # Arm state
uv run python validation/test_lidar_ros2.py check           # Lidar status
uv run python validation/test_lidar_ros2.py scan --samples 5  # Read scan data
uv run python validation/test_cameradepth_ros2.py check     # Camera topics
uv run python validation/test_cameradepth_ros2.py stream    # Read streams

# Autonomous exploration - ON-ROBOT (fast, recommended)
uv run python deploy_explorer.py deploy                    # Upload explorer to robot (one time)
uv run python deploy_explorer.py start --duration 10       # Start exploration (streams output)
uv run python deploy_explorer.py start --duration 5 --speed 0.35  # Custom speed
uv run python deploy_explorer.py start --yolo --duration 10       # With YOLO object detection (CPU, 2-5 FPS)
uv run python deploy_explorer.py start --yolo-hailo --duration 10 # With YOLO + Hailo-8 acceleration (25-40 FPS)
uv run python deploy_explorer.py start --yolo --yolo-logging --duration 10  # With YOLO + debug logging (images saved to ~/yolo_logs)
uv run python deploy_explorer.py start --yolo-hailo --yolo-logging --duration 10  # Hailo + debug logging
uv run python deploy_explorer.py start --rosbag --duration 10     # With ROS2 bag recording
uv run python deploy_explorer.py start --yolo-hailo --rosbag --duration 15  # Combined options
uv run python deploy_explorer.py stop                      # Stop exploration
uv run python deploy_explorer.py status                    # Check robot status
uv run python deploy_explorer.py logs                      # View exploration logs
uv run python deploy_explorer.py logs -f                   # Follow logs in real-time

# Autonomous exploration - REMOTE (legacy, slower due to SSH overhead)
uv run python validation/test_exploration.py status                    # Check prerequisites
uv run python validation/test_exploration.py start --duration 5 --yes  # Start (slower ~7 samples/5min)
uv run python validation/test_exploration.py stop                      # Emergency stop

# Voice Control (runs ON the robot, requires WonderEcho Pro + ROS2 stack)
uv run python deploy_voicecontroller.py deploy   # Full deployment (script + deps + service)
uv run python deploy_voicecontroller.py upload   # Upload script only
uv run python deploy_voicecontroller.py install-service  # Install systemd service
uv run python deploy_voicecontroller.py test     # Run check on robot

# Systemd service management (auto-starts on boot):
# sudo systemctl start tars-voice    # Start voice control
# sudo systemctl stop tars-voice     # Stop voice control
# sudo systemctl status tars-voice   # Check status
# journalctl -u tars-voice -f        # View logs

# On the robot directly:
# python3 ~/robot_voicecontroller.py check       # Check components
# python3 ~/robot_voicecontroller.py listen      # Single voice command
# python3 ~/robot_voicecontroller.py loop        # Continuous mode (always listening)
# python3 ~/robot_voicecontroller.py loop --wake-word  # Wake word mode ("Hey TARS")
# python3 ~/robot_voicecontroller.py test-tts "Hello"  # Test TTS
# python3 ~/robot_voicecontroller.py test-llm "go forward"  # Test LLM command generation
```

## Architecture

**`checkLanderPi.py`** - Health check tool using Typer
- `LanderPiChecker` class runs diagnostic checks via SSH
- `HealthReport` dataclass aggregates all check results
- Two modes: `check` (comprehensive) and `quick` (critical only)
- Exit codes: 0 = all OK, 1 = warnings, 2 = errors

**`setup_landerpi.py`** - Main CLI tool using Typer
- `RemoteSetupManager` class handles SSH connections via Fabric
- State tracking: Creates marker files in `~/.landerpi_setup/` on remote Pi
- `is_step_done()`/`mark_step_done()` enable idempotent, resumable deployments

**`migrate_to_nvme.py`** - SD to NVMe migration tool
- Automates cloning SD card to NVMe for faster boot and I/O
- Uses `rpi-clone` for safe filesystem cloning
- Configures bootloader (BOOT_ORDER=0xf416, PCIE_PROBE=1) for NVMe boot priority
- Enables PCIe Gen 3 in `/boot/firmware/config.txt` for maximum NVMe speed
- Commands:
  - `status` - Check NVMe detection, boot device, space requirements
  - `clone` - Install rpi-clone and clone SD to NVMe
  - `configure` - Set boot order and enable PCIe Gen 3
  - `verify` - Verify NVMe boot with speed test
  - `update-eeprom` - Update bootloader firmware to latest version
- BOOT_ORDER codes: 6=NVMe, 1=SD, 4=USB, f=Loop (read right-to-left)
- Typical workflow: `status` → `clone` → `configure` → reboot → `verify`

**`validation/test_chassis_direct.py`** - Direct chassis motion test (NO ROS2 REQUIRED)
- Uses `ros_robot_controller_sdk.py` for direct serial communication with STM32
- Speed: 0.3 m/s (max), configurable duration with `--duration`
- Reads credentials from `config.json` automatically
- Commands: `test` (move robot), `motor-test` (individual motors), `stop` (emergency stop), `status` (battery/IMU)
- Motor mapping: M1=front-left, M2=back-left, M3=front-right, M4=back-right
- Forward motion: left side negative speed, right side positive speed

**`validation/test_chassis_motion.py`** - ROS2-based chassis motion test
- `ChassisMotionTest` class handles SSH + ROS2 velocity commands
- Requires deployed ROS2 stack (deploy_ros2_stack.py)
- Uses `/cmd_vel` topic with `geometry_msgs/msg/Twist`
- Commands: `test` (move robot), `stop` (emergency stop)

**`deploy_ros2_stack.py`** - ROS2 stack deployment (persistent across reboots)
- Uploads `ros2_nodes/`, `docker/`, `config/`, `drivers/` to `~/landerpi/` on robot
- Starts Docker Compose stack with `restart: unless-stopped`
- `cmd_vel_bridge` node converts `/cmd_vel` (Twist) to `/ros_robot_controller/set_motor`
- Commands: `deploy` (upload + start), `stop` (stop stack), `logs` (view logs)

**`validation/test_lidar.py`** - Lidar validation test (Docker-based ROS2)
- Tests MS200/LD19 lidar connectivity and functionality via Docker
- ROS2 runs inside Docker container (`ros:humble-ros-base` or `landerpi-ros2:latest`)
- Reads credentials from `config.json` automatically
- Lidar device: `/dev/ldlidar` or `/dev/ttyUSB0` (baud: 230400)
- ROS2 topic: `/scan`
- Commands:
  - `check` - Verify device, Docker, ROS2 availability
  - `start-driver` - Start lidar driver in Docker container
  - `scan` - Read scan data and display statistics
  - `stop-driver` - Stop lidar driver container
  - `test` - Test lidar modes (requires HiWonder ROS2 workspace)
  - `stop` - Stop all lidar functionality

**`validation/test_arm.py`** - Robotic arm test (NO ROS2 REQUIRED)
- Uses `ros_robot_controller_sdk.py` for direct serial communication with STM32
- Arm servos: IDs 1-5 (5-DOF arm) + ID 10 (gripper)
- Position range: 0-1000, with 500 as center/home position
- Reads credentials from `config.json` automatically
- Commands:
  - `test` - Run comprehensive arm test (positions + gripper)
  - `status` - Read all servo positions, voltages, temperatures
  - `home` - Move arm to home position (all servos at 500)
  - `servo-test` - Test individual servos or all servos
  - `stop` - Emergency stop (disable torque on all servos)

**`validation/test_arm_ros2.py`** - ROS2-based arm test (requires deployed stack)
- Uses `/arm/cmd` and `/arm/state` topics via `docker exec landerpi-ros2`
- Commands: `test`, `home`, `stop`, `status`

**`validation/test_lidar_ros2.py`** - ROS2-based lidar test (requires deployed stack)
- Uses `/scan` topic via `docker exec landerpi-ros2`
- Commands: `check`, `scan`, `status`

**`validation/test_cameradepth_ros2.py`** - ROS2-based camera test (requires deployed stack)
- Uses `/aurora/*` topics via `docker exec landerpi-ros2`
- Commands: `check`, `stream`, `status`

**`validation/test_exploration.py`** - Autonomous exploration test (requires ROS2 stack)
- Frontier-based exploration with sensor fusion (lidar + depth camera; depth camera planned)
- Uses `ExplorationController` from `validation/exploration/` module
- 10 Hz control loop with safety limits (battery cutoff, max runtime)
- Commands: `start`, `stop`, `status`
- Options: `--duration` (minutes), `--min-battery` (voltage), `--rosbag` (recording)

**`validation/exploration/`** - Exploration module
- `explorer.py` - `ExplorationController` main class, state machine (IDLE → EXPLORING → AVOIDING → STOPPED)
- `frontier_planner.py` - Frontier detection and goal selection from lidar scans
- `safety_monitor.py` - Battery monitoring, runtime limits, emergency stop
- `data_logger.py` - Exploration metrics and optional ROS2 bag recording
- `ros2_hardware.py` - Hardware interface with low-latency sensor bridge

**Sensor Bridge (Low-Latency Topic Reading)**

The sensor bridge provides ~10ms latency for reading ROS2 topics instead of ~400-500ms with subprocess-based `ros2 topic echo`.

**Architecture:**
```
Docker container:
  sensor_bridge_node → subscribes to /scan, /hazards, /depth_stats, /battery
                     → writes JSON to /landerpi_data/*.json (shared volume)

Host Python (robot_explorer.py):
  ROS2Hardware → reads ~/landerpi_data/*.json (~10ms latency)
```

**JSON Files:**
| File | Topic | Contents |
|------|-------|----------|
| `lidar.json` | /scan | ranges, angle_min, angle_increment |
| `hazards.json` | /hazards | hazard list with type, distance, angle |
| `depth_stats.json` | /depth_stats | min_depth_m, avg_depth_m, valid_percent |
| `battery.json` | /battery | voltage |

**Configuration (`ros2_hardware.py`):**
```python
ROS2Config(
    use_file_bridge=True,    # Enable file-based reading (default)
    data_dir="~/landerpi_data",
    file_stale_threshold=2.0  # Fallback to subprocess if data > 2s old
)
```

**Performance:**
| Metric | Subprocess | Sensor Bridge |
|--------|------------|---------------|
| Topic read latency | 400-500ms | 5-10ms |
| Control loop effective rate | ~2-3Hz | 10Hz |
| Obstacle reaction time | >1s | <200ms |

**Fallback:** If JSON files are missing or stale (>2s old), automatically falls back to subprocess-based reading.

**YOLO Mode for Autonomous Exploration**

YOLO mode enables YOLOv11 object detection during autonomous exploration, allowing the robot to identify and avoid semantic hazards (like people, pets, etc.) that lidar alone might not classify as obstacles.

**How it works:**
1. YOLO detector node subscribes to `/aurora/rgb/image_raw` (depth camera RGB feed)
2. Runs YOLOv11n inference on each frame to detect objects
3. Publishes detected hazards to `/hazards` topic (std_msgs/String, JSON format) if within `hazard_distance` (2.5m)
4. Publishes annotated images to `/yolo/annotated_image` for live visualization
5. Exploration controller fuses hazards with lidar data for enhanced obstacle avoidance
6. Semantic hazards within `semantic_stop_distance` (0.8m) trigger immediate stop

**Published Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/yolo/detections` | vision_msgs/Detection2DArray | Bounding boxes and class info |
| `/yolo/annotated_image` | sensor_msgs/Image | Live camera feed with bboxes drawn (~14 Hz) |
| `/hazards` | std_msgs/String | JSON hazards for exploration controller |

**Live Annotated Video Stream:**

The `/yolo/annotated_image` topic publishes camera frames with YOLO bounding boxes drawn:
- Red boxes: Hazard classes (person, dog, cat, etc.)
- Green boxes: Other detected objects
- Labels: Class name + confidence score

View with RViz by adding an Image display and setting topic to `/yolo/annotated_image`.

**Usage from local machine:**
```bash
uv run python deploy_explorer.py start --yolo --duration 10           # CPU YOLO (2-5 FPS)
uv run python deploy_explorer.py start --yolo-hailo --duration 10     # Hailo YOLO (25-40 FPS)
uv run python deploy_explorer.py start --yolo-hailo --yolo-logging --duration 10  # Hailo + debug logging
uv run python deploy_explorer.py start --yolo-hailo --rosbag --duration 15  # Combined options
```

**Usage directly on robot:**
```bash
python3 ~/landerpi/robot_explorer.py explore --duration 5 --yolo
```

**Prerequisites:**
1. ROS2 stack deployed: `uv run python deploy_ros2_stack.py deploy`
2. Explorer deployed: `uv run python deploy_explorer.py deploy`
3. Depth camera running (provides RGB feed for YOLO)
4. YOLOv11 model downloaded on robot (automatic on first run)

**Hazard Configuration (JSON):**

Edit `config/yolo_hazards.json` to add/remove hazard classes without changing code:
```json
{
  "hazard_classes": ["person", "dog", "cat", "cup", "bottle", "stop sign"],
  "hazard_distance": 2.5,
  "semantic_stop_distance": 0.8
}
```

After editing, redeploy: `uv run python deploy_ros2_stack.py deploy`

**Key thresholds:**
| Parameter | Value | Location |
|-----------|-------|----------|
| `hazard_distance` | 2.5m | `config/yolo_hazards.json` |
| `semantic_stop_distance` | 0.8m | `config/yolo_hazards.json` |

**Note:** `hazard_distance` (2.5m) > `semantic_stop_distance` (0.8m) ensures hazards are reported before the robot stops.

**Camera Health Monitoring:**

When running with `--yolo`, the explorer monitors camera health every 5 seconds:
- Shows camera status (OK/OFFLINE + Hz) in every status line
- Warns if camera stops publishing (Aurora 930 can crash during extended operation)
- Reports camera recovery with publish rate

Example output:
```
[12:34:56.789] Status: 9.4 min | moving forward | escape: NONE cam: OK (9.8Hz)
[12:35:01.234] [WARNING] Camera stopped publishing!
[12:35:01.235] Status: 9.3 min | stopped | escape: NONE cam: OFFLINE (0.0Hz)
[12:35:06.456] [INFO] Camera recovered (10.1 Hz)
```

**YOLO Debug Logging:**

When `--yolo-logging` is enabled, the YOLO node saves annotated images and detection history.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_logging` | false | Save annotated images |
| `log_dir` | ~/yolo_logs | Output directory |
| `log_all_frames` | false | Log all frames vs only detections |
| `max_log_images` | 500 | Auto-cleanup threshold |

**Log Output Structure:**
```
~/yolo_logs/session_YYYYMMDD_HHMMSS/
├── metadata.json           # Session config
├── detections.jsonl        # All detections with timestamps
└── images/
    └── frame_000001_*_det.jpg  # Annotated images with bboxes
```

**Usage:**
```bash
# View logged detections:
ssh tonbost@192.168.50.169 "ls ~/yolo_logs/"
ssh tonbost@192.168.50.169 "tail -10 ~/yolo_logs/session_*/detections.jsonl"

# Copy images locally:
scp -r tonbost@192.168.50.169:~/yolo_logs/session_* ./yolo_debug/
```

**Note:** Enabling YOLO logging restarts ROS2 stack (adds ~8 second delay at start)

**Timestamped Explorer Output:**

Explorer output includes timestamps `[HH:MM:SS.mmm]` for cross-referencing with YOLO logs:
```
[15:47:52.123] Obstacle at 0.08m - stopped (blocked: 1)
[15:48:01.789] HAZARD: bottle at 0.85m - STOPPING
[15:48:02.012] Status: 4.4 min remaining | action: stopped
```

Correlate with YOLO detections by converting Unix timestamps in `detections.jsonl`.

**Architecture:**
```
/aurora/rgb/image_raw → YOLO detector → /yolo/detections
                              ↓               ↓
                    /yolo/annotated_image   /hazards (JSON)
                              ↓               ↓
                           RViz    Exploration controller ← /scan (lidar)
                                              ↓
                                        /cmd_vel (motion)
```

## Hailo 8 AI Accelerator

The robot has a Hailo-8 NPU on PCIe for hardware-accelerated inference. Uses **HailoRT 4.23** with Python 3.13 support.

**Architecture (ZeroMQ Bridge):**

HailoRT requires Python 3.13 but ROS2 Humble uses Python 3.10. Solution: Run Hailo inference on host, communicate via ZeroMQ:

```
Host (Python 3.13 + HailoRT):
  └── hailo_inference_server.py → listens tcp://localhost:5555
                                → runs YOLOv11 on Hailo-8
                                → managed by systemd (hailo-server.service)

Docker container (ROS2 Humble, Python 3.10):
  └── yolo_hailo_bridge (ROS2 node) → subscribes /aurora/rgb/image_raw
                                    → sends images to host via ZeroMQ
                                    → receives detections
                                    → publishes to /hazards, /yolo/detections
```

**Setup:**
```bash
# Check hardware status
uv run python deploy_hailo8.py check

# Install HailoRT 4.23 driver (one-time)
uv run python deploy_hailo8.py install

# Deploy models, inference server, and systemd service
uv run python deploy_hailo8.py deploy

# Validate
uv run python deploy_hailo8.py test --benchmark

# Show device info
uv run python deploy_hailo8.py status

# Force reinstall (if upgrading from older version)
uv run python deploy_hailo8.py install --force

# Manual server control (normally auto-started by deploy_explorer.py)
uv run python deploy_hailo8.py start   # Start inference server
uv run python deploy_hailo8.py stop    # Stop inference server
uv run python deploy_hailo8.py logs    # View server logs
uv run python deploy_hailo8.py logs -f # Follow logs
```

**Key Points:**
- Uses Raspberry Pi **Trixie** repository for HailoRT 4.23
- **Python 3.13 supported** (Ubuntu 25.10 compatible)
- M.2 modules have firmware 4.19.0 (not upgradeable, works with 4.23 runtime)
- Firmware version warning is cosmetic - inference works correctly
- **ZeroMQ bridge** enables Hailo in Docker (host has Python 3.13, container has 3.10)

**Installed Packages (Host):**
| Package | Version | Purpose |
|---------|---------|---------|
| `hailort` | 4.23.0 | Runtime library and CLI |
| `hailort-pcie-driver` | 4.23.0 | Kernel driver (DKMS) |
| `python3-hailort` | 4.23.0 | Python API bindings |
| `pyzmq` | 27.x | ZeroMQ for host-container communication |

**Performance:**
| Backend | Model | FPS | Latency |
|---------|-------|-----|---------|
| CPU (Pi5) | YOLOv11n | 2-5 | 200-500ms |
| Hailo-8 | YOLOv11n | 25-40 | 25-40ms |

**File Structure:**
```
hailo8-int/
├── host_server/
│   └── hailo_inference_server.py  # ZeroMQ server (runs on host)
├── models/
│   └── yolov11n.hef               # Pre-compiled HEF model
└── docs/
    ├── INSTALLATION.md            # Driver setup
    ├── MODEL_CONVERSION.md        # ONNX to HEF
    └── TROUBLESHOOTING.md         # Common issues

ros2_nodes/yolo_hailo_bridge/      # ROS2 node (runs in Docker)
└── yolo_hailo_bridge/
    └── bridge_node.py             # ZeroMQ client, publishes to ROS2

systemd/hailo-server.service       # Systemd service for inference server
```

**Pre-compiled Models (Recommended):**
Download from Hailo Model Zoo - no conversion needed:
```bash
cd hailo8-int/conversion

# YOLOv11n (nano) - recommended for real-time on Pi
curl -L -o yolov11n.hef \
  "https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.17.0/hailo8/yolov11n.hef"

# Copy to models and deploy
mkdir -p ../models && cp yolov11n.hef ../models/
uv run python deploy_hailo8.py deploy
```

Available models: yolov11n (8MB), yolov11s (18MB), yolov11m (34MB), yolov11l (50MB), yolov11x (100MB)

Full model list: https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_object_detection.rst

**Manual Conversion (Advanced):**
For custom models, convert to HEF format on x86_64 (Docker):
```bash
cd hailo8-int/conversion
docker build -t hailo-converter .
docker run -v $(pwd):/workspace hailo-converter python convert_yolo.py \
    --input yolo11n.onnx --output yolo11n_hailo.hef

# On ARM machines (Apple Silicon) with Podman:
podman build --platform linux/amd64 -t hailo-converter .
podman run --platform linux/amd64 -v $(pwd):/workspace hailo-converter python convert_yolo.py \
    --input yolo11n.onnx --output yolo11n_hailo.hef
```

See `hailo8-int/docs/` for detailed documentation:
- `INSTALLATION.md` - Driver setup guide (HailoRT 4.23)
- `MODEL_CONVERSION.md` - ONNX to HEF conversion
- `TROUBLESHOOTING.md` - Common issues and fixes

**`ros2_nodes/arm_controller/`** - ROS2 arm controller node
- Subscribes to `/arm/cmd` (JSON commands)
- Publishes to `/arm/state` (JSON state)
- Supports: set_position, home, stop, gripper actions

**`ros2_nodes/lidar_driver/`** - ROS2 lidar driver node
- Publishes to `/scan` (sensor_msgs/LaserScan)
- Reads LD19/MS200 serial protocol at 230400 baud
- Accumulates 360° scan data before publishing

**Deployment Steps** (in order):
1. `system_update` - apt update/upgrade, install base tools
2. `hardware_config` - Enable I2C/SPI in `/boot/firmware/config.txt`
3. `peripheral_access` - Configure user groups (dialout, audio, video, i2c, gpio) and vcgencmd
4. `motion_controller` - Upload ros_robot_controller_sdk.py to ~/ros_robot_controller/
5. `camera_setup` - Install USB libs, udev rules, upload and extract Deptrum driver
6. `docker_ros2` - Install Docker, pull `ros:humble-ros-base`, build custom image with camera deps
7. `camera_driver_build` - Build Deptrum ROS2 driver in Docker container
8. `remote_access` - Enable SSH, optionally install VNC

**Peripheral Access** (Step 3 details):
- Adds user to groups: `dialout` (serial), `audio`, `video`, `i2c`, `gpio`
- Creates `/dev/vcio` device for `vcgencmd` temperature monitoring
- Installs udev rule `/etc/udev/rules.d/99-vcio.rules` for persistence

**Motion Controller** (Step 4 details):
- Uploads `ros_robot_controller_sdk.py` to `~/ros_robot_controller/`
- SDK enables direct serial communication with STM32 motor controller
- Used by `validation/test_chassis_direct.py` for motion control without ROS2

**Camera Setup** (Step 5 details):
- Installs libusb-1.0-0-dev and libudev-dev for USB camera access
- Creates udev rule `/etc/udev/rules.d/99-deptrum-libusb.rules` for Deptrum cameras (VID 3251)
- Uploads Deptrum ROS2 driver from `drivers/depth-camera/` to `~/deptrum_ws/`
- Extracts driver and fixes package.xml name mismatch

**Camera Driver Build** (Step 7 details):
- Runs `colcon build` inside Docker container with `-DSTREAM_SDK_TYPE=AURORA930`
- Fixes ownership of build files (Docker creates as root)
- Verifies build succeeded by checking for install directory

**Docker Image** (`landerpi-ros2`):
- Based on `ros:humble-ros-base`
- Includes camera dependencies: cv-bridge, image-transport, angles, tf2-ros, tf2-geometry-msgs, diagnostic-updater
- Includes navigation: navigation2, nav2-bringup, slam-toolbox

**Documentation Files:**
- `DepthCameraHowTo.md` - Aurora 930 depth camera setup guide with Deptrum SDK
- `CameraCheck.md` - Camera status report and troubleshooting
- `LanderPi_Ubuntu_Setup_Guide.md` - Complete manual setup guide with Docker-based ROS2
- `LanderPi_Linux_Command_Reference.md` - Linux command reference for robot setup

**Drivers:**
- `drivers/depth-camera/deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz` - Aurora 930 ROS2 driver (ARM64)

**Voice Control System:**

**`robot_voicecontroller.py`** - Voice controller (runs ON the robot)
- Uses WonderEcho Pro for audio I/O (card 1, stereo 48kHz)
- Local ASR via faster-whisper (base.en model)
- AWS Bedrock Claude Haiku 4.5 for command generation
- ElevenLabs Flash v2.5 for TTS response
- ROS2-based motor control via `docker exec landerpi-ros2` (requires deployed ROS2 stack)
- Wake word detection: "Hey TARS" activation with beep confirmation
- Startup announcement via TTS when service starts
- Commands: `listen` (single), `loop` (continuous), `loop --wake-word`, `check`, `test-tts`, `test-llm`

**Voice Commands Supported:**
- Movement: `forward`, `backward`, `turn_left`, `turn_right`, `strafe_left`, `strafe_right`, `stop`
- Compound: `look_around` (360° scan), `patrol` (square pattern), `come_here` (approach user)
- Modes: `follow_me` (continuous tracking - toggle on/off)
- Parameters: speed (0.1-0.5 m/s), duration (0.5-10.0 seconds)

**`deploy_voicecontroller.py`** - Deployment tool (runs locally)
- Uploads script and installs deps on robot via SSH
- Configures AWS credentials and ElevenLabs API key
- Installs systemd service (`tars-voice.service`) for auto-start on boot
- Generates activation beep sound using sox
- Commands: `deploy`, `upload`, `install`, `configure`, `install-service`, `test`

**`systemd/tars-voice.service`** - Systemd service for boot persistence
- Starts voice controller in wake word mode after boot
- Depends on network and Docker service
- Auto-restarts on failure

**Voice Control Architecture (all on robot):**
```
WonderEcho Pro (arecord) → faster-whisper (ASR) → Bedrock Haiku (LLM)
                                                         ↓
                                                 JSON command
                                                         ↓
                         docker exec landerpi-ros2 ros2 topic pub /controller/cmd_vel
                                                         ↓
                         ElevenLabs TTS → WonderEcho Pro (aplay)
```

**AWS IAM Policy** (`aws/landerpi-bedrock-policy.json`):
- BedrockInvokeOnly: Minimal permissions for robot
- Only allows `bedrock:InvokeModel` on Claude models

## Key Patterns

- Remote commands use `conn.run()` or `conn.sudo()` with `hide=True`
- `append_to_remote_file()` is idempotent - checks for existing content before appending
- Step completion markers allow safe re-runs without duplicating work

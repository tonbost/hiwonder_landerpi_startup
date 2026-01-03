---
name: landerpi-yolo
description: YOLOv11 object detection for HiWonder LanderPi robot. Provides semantic hazard detection during autonomous exploration, detecting people, pets, and obstacles that lidar alone cannot classify. Requires landerpi-core skill for connection setup and landerpi-camera skill for RGB feed.
---

# LanderPi YOLO Object Detection

## Overview

YOLO-based semantic hazard detection for LanderPi robot. Uses YOLOv11n to detect objects from the Aurora 930 depth camera's RGB feed, then fuses detections with lidar data to provide distance-aware hazard warnings.

**Prerequisites:**
- Load `landerpi-core` skill for connection and Docker setup
- Load `landerpi-camera` skill for depth camera feed
- ROS2 stack deployed (`deploy_ros2_stack.py deploy`)

## YOLO Pipeline

```
/aurora/rgb/image_raw → yolo_hailo_bridge → /yolo/detections → obstacle_fusion → /hazards
        (camera)              (YOLO)             ↓                  (fusion)        (JSON)
                                          /yolo/annotated_image
                                                 ↓
                                              RViz
```

| Node | Subscribes | Publishes | Purpose |
|------|------------|-----------|---------|
| `yolo_hailo_bridge` | `/aurora/rgb/image_raw` | `/yolo/detections`, `/yolo/annotated_image`, `/hazards` | Run YOLOv11 inference via Hailo |
| `obstacle_fusion` | `/yolo/detections`, `/scan` | `/hazards` | Correlate detections with lidar distance |

## Usage

### Enable YOLO During Exploration

**From local machine:**
```bash
# Deploy explorer (first time)
uv run python deploy_explorer.py deploy

# Start exploration with YOLO (CPU, 2-5 FPS)
uv run python deploy_explorer.py start --yolo --duration 10

# Start exploration with YOLO + Hailo-8 acceleration (25-40 FPS)
uv run python deploy_explorer.py start --yolo-hailo --duration 10

# With debug logging (saves annotated images to ~/yolo_logs)
uv run python deploy_explorer.py start --yolo --yolo-logging --duration 10
uv run python deploy_explorer.py start --yolo-hailo --yolo-logging --duration 10

# Combined with ROS2 bag recording
uv run python deploy_explorer.py start --yolo-hailo --rosbag --duration 15
```

**Directly on robot:**
```bash
python3 ~/landerpi/robot_explorer.py explore --duration 5 --yolo
```

### Verification Commands

**1. Check camera is publishing:**
```bash
ssh user@robot "docker exec landerpi-ros2 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic hz /aurora/rgb/image_raw'"
```

**2. Check YOLO detections:**
```bash
ssh user@robot "docker exec landerpi-ros2 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic echo /yolo/detections --once'"
```

**3. Check hazards output:**
```bash
ssh user@robot "docker exec landerpi-ros2 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic echo /hazards --once'"
```

**4. Check YOLO logs:**
```bash
uv run python deploy_ros2_stack.py logs | grep -i yolo
```

**5. Quick test (put yourself or object in front of camera):**
```bash
ssh user@robot "docker exec landerpi-ros2 bash -c 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /hazards'"
```

## ROS2 Topics

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/aurora/rgb/image_raw` | `sensor_msgs/Image` | ~30 Hz | RGB camera feed (input) |
| `/yolo/detections` | `vision_msgs/Detection2DArray` | ~14 Hz | Raw YOLO detections |
| `/yolo/annotated_image` | `sensor_msgs/Image` | ~14 Hz | Live camera feed with bounding boxes |
| `/hazards` | `std_msgs/String` | 10 Hz | JSON hazards with distance |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | Lidar data (for distance) |

### Live Annotated Video Stream

The `/yolo/annotated_image` topic publishes camera frames with YOLO bounding boxes drawn in real-time (~14 Hz when using Hailo).

**Annotation style:**
- Red boxes: Hazard classes (person, dog, cat, etc.)
- Green boxes: Other detected objects
- Labels: Class name + confidence score

**View with RViz:**
1. Connect RViz to robot's ROS2 network
2. Add Image display
3. Set topic to `/yolo/annotated_image`

**Verify topic is publishing:**
```bash
ssh user@robot "docker exec landerpi-ros2 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic hz /yolo/annotated_image'"
```

## Hazard Classes

The obstacle_fusion node filters detections to hazard classes defined in a JSON config file.

**Default hazard classes:**
| Category | Classes |
|----------|---------|
| People | `person` |
| Animals | `dog`, `cat` |
| Objects | `cup`, `bottle`, `stop sign` |

**Note:** The full YOLO COCO model detects 80 classes - any can be added to the config.

### Configuring Hazard Classes (JSON Config)

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
| Parameter | Value | Purpose |
|-----------|-------|---------|
| `hazard_distance` | 2.5m | Max distance to report hazard (fusion node) |
| `semantic_stop_distance` | 0.8m | Distance to trigger stop (explorer) |

**Important:** `hazard_distance` > `semantic_stop_distance` ensures hazards are reported before stop is triggered.

## Hazard Message Format

The `/hazards` topic publishes JSON strings:

```json
{
  "hazards": [
    {
      "type": "person",
      "distance": 0.85,
      "angle": 5.2,
      "score": 0.92,
      "source": "depth"
    }
  ]
}
```

| Field | Type | Description |
|-------|------|-------------|
| `type` | string | Object class name |
| `distance` | float | Distance in meters |
| `angle` | float | Angle from center (degrees, + = left) |
| `score` | float | Detection confidence (0-1) |
| `source` | string | Distance source: `depth` (camera) or `lidar` (fallback) |

## How Fusion Works

1. **Detection**: YOLO detects object bounding box in image (cx, cy, width, height)
2. **Distance Lookup** (depth camera - preferred, multi-region sampling):
   - **Region 1 - Center**: Sample inner 50% of bbox (original approach)
   - **Region 2 - Lower**: Sample bottom 30% of bbox (catches floor-level objects)
   - **Region 3 - Vertical strip**: Sample 20% width strip through full bbox height (tall thin objects)
   - Filter valid depths (100mm - 10000mm) from all regions
   - Return **minimum depth** across all regions (closest point wins)
   - Convert mm to meters using `depth_scale`
3. **Distance Lookup** (lidar - fallback):
   - Map bbox center X to angle: `angle = ((width/2 - cx) / (width/2)) * (fov/2)`
   - Query lidar at calculated angle (±10° cone)
   - Return minimum distance in cone
4. **Hazard Check**: If distance < 2.5m and class in hazard_classes, publish hazard
5. **Stop Decision**: Exploration controller stops if hazard distance < 0.8m (semantic_stop_distance)

### Multi-Region Depth Sampling (Why It Matters)

The camera is mounted higher than floor-level objects. For a bottle on the floor:

```
Camera View (looking down):
┌─────────────────────────────┐
│      Background (far)       │ ← Old: sampled HERE (bbox center)
│   ┌─────────┐               │     = background distance (1.5m+)
│   │ Bottle  │               │
│   │  top    │ ← Region 1: Center (50%)
│   │         │ ← Region 3: Vertical strip
│   │  body   │
│   └─────────┘ ← Region 2: Lower (30%) = ACTUAL bottle
│      Floor                  │
└─────────────────────────────┘
```

**Without multi-region**: Bbox center points to background → distance > 1.5m → no hazard
**With multi-region**: Lower region catches bottle surface → correct distance → hazard published

## Exploration Integration

When `--yolo` flag is used, the exploration controller:

1. Reads `/hazards` topic via `hardware.read_hazards()`
2. Updates `SensorFusion` with hazard list
3. If hazard within `stop_distance * 1.5` (22.5cm), triggers immediate stop
4. Hazard distance overrides lidar distance if closer

**sensor_fusion.py logic:**
```python
def update_hazards(self, hazards: List[dict]) -> None:
    for h in hazards:
        if h["distance"] < self.config.stop_distance * 1.5:
            self.state.semantic_hazard = True
            self.state.hazard_type = h["type"]
            if h["distance"] < self.state.closest_distance:
                self.state.closest_distance = h["distance"]
```

### YOLO + Lidar Sensor Fusion

**YOLO is additive to lidar, not a replacement.** Both systems work together:

| Sensor | Role | What it detects |
|--------|------|-----------------|
| Lidar | Primary distance | Walls, furniture, any physical obstacle |
| Depth camera | Secondary distance | Low obstacles lidar might miss |
| YOLO | Semantic classification | People, pets, specific objects |

**Fusion logic in `sensor_fusion.py`:**

```python
# 1. Lidar is PRIMARY distance source
self.state.closest_distance = self.state.lidar_distance

# 2. Depth camera can LOWER distance (safety override for low obstacles)
if depth_distance < lidar_distance * 0.7:
    self.state.closest_distance = depth_distance

# 3. YOLO semantic hazard triggers IMMEDIATE STOP
if self.state.semantic_hazard:
    self.state.should_stop = True  # Regardless of distance
```

**Key behaviors:**
- ✅ Lidar obstacle detection **always runs** (even with `--yolo`)
- ✅ YOLO hazards are **additive** - extra safety layer
- ✅ YOLO can detect threats lidar misses (people at different heights, pets)
- ✅ Semantic hazards trigger stop even if lidar shows clear path
- ✅ YOLO distance can override lidar if closer (e.g., person detected at 0.5m)

## YOLO Node Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | `yolo11n.pt` | YOLO model file |
| `confidence_threshold` | 0.4 | Minimum detection confidence |
| `device` | `cpu` | Inference device (RPi5 = cpu) |
| `enable_logging` | `false` | Enable debug image logging |
| `log_dir` | `~/yolo_logs` | Directory for logged images |
| `log_all_frames` | `false` | Log all frames (vs only detections) |
| `max_log_images` | 500 | Auto-cleanup threshold |

**Note:** First run downloads the model (~6MB for yolo11n).

## Debug Logging

Enable YOLO logging to save annotated images and detection history for debugging.

### Enable Logging

**Option 1: Via deploy_explorer flag (recommended)**
```bash
# Start exploration with YOLO logging enabled
uv run python deploy_explorer.py start --yolo --yolo-logging --duration 10
```
This automatically restarts the ROS2 stack with `enable_yolo_logging:=true`.

**Option 2: Manual via SSH**
```bash
# Restart stack with logging enabled
ssh user@robot "cd ~/landerpi/docker && docker compose down"
ssh user@robot "cd ~/landerpi/docker && docker compose up -d"
# Then manually start with logging:
ssh user@robot "docker exec landerpi-ros2 bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run yolo_detector yolo_node --ros-args -p enable_logging:=true'"
```

**Option 3: Edit launch file parameter** (persistent)
Set `enable_yolo_logging:=true` in the launch command.

### Log Output Structure

```
~/yolo_logs/
└── session_20250101_120000/
    ├── metadata.json          # Session config
    ├── detections.jsonl       # Detection records
    └── images/
        ├── frame_000001_120001_123_det.jpg  # Annotated (has detections)
        ├── frame_000002_120001_456.jpg       # Raw (no detections, if log_all_frames=true)
        └── ...
```

### View Logs

```bash
# List sessions
ssh user@robot "ls -la ~/yolo_logs/"

# View latest detections
ssh user@robot "tail -20 ~/yolo_logs/session_*/detections.jsonl"

# Copy images to local machine
scp -r user@robot:~/yolo_logs/session_* ./yolo_debug/

# View session summary
ssh user@robot "cat ~/yolo_logs/session_*/metadata.json"
```

### Detection Record Format

Each line in `detections.jsonl`:
```json
{
  "timestamp": 1704110400.123,
  "frame": 42,
  "image": "frame_000042_120530_123_det.jpg",
  "detections": [
    {"class": "person", "score": 0.92, "cx": 320, "cy": 240, "width": 100, "height": 200}
  ]
}
```

### Auto-Cleanup

Images are automatically cleaned up when exceeding `max_log_images` (default 500). Oldest images are deleted first. The `detections.jsonl` file is not cleaned up.

### Correlating Explorer Output with YOLO Logs

Explorer output now includes timestamps for cross-referencing with YOLO detection logs:

**Explorer output format:**
```
[15:47:52.123] Obstacle at 0.08m - stopped (blocked: 1)
[15:47:52.456] [TARS] Starting exploration
[15:48:01.789] HAZARD: bottle at 0.85m - STOPPING
[15:48:02.012] Status: 4.4 min remaining | action: stopped | escape: NONE
```

**YOLO detections.jsonl format:**
```json
{"timestamp": 1735760881.789, "frame": 42, "detections": [{"class": "bottle", ...}]}
```

**To correlate:**
1. Convert YOLO Unix timestamp to local time: `date -r 1735760881` → `15:48:01`
2. Match with explorer output timestamp `[15:48:01.789]`
3. Find corresponding annotated image in `~/yolo_logs/session_*/images/`

**Quick correlation script:**
```bash
# On robot: convert detection timestamps to readable format
cat ~/yolo_logs/session_*/detections.jsonl | python3 -c "
import sys, json
from datetime import datetime
for line in sys.stdin:
    d = json.loads(line)
    ts = datetime.fromtimestamp(d['timestamp']).strftime('%H:%M:%S.%f')[:-3]
    det = d.get('detections', [])
    if det:
        print(f'[{ts}] {det[0][\"class\"]} score={det[0][\"score\"]:.2f}')
"
```

## Fusion Node Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `image_width` | 640 | Camera image width |
| `image_height` | 480 | Camera image height |
| `fov_horizontal` | 60.0 | Camera horizontal FOV (degrees) |
| `hazard_classes` | (from JSON config) | Classes that trigger hazards |
| `hazard_distance` | (from JSON config) | Max distance to report hazard (meters) |
| `use_depth_camera` | true | Use depth camera for distance (more accurate than lidar) |
| `depth_scale` | 0.001 | Depth value scale (mm to meters) |

**Note:** `hazard_classes` and `hazard_distance` are now loaded from `config/yolo_hazards.json`. See [Configuring Hazard Classes](#configuring-hazard-classes-json-config).

## Semantic Stop Distance

The exploration controller uses a separate stop distance for semantic hazards:

| Threshold | Value | Purpose |
|-----------|-------|---------|
| `stop_distance` | 0.15m | Lidar obstacles (walls, furniture) |
| `semantic_stop_distance` | 0.8m | YOLO hazards (people, pets, bottles) |

This means the robot stops **0.8m away from people/bottles** but can get **0.15m from walls**.

**Important:** The fusion node's `hazard_distance` (2.5m) is intentionally larger than `semantic_stop_distance` (0.8m). This allows the fusion node to report all nearby hazards, while the explorer decides when to actually stop.

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                    Docker: landerpi-ros2                          │
│                                                                   │
│  ┌─────────────┐    /aurora/rgb     ┌──────────────┐             │
│  │   Aurora    │ ──────────────────→│ yolo_detector│             │
│  │   Camera    │    image_raw       │  (YOLOv11n)  │             │
│  │   Driver    │                    └──────┬───────┘             │
│  │             │    /aurora/depth          │                      │
│  │             │ ────────────┐     /yolo/detections               │
│  └─────────────┘             │             │                      │
│                              │     ┌───────▼───────┐              │
│  ┌─────────────┐             └────→│ obstacle_     │              │
│  │   Lidar     │    /scan          │ fusion        │  /hazards    │
│  │   Driver    │ ─────────────────→│ (depth+YOLO)  │─────────────→│
│  └─────────────┘   (fallback)      └───────────────┘              │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
                                            │
                                    ┌───────▼───────┐
                                    │ Exploration   │
                                    │ Controller    │
                                    │ (reads /hazards)│
                                    └───────────────┘
```

## Performance

| Metric | Value | Notes |
|--------|-------|-------|
| Inference Time | ~200-300ms | YOLOv11n on RPi5 CPU |
| Detection Rate | ~3-5 Hz | Limited by CPU inference |
| Fusion Rate | 10 Hz | Timer-based publishing |
| Latency | ~300-500ms | Detection to hazard output |

**Note:** RPi5 has no GPU/CUDA, so inference runs on CPU. Consider ONNX export or smaller models (yolo11n is already smallest) for better performance.

## Hailo Acceleration

For 5-10x faster inference, use `--yolo-hailo` flag for Hailo-8 hardware acceleration:

```bash
# One-time setup
uv run python deploy_hailo8.py check    # Check hardware
uv run python deploy_hailo8.py install  # Install driver

# Download pre-compiled model (no conversion needed!)
cd hailo8-int/conversion
curl -L -o yolov11n.hef \
  "https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.17.0/hailo8/yolov11n.hef"
mkdir -p ../models && cp yolov11n.hef ../models/
cd ../..

# Deploy models to robot
uv run python deploy_hailo8.py deploy

# Use Hailo during exploration
uv run python deploy_explorer.py start --yolo-hailo --duration 10
```

| Backend | FPS | Latency | Flag |
|---------|-----|---------|------|
| CPU | 2-5 | 200-500ms | `--yolo` |
| Hailo-8 | 25-40 | 25-40ms | `--yolo-hailo` |

**Pre-compiled models available:**
| Model | mAP | Size |
|-------|-----|------|
| yolov11n | 39.0 | ~8 MB |
| yolov11s | 46.3 | ~18 MB |
| yolov11m | 51.1 | ~34 MB |

Full model list: https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_object_detection.rst

See `landerpi-hailo` skill for:
- Driver installation (HailoRT 4.23, Python 3.13 support)
- Manual model conversion (for custom-trained models)
- Troubleshooting

## Troubleshooting

### Problem: No detections on /yolo/detections

**Symptoms:**
- Topic exists but no messages
- Camera topic has data

**Diagnosis:**
```bash
# Check YOLO node is running
uv run python deploy_ros2_stack.py logs | grep -i "yolo\|model"
```

**Possible causes:**
1. Model failed to download - check internet on robot
2. Model load error - check logs for "Failed to load model"
3. No objects in frame - point camera at person/cup

**Solution:**
```bash
# SSH to robot and manually test YOLO
docker exec -it landerpi-ros2 bash
source /opt/ros/humble/setup.bash
python3 -c "from ultralytics import YOLO; m = YOLO('yolo11n.pt'); print('OK')"
```

### Problem: No hazards even with detections

**Symptoms:**
- `/yolo/detections` has data
- `/hazards` is empty

**Diagnosis:**
```bash
# Check fusion node logs
uv run python deploy_ros2_stack.py logs | grep -i fusion
```

**Possible causes:**
1. Object not in hazard_classes - check launch file for current classes
2. Object too far - must be < 2.5m for hazard to be published
3. Depth sampling missing the object (see below)
4. `hazard_distance` configured too low (see next section)

**Solution:**
1. Move closer to camera (< 2.5m)
2. Ensure object is a hazard class
3. Check depth camera is running: `ros2 topic echo /aurora/depth/image_raw --once`

### Problem: Robot doesn't stop for detected objects (hazard_distance issue)

**Symptoms:**
- YOLO detects object with high confidence (check `~/yolo_logs/`)
- Robot keeps moving and collides with object
- No "HAZARD:" messages in explorer output

**Root Cause:**
The fusion node's `hazard_distance` threshold was too low, filtering out objects before they reached the explorer. For example, if `hazard_distance=1.0m` and the bottle is estimated at 1.5m, the hazard is silently discarded.

**Diagnosis:**
```bash
# Check current hazard_distance setting
grep hazard_distance ros2_nodes/cmd_vel_bridge/launch/landerpi.launch.py
```

**Solution:**
Ensure `hazard_distance` (fusion) > `semantic_stop_distance` (explorer):

| Parameter | Location | Correct Value |
|-----------|----------|---------------|
| `hazard_distance` | launch file | 2.5m (report hazards) |
| `semantic_stop_distance` | sensor_fusion.py | 0.8m (stop distance) |

**Fix applied (2026-01-01):** Changed `hazard_distance` from 1.0m to 2.5m.

```bash
# Redeploy to apply fix
uv run python deploy_ros2_stack.py deploy
```

### Problem: Floor-level objects not triggering stop (FIXED)

**Symptoms:**
- YOLO detects bottle/cup on floor with high confidence
- Robot keeps moving, doesn't stop
- Debug logs show detection but no hazard published

**Root Cause (historical):**
Camera is mounted higher than floor objects. Old depth sampling only checked bbox center, which pointed to background (tablecloth, wall) instead of the actual object.

```
Bottle on floor, camera looking down:
- Bbox center (old) → background at 1.5m+ → no hazard
- Bbox lower region (new) → bottle at 0.3m → hazard published
```

**Solution:**
This was fixed with multi-region depth sampling (2026-01-01). The fusion node now samples:
1. Center region (50%) - original behavior
2. Lower region (30%) - catches floor-level objects
3. Vertical strip (20%) - catches tall thin objects

**To apply fix:**
```bash
uv run python deploy_ros2_stack.py deploy
```

**Verification:**
```bash
# Check hazards include "source" field showing depth worked
ssh user@robot "docker exec landerpi-ros2 bash -c 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /hazards'"
```

### Problem: YOLO is slow / laggy

**Symptoms:**
- Detection rate < 1 Hz
- High CPU usage

**Cause:** YOLOv11n on RPi5 CPU is inherently slow (~200-300ms per frame)

**Solutions:**
1. This is expected on RPi5 - no GPU available
2. Consider using yolo11n (smallest model)
3. Reduce input resolution if needed
4. Accept ~3-5 Hz detection rate

### Problem: Camera topic not publishing

**Symptoms:**
- `/aurora/rgb/image_raw` shows 0 Hz
- YOLO has nothing to process

**Solution:**
See `landerpi-camera` skill for camera troubleshooting.

```bash
# Quick check
uv run python validation/test_cameradepth_ros2.py check
```

### Problem: Robot doesn't stop for hazards (read_hazards blocking)

**Symptoms:**
- `/hazards` topic is being published correctly
- YOLO detections working at good FPS
- Robot doesn't stop, continues moving into hazards
- Control loop runs slower than expected (~0.5Hz instead of 10Hz)

**Root Cause:**
The `read_hazards()` function in `ros2_hardware.py` uses `ros2 topic echo` which has significant initialization overhead (~300-500ms). This blocks the control loop, causing:
1. Lidar readings to be processed too slowly
2. Robot can't react to obstacles in time
3. Even lidar-detected obstacles are missed

**Diagnosis:**
```bash
# Check control loop rate during exploration
# Look for "Status:" lines - they should appear every 5 seconds
# If control loop is slow, status updates will be delayed

# Time the hazard reading function
ssh user@robot "time docker exec landerpi-ros2 bash -c 'source /opt/ros/humble/setup.bash && timeout 0.5 ros2 topic echo /hazards --once 2>/dev/null'"
```

**Key insight:** `ros2 topic echo` takes 300-500ms just to initialize the ROS2 node before it can receive any messages. This is unavoidable overhead.

**Solution (implemented):**
1. **Caching**: Only read hazards every 0.5 seconds, cache results
2. **Short timeout**: Use `timeout 0.3` inside docker, 1.0s subprocess timeout
3. **Keep cached value**: If read fails, keep previous cached hazards

```python
# ros2_hardware.py caching pattern
def read_hazards(self) -> List[dict]:
    now = time.time()
    if now - self._last_hazard_time < self._hazard_read_interval:
        return self._cached_hazards  # Return cached value

    self._last_hazard_time = now
    # ... read with short timeout ...
    return self._cached_hazards
```

**Limitation:** Even with caching, the first read after cache expiry may return empty if `ros2 topic echo` doesn't receive a message within the timeout. This is acceptable because:
1. Lidar provides primary obstacle detection (always working)
2. YOLO hazards are additive safety layer
3. Next cache read will likely succeed

**Future improvement:** A persistent ROS2 subscriber process could eliminate initialization overhead entirely.

### Problem: JSON truncation in hazard messages

**Symptoms:**
- `/hazards` messages are being read but parsing fails
- JSON ends with `...` instead of `}`
- Partial hazard data extracted

**Root Cause:**
`ros2 topic echo` truncates long string fields. The `/hazards` topic publishes JSON inside a `std_msgs/String` data field, which gets truncated if too long.

**Workaround (implemented):**
Use regex to extract individual hazard objects from truncated JSON:

```python
# Handle truncated JSON like: {"hazards": [{"type": "person", "distance": 0.5, ...
for match in re.finditer(
    r'\{"type":\s*"([^"]+)",\s*"distance":\s*([\d.]+),\s*"angle":\s*([-\d.]+),\s*"score":\s*([\d.]+),\s*"source":\s*"([^"]+)"\}',
    truncated_output
):
    hazards.append({
        "type": match.group(1),
        "distance": float(match.group(2)),
        # ... etc
    })
```

**Better solution (not yet implemented):**
Publish hazards as separate ROS2 message type instead of JSON string, or use shorter field names to fit within ros2 topic echo's display limits.

### Problem: Camera stops mid-exploration (stability issue)

**Symptoms:**
- Exploration starts normally with YOLO detections
- After several minutes, camera stops publishing
- `Depth samples: 0` in exploration summary
- YOLO logs show detections stopped partway through

**Diagnosis:**
```bash
# Check if camera is currently publishing
docker exec landerpi-ros2 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic hz /aurora/rgb/image_raw'

# Check camera driver logs for errors
docker logs landerpi-ros2 2>&1 | grep -i "aurora\|camera\|ERR"
```

**Possible causes:**
1. USB power instability on long runs
2. Camera driver timeout/crash
3. Memory exhaustion from YOLO inference

**Workarounds:**
1. Restart ROS2 stack before each exploration: `uv run python deploy_ros2_stack.py deploy`
2. Use shorter exploration durations (2-3 min instead of 5+)
3. Check USB connections are secure

**Note:** This is a known hardware/driver stability issue with Aurora 930 on RPi5. The camera works initially but may crash during extended operation.

## Camera Health Monitoring

When running with `--yolo`, the explorer monitors camera health every 30 seconds:

- Detects when camera stops publishing (Aurora 930 can crash during extended operation)
- Reports camera recovery with publish rate
- TTS announcements: "Warning: Camera offline" / "Camera recovered"

**Example output:**
```
[12:34:56.789] [WARNING] Camera stopped publishing! YOLO detection disabled.
[12:35:26.123] [INFO] Camera recovered (9.8 Hz)
```

## Files

| File | Location | Purpose |
|------|----------|---------|
| `yolo_hazards.json` | `config/` | Hazard classes and thresholds (JSON config) |
| `yolo_node.py` | `ros2_nodes/yolo_detector/yolo_detector/` | YOLO detector node |
| `fusion_node.py` | `ros2_nodes/obstacle_fusion/obstacle_fusion/` | Lidar+YOLO fusion |
| `sensor_fusion.py` | `validation/exploration/` | Hazard integration in explorer |
| `ros2_hardware.py` | `validation/exploration/` | `read_hazards()` and `check_camera_health()` |

## Adding Custom Hazard Classes

To detect additional objects (e.g., bird, teddy bear):

1. Edit `config/yolo_hazards.json`:
```json
{
  "hazard_classes": ["person", "dog", "cat", "cup", "bottle", "stop sign", "bird", "teddy bear"],
  "hazard_distance": 2.5,
  "semantic_stop_distance": 0.8
}
```

2. Redeploy ROS2 stack:
```bash
uv run python deploy_ros2_stack.py deploy
```

3. YOLO detects 80 COCO classes - any can be added to hazard_classes.

## COCO Classes Available

Full list of detectable classes (YOLO COCO):
```
person, bicycle, car, motorcycle, airplane, bus, train, truck, boat,
traffic light, fire hydrant, stop sign, parking meter, bench, bird, cat,
dog, horse, sheep, cow, elephant, bear, zebra, giraffe, backpack, umbrella,
handbag, tie, suitcase, frisbee, skis, snowboard, sports ball, kite,
baseball bat, baseball glove, skateboard, surfboard, tennis racket, bottle,
wine glass, cup, fork, knife, spoon, bowl, banana, apple, sandwich, orange,
broccoli, carrot, hot dog, pizza, donut, cake, chair, couch, potted plant,
bed, dining table, toilet, tv, laptop, mouse, remote, keyboard, cell phone,
microwave, oven, toaster, sink, refrigerator, book, clock, vase, scissors,
teddy bear, hair drier, toothbrush
```

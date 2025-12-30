# Depth Camera Integration for Autonomous Exploration

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Integrate the Aurora 930 depth camera with lidar-based autonomous exploration for more accurate obstacle detection.

**Architecture:** The `SensorFusion` class already supports depth camera data - we need to read `/aurora/depth/image_raw` from ROS2 and pass it to the existing fusion logic. The fusion uses conservative min() of both sensors for obstacle detection.

**Tech Stack:** ROS2 Humble (Docker), sensor_msgs/Image (MONO16), Python, Fabric SSH

---

## Current State Analysis

- `sensor_fusion.py` has `update_depth()` method fully implemented (lines 55-93)
- `test_exploration.py` hardcodes `depth_data=None` (line 275)
- Lidar reading pattern exists in `read_lidar_scan()` (lines 169-225)
- Camera topics available: `/aurora/depth/image_raw` (MONO16, mm units)

## Implementation Tasks

### Task 1: Add Depth Image Reading Method

**Files:**
- Modify: `validation/test_exploration.py` (after line 225)

**Step 1: Add the read_depth_image method**

Add this method to the `ExplorationRunner` class after `read_lidar_scan()`:

```python
def read_depth_image(self) -> tuple:
    """Read depth image from camera. Returns (depth_data, width, height)."""
    # Script to read one depth image - outputs JSON with dimensions and data
    depth_script = """
import sys
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time

class DepthReader(Node):
    def __init__(self):
        super().__init__('depth_reader')
        self.data = None
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Image, '/aurora/depth/image_raw', self.cb, qos)

    def cb(self, msg):
        if self.data is None:
            # MONO16 = 2 bytes per pixel, little-endian
            width = msg.width
            height = msg.height
            # Extract depth values from raw data
            depths = []
            for i in range(0, len(msg.data), 2):
                if i + 1 < len(msg.data):
                    # Little-endian 16-bit
                    val = msg.data[i] | (msg.data[i + 1] << 8)
                    depths.append(val)
            self.data = {
                'width': width,
                'height': height,
                'depths': depths,
            }

rclpy.init()
node = DepthReader()
start = time.time()
while node.data is None and (time.time() - start) < 2.0:
    rclpy.spin_once(node, timeout_sec=0.1)
if node.data:
    # Only output dimensions and sample to reduce data size
    # For fusion, we only need center region anyway
    w, h = node.data['width'], node.data['height']
    depths = node.data['depths']
    # Extract center 200x160 region (larger than fusion needs)
    cx, cy = w // 2, h // 2
    hw, hh = 100, 80
    center_depths = []
    for y in range(max(0, cy - hh), min(h, cy + hh)):
        for x in range(max(0, cx - hw), min(w, cx + hw)):
            idx = y * w + x
            if idx < len(depths):
                center_depths.append(depths[idx])
    out = {
        'width': min(hw * 2, w),
        'height': min(hh * 2, h),
        'depths': center_depths,
    }
    print(json.dumps(out))
node.destroy_node()
rclpy.shutdown()
"""
    try:
        # Write script to temp file on robot and copy into container
        self.conn.run(f"cat > /tmp/read_depth.py << 'SCRIPT_EOF'\n{depth_script}\nSCRIPT_EOF", hide=True)
        self.conn.run("docker cp /tmp/read_depth.py landerpi-ros2:/tmp/read_depth.py", hide=True)

        # Run inside the already-running landerpi-ros2 container
        result = self.conn.run(
            "docker exec landerpi-ros2 bash -c '"
            "source /opt/ros/humble/setup.bash && "
            "source /ros2_ws/install/setup.bash 2>/dev/null; "
            "source /deptrum_ws/install/local_setup.bash 2>/dev/null; "
            "python3 /tmp/read_depth.py'",
            hide=True, warn=True, timeout=10
        )
        if result.ok and result.stdout.strip():
            data = json.loads(result.stdout.strip())
            return data['depths'], data['width'], data['height']
    except Exception:
        pass
    return [], 0, 0
```

**Step 2: Verify syntax by running a quick check**

Run: `python3 -m py_compile validation/test_exploration.py`
Expected: No output (success)

**Step 3: Commit**

```bash
git add validation/test_exploration.py
git commit -m "feat(exploration): add read_depth_image method for camera integration"
```

---

### Task 2: Integrate Depth Reading into Main Loop

**Files:**
- Modify: `validation/test_exploration.py` (lines 270-276)

**Step 1: Update the sensor reading section in run_exploration**

Find this code block (around line 270):

```python
# Read sensors
ranges, angle_min, angle_inc = self.read_lidar_scan()
if ranges:
    self.controller.update_sensors(
        ranges, angle_min, angle_inc,
        depth_data=None, depth_width=0, depth_height=0
    )
```

Replace with:

```python
# Read sensors
ranges, angle_min, angle_inc = self.read_lidar_scan()
depth_data, depth_width, depth_height = self.read_depth_image()

if ranges:
    self.controller.update_sensors(
        ranges, angle_min, angle_inc,
        depth_data=depth_data if depth_data else None,
        depth_width=depth_width,
        depth_height=depth_height
    )
```

**Step 2: Verify syntax**

Run: `python3 -m py_compile validation/test_exploration.py`
Expected: No output (success)

**Step 3: Commit**

```bash
git add validation/test_exploration.py
git commit -m "feat(exploration): integrate depth camera data into main loop"
```

---

### Task 3: Add Depth Logging to Remote Logger

**Files:**
- Modify: `validation/exploration/remote_data_logger.py` (check if log_depth exists)
- Modify: `validation/exploration/explorer.py` (add depth logging call)

**Step 1: Check remote_data_logger.py for log_depth method**

Read the file to verify `log_depth` method exists. If not, add it.

**Step 2: Add depth stats calculation and logging in explorer.py**

In `ExplorationController.update_sensors()`, after updating sensor fusion, add depth logging:

Find this section in `explorer.py` (around line 95-110):

```python
def update_sensors(
    self,
    lidar_ranges: List[float],
    angle_min: float,
    angle_increment: float,
    depth_data: Optional[List[int]] = None,
    depth_width: int = 0,
    depth_height: int = 0,
) -> None:
```

After the sensor fusion updates, add depth logging:

```python
# Log depth stats if available
if depth_data and self.logger:
    valid_depths = [d for d in depth_data if 150 <= d <= 3000]
    if valid_depths:
        min_depth = min(valid_depths) / 1000.0  # mm to m
        avg_depth = sum(valid_depths) / len(valid_depths) / 1000.0
        valid_pct = len(valid_depths) / len(depth_data) * 100.0
        self.logger.log_depth(min_depth, avg_depth, valid_pct)
```

**Step 3: Verify syntax**

Run: `python3 -m py_compile validation/exploration/explorer.py`
Expected: No output (success)

**Step 4: Commit**

```bash
git add validation/exploration/explorer.py validation/exploration/remote_data_logger.py
git commit -m "feat(exploration): add depth camera logging"
```

---

### Task 4: Add Status Display for Depth Sensor

**Files:**
- Modify: `validation/test_exploration.py` (status display section)

**Step 1: Update periodic status display to show depth info**

Find the periodic status section (around line 283-287):

```python
# Periodic status
if time.time() - last_status_time >= status_period:
    last_status_time = time.time()
    status = self.controller.get_status()
    remaining = status['safety']['remaining_minutes']
    console.print(f"[dim]Status: {remaining:.1f} min remaining, action: {status['action']}[/dim]")
```

Replace with:

```python
# Periodic status
if time.time() - last_status_time >= status_period:
    last_status_time = time.time()
    status = self.controller.get_status()
    remaining = status['safety']['remaining_minutes']
    sensors = status.get('sensors', {})
    depth_m = sensors.get('depth_m', 'N/A')
    lidar_m = sensors.get('lidar_m', 'N/A')
    console.print(
        f"[dim]Status: {remaining:.1f} min remaining | "
        f"depth: {depth_m}m, lidar: {lidar_m}m | "
        f"action: {status['action']}[/dim]"
    )
```

**Step 2: Verify syntax**

Run: `python3 -m py_compile validation/test_exploration.py`
Expected: No output (success)

**Step 3: Commit**

```bash
git add validation/test_exploration.py
git commit -m "feat(exploration): show depth sensor status in periodic display"
```

---

### Task 5: Add Camera Check to Prerequisites

**Files:**
- Modify: `validation/test_exploration.py` (check_prerequisites method)

**Step 1: Add camera topic check to prerequisites**

Find `check_prerequisites()` method. After the depth camera USB check, add a camera topic check:

```python
# Check camera topic (if camera USB detected)
if checks[-1][1]:  # If camera USB found
    result = self.conn.run(
        "docker exec landerpi-ros2 bash -c '"
        "source /opt/ros/humble/setup.bash && "
        "source /deptrum_ws/install/local_setup.bash 2>/dev/null; "
        "ros2 topic list 2>/dev/null | grep /aurora/depth'",
        hide=True, warn=True
    )
    checks.append(("Depth topic", result.ok))
```

**Step 2: Verify syntax**

Run: `python3 -m py_compile validation/test_exploration.py`
Expected: No output (success)

**Step 3: Commit**

```bash
git add validation/test_exploration.py
git commit -m "feat(exploration): add depth camera topic to prerequisite checks"
```

---

### Task 6: Integration Test

**Step 1: Run prerequisite check**

Run: `uv run python validation/test_exploration.py status`

Expected: All prerequisites pass, including "Depth topic: OK"

**Step 2: Run short exploration test with camera**

Run: `uv run python validation/test_exploration.py start --duration 2 --yes`

Expected:
- Status updates show both depth and lidar distances
- Robot avoids obstacles using fused sensor data
- Exploration completes without errors

**Step 3: Verify logs contain depth data**

On robot, check: `cat ~/landerpi/logs/exploration_*/depth_summary.jsonl`

Expected: JSON lines with `min_depth`, `avg_depth`, `valid_percent` fields

**Step 4: Final commit**

```bash
git add -A
git commit -m "feat(exploration): complete depth camera integration

- Add read_depth_image() method for ROS2 depth topic reading
- Integrate depth data into main exploration loop
- Add depth logging and status display
- Add camera topic to prerequisite checks
- Sensor fusion now uses both lidar and depth camera"
```

---

## Verification Checklist

- [ ] `read_depth_image()` method added and working
- [ ] Main loop passes depth data to controller
- [ ] Status display shows depth distance
- [ ] Prerequisites check camera topic
- [ ] Depth data logged to robot
- [ ] Short exploration test runs with both sensors

## Future Enhancements (Not in This Plan)

1. **Weighted fusion** - Use sensor confidence instead of simple min()
2. **Depth-based frontier scoring** - Use depth for vertical clearance checks
3. **Obstacle classification** - Distinguish walls from cliffs using depth gradients
4. **Visualization** - Display fused sensor data for debugging

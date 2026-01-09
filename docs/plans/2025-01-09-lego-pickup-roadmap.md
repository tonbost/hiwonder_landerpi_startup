# Lego Pickup Feature Roadmap

**Date:** 2025-01-09
**Goal:** Enable the robot to find Lego bricks in a multi-room apartment, pick them up, and deposit them in a designated location.

## Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Approach | Foundation first | Build reusable SLAM + kinematics before pickup |
| Priority | SLAM navigation first | Enables location memory, drop zone navigation |
| Environment | Multi-room apartment | Requires loop closure, larger maps |
| SLAM sensors | Lidar-only (initially) | Simpler, proven, lighting-independent |
| SLAM package | rtabmap | Better loop closure, multi-session maps, RGB-D ready for future |

## Phase Overview

```
Phase 1 (SLAM) ─────────────────────────┐
                                        ├──► Phase 4 (Pick & Place)
Phase 2 (Arm Kinematics) ───────────────┤
                                        │
Phase 3 (Object Localization) ──────────┘
     └── requires Phase 1 (map frame)
```

Phases 2 and 3 can be developed in parallel after Phase 1 is complete.

---

## Phase 1: SLAM Navigation

**Goal:** Robot can build a map of the apartment, save it, and navigate to any point on the map.

### Components

| Component | Purpose | Integration |
|-----------|---------|-------------|
| **rtabmap** | SLAM - builds occupancy grid from lidar | Subscribes to `/scan`, publishes `/map` and `/tf` (map→odom) |
| **Nav2** | Path planning + obstacle avoidance | Uses `/map`, publishes to `/cmd_vel` |
| **Localization** | Find robot position in saved map | rtabmap's localization mode |

### Implementation Steps

1. **Create rtabmap launch file** - Configure for 2D lidar mode (no RGB-D initially)
2. **Add Nav2 bringup** - Path planner, controller, costmaps
3. **Extend Docker image** - Add rtabmap-ros package to `landerpi-ros2` image
4. **Create mapping workflow:**
   - `map` command: Drive robot around (teleop or exploration), build map
   - `save` command: Persist map to file (`~/.ros/rtabmap.db`)
   - `localize` command: Load saved map, localize only (no new mapping)
5. **Create navigation API:**
   - `navigate_to(x, y, theta)` - Send goal to Nav2
   - `get_pose()` - Get current robot position in map frame

### New Files

```
ros2_nodes/slam/
├── launch/rtabmap_slam.launch.py
├── launch/nav2_bringup.launch.py
├── config/rtabmap_params.yaml
└── config/nav2_params.yaml
deploy_slam.py          # Deployment CLI (like deploy_ros2_stack.py)
validation/test_slam.py # Validation tests
```

### Reference Code

Existing reference files in `reference/LanderPi/src/slam/`:
- `launch/rtabmap_slam.launch.py`
- `config/slam.yaml`
- `launch/include/rtabmap.launch.py`

---

## Phase 2: Arm Kinematics

**Goal:** Given a 3D point (x, y, z) relative to the robot base, calculate servo positions to reach it with the gripper.

### Components

| Component | Purpose |
|-----------|---------|
| **Forward Kinematics (FK)** | Servo positions → gripper 3D position |
| **Inverse Kinematics (IK)** | Target 3D position → servo positions |
| **Workspace limits** | Define reachable volume, safety bounds |

### Implementation Steps

1. **Measure arm geometry** - Link lengths, joint offsets, servo angle ranges (reference code has `transform.py` with some values)
2. **Implement FK** - DH parameters or geometric approach, verify against physical arm
3. **Implement IK** - Analytical solution for 5-DOF arm (reference `kinematics_control.py` uses iterative approach)
4. **Define workspace:**
   - Floor pickup zone: ~15-30cm in front of robot, 0-5cm height
   - Safe travel poses: arm tucked for navigation
5. **Create ROS2 service interface:**
   - `/kinematics/set_pose` - Move gripper to (x, y, z, pitch)
   - `/kinematics/get_pose` - Current gripper position
6. **Calibration routine** - Verify IK accuracy with physical measurements

### New Files

```
ros2_nodes/arm_kinematics/
├── arm_kinematics/kinematics_node.py
├── arm_kinematics/ik_solver.py
├── arm_kinematics/fk_solver.py
├── config/arm_geometry.yaml    # Link lengths, limits
└── launch/kinematics.launch.py
validation/test_kinematics.py   # IK accuracy tests
```

### Key Constraint

Lego bricks on floor require arm to reach down and forward. The 5-DOF arm should reach ~20-25cm forward at floor level - needs verification with physical measurements.

### Reference Code

Existing reference files in `reference/LanderPi/src/driver/kinematics/`:
- `kinematics/kinematics_control.py`
- `kinematics/transform.py`

---

## Phase 3: Object Localization

**Goal:** Detect Lego bricks with YOLO, get their 3D position in the map frame (not just "brick detected" but "brick at map coordinates x=2.3, y=1.1").

### Components

| Component | Purpose |
|-----------|---------|
| **YOLO detector** | Find Lego bricks in RGB image (already exists) |
| **Depth lookup** | Get distance to detected object from depth camera |
| **TF transform** | Convert camera-frame position → robot-frame → map-frame |

### Implementation Steps

1. **Train/fine-tune YOLO for Lego** - Current model detects generic objects; may need Lego-specific training or test with existing classes
2. **Pixel-to-3D projection:**
   - YOLO gives bounding box (u, v) in image
   - Depth camera gives distance at that pixel
   - Camera intrinsics convert to 3D point in camera frame
3. **TF chain setup:**
   ```
   map → odom → base_link → camera_link
   ```
   - rtabmap provides map→odom
   - Robot odometry provides odom→base_link
   - Static transform for camera mounting position (base_link→camera_link)
4. **Object tracking:**
   - Same brick seen from multiple angles → single map coordinate
   - Simple approach: cluster detections within 10cm radius
5. **Create detection API:**
   - `/lego/detections` - List of detected bricks with map coordinates
   - `/lego/nearest` - Closest brick to robot

### New Files

```
ros2_nodes/lego_detector/
├── lego_detector/detector_node.py
├── lego_detector/depth_projector.py
├── config/camera_intrinsics.yaml
└── launch/lego_detector.launch.py
config/yolo_hazards.json         # Add "lego" class or use existing small objects
```

---

## Phase 4: Pick & Place

**Goal:** Orchestrate all components: find brick → navigate close → pick up → navigate to drop zone → release.

### State Machine

```
IDLE → SEARCHING → APPROACHING → ALIGNING → PICKING → TRANSPORTING → DROPPING → IDLE
```

| State | Action | Success Condition |
|-------|--------|-------------------|
| **SEARCHING** | Explore or rotate, run YOLO | Brick detected with 3D position |
| **APPROACHING** | Nav2 goal ~30cm from brick | Within arm reach, facing brick |
| **ALIGNING** | Fine adjustment with depth camera | Brick centered, distance confirmed |
| **PICKING** | IK to position, lower arm, close gripper | Gripper closed, object sensed |
| **TRANSPORTING** | Nav2 to drop zone coordinates | At drop zone |
| **DROPPING** | Arm to drop position, open gripper | Gripper open |

### Implementation Steps

1. **Define drop zone** - Fixed map coordinates (e.g., corner with box), or ArUco marker for dynamic detection
2. **Approach planner** - Calculate Nav2 goal that positions robot optimally for arm reach
3. **Visual servoing (alignment)** - Fine-tune position using camera, not just Nav2 (Nav2 accuracy ~5-10cm, need ~2cm for pickup)
4. **Grasp verification** - Detect if pickup succeeded (gripper position feedback, or weight change if available)
5. **Error recovery** - What if brick moves? Pickup fails? Path blocked?

### New Files

```
ros2_nodes/lego_pickup/
├── lego_pickup/pickup_controller.py   # State machine
├── lego_pickup/approach_planner.py
├── lego_pickup/visual_servoing.py
└── launch/lego_pickup.launch.py
deploy_lego_pickup.py     # Full system deployment
validation/test_pickup.py # End-to-end test
```

---

## Implementation Order

| Step | Work | Builds On |
|------|------|-----------|
| 1.1 | Docker image with rtabmap-ros | Existing landerpi-ros2 |
| 1.2 | rtabmap launch + config (mapping mode) | 1.1 |
| 1.3 | Nav2 integration | 1.2 |
| 1.4 | Map save/load workflow | 1.2 |
| 1.5 | `deploy_slam.py` CLI + validation | 1.1-1.4 |
| 2.1 | Arm geometry measurement | Physical robot |
| 2.2 | FK/IK implementation | 2.1 |
| 2.3 | Kinematics ROS2 node | 2.2 |
| 2.4 | Floor reach validation | 2.3 |
| 3.1 | TF chain setup (camera transform) | 1.3 |
| 3.2 | Depth-to-3D projection | 3.1 |
| 3.3 | Lego detector node | 3.2 + existing YOLO |
| 4.1 | Pickup state machine | 1-3 complete |
| 4.2 | Visual servoing | 4.1 |
| 4.3 | End-to-end testing | 4.2 |

---

## Risks & Mitigations

| Risk | Mitigation |
|------|------------|
| Arm can't reach floor | Measure workspace first (Phase 2.1); may need robot to tilt forward |
| YOLO misses small Lego | Fine-tune model or use color-based detection as fallback |
| Nav2 accuracy insufficient | Visual servoing in Phase 4.2 handles fine alignment |
| rtabmap resource usage on Pi5 | Test early; may need to reduce map resolution |

---

## Success Criteria

**Phase 1 Complete:**
- Robot can build map of apartment by driving around
- Map persists across reboots
- Robot can navigate to arbitrary (x, y) coordinates with ~10cm accuracy

**Phase 2 Complete:**
- Given 3D point in robot frame, arm moves gripper there with ~1cm accuracy
- Floor pickup zone verified reachable

**Phase 3 Complete:**
- Lego bricks detected and localized in map frame
- Position accuracy ~5cm at 1m distance

**Phase 4 Complete:**
- Robot finds brick, picks it up, deposits in drop zone
- Success rate >80% on flat floor with good lighting

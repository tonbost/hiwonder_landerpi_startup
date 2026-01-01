# YOLOv11 Integration Plan

## Goal
Integrate YOLOv11n object detection into the LanderPi ROS2 stack to enable semantically aware obstacle avoidance, running on the Raspberry Pi 5.

## Proposed Changes

### git Configuration
- Create a new branch `feature/yolo-integration`.

### Docker Environment
#### [MODIFY] [setup_landerpi.py](file:///Users/ZDZ/Documents/gitrepo/personalproject/hiwonderSetup/setup_landerpi.py)
- Update the embedded Dockerfile string to install `ultralytics` and `onnxruntime`.

### ROS2 Nodes
#### [NEW] [ros2_nodes/yolo_detector](file:///Users/ZDZ/Documents/gitrepo/personalproject/hiwonderSetup/ros2_nodes/yolo_detector)
- Create a new ROS2 package `yolo_detector`.
- **Node**: `yolo_node`
    - **Subscribes**: `/aurora/rgb/image_color` (sensor_msgs/Image)
    - **Publishes**: `/yolo/detections` (vision_msgs/Detection2DArray)
    - **Logic**:
        - Load YOLOv11n model.
        - Inference on incoming images.
        - Publish detections.

#### [NEW] [ros2_nodes/obstacle_fusion](file:///Users/ZDZ/Documents/gitrepo/personalproject/hiwonderSetup/ros2_nodes/obstacle_fusion)
- Create a new ROS2 package `obstacle_fusion`.
- **Node**: `fusion_node`
    - **Subscribes**: `/yolo/detections` and `/scan`.
    - **Publishes**: `/hazards` (JSON string or custom msg).
    - **Logic**: Correlate detections with LiDAR to identify hazardous objects (e.g. "Person" < 1m).

### Logic Layer (Exploration Module)
#### [MODIFY] [validation/exploration/ros2_hardware.py](file:///Users/ZDZ/Documents/gitrepo/personalproject/hiwonderSetup/validation/exploration/ros2_hardware.py)
- Add `read_hazards()` method to read the `/hazards` topic.

#### [MODIFY] [validation/exploration/sensor_fusion.py](file:///Users/ZDZ/Documents/gitrepo/personalproject/hiwonderSetup/validation/exploration/sensor_fusion.py)
- Update `ObstacleState` to include semantic hazards.
- Update `SensorFusion` to digest hazard data from hardware.

#### [MODIFY] [validation/exploration/explorer.py](file:///Users/ZDZ/Documents/gitrepo/personalproject/hiwonderSetup/validation/exploration/explorer.py)
- Update `step()` loop to check for hazards and trigger avoidance/stop.

### Robot Scripts
#### [MODIFY] [robot_explorer.py](file:///Users/ZDZ/Documents/gitrepo/personalproject/hiwonderSetup/robot_explorer.py)
- Add `--enable-yolo` CLI argument.
- Pass this config down to the controller/hardware.

### Configuration
#### [MODIFY] [docker/docker-compose.yml](file:///Users/ZDZ/Documents/gitrepo/personalproject/hiwonderSetup/docker/docker-compose.yml)
- mount new nodes and update build command.

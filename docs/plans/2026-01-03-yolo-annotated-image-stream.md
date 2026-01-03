# Plan: Live YOLO Annotated Image Stream

**Date:** 2026-01-03
**Status:** Approved for Implementation

## Problem Statement

During autonomous exploration with YOLO detection, there's no way to see what the robot is detecting in real-time. The `--yolo-logging` flag saves annotated images to disk but this is post-hoc, not live.

**Current Flow:**
```
/aurora/rgb/image_raw → yolo_hailo_bridge → Hailo server → detections → /hazards
                                                                      → /yolo/detections
```

**Target Flow:**
```
/aurora/rgb/image_raw → yolo_hailo_bridge → Hailo server → detections → /hazards
        │                                                             → /yolo/detections
        │                                                             → /yolo/annotated_image (NEW)
        └───────────────────────────────────────────────────────────────────┘
                                    draw bboxes
```

## Architecture Design

### Component Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    Docker Container (landerpi-ros2)              │
│                                                                  │
│  ┌──────────────────┐         ┌─────────────────────────────┐   │
│  │  Aurora Camera   │────────▶│    yolo_hailo_bridge        │   │
│  │/aurora/rgb/image │         │                             │   │
│  └──────────────────┘         │  1. Send image to Hailo     │   │
│                               │  2. Receive detections      │   │
│                               │  3. Draw bboxes on image    │   │
│                               │  4. Publish annotated       │   │
│                               └─────────────────────────────┘   │
│                                        │                        │
│                    ┌───────────────────┼───────────────────┐    │
│                    ▼                   ▼                   ▼    │
│            ┌──────────────┐   ┌──────────────┐   ┌──────────────┐│
│            │/yolo/detections│   │  /hazards   │   │/yolo/annotated│
│            │Detection2DArray│   │   String    │   │    Image     │
│            └──────────────┘   └──────────────┘   └──────────────┘│
│                                                         │        │
└─────────────────────────────────────────────────────────│────────┘
                                                          │
                                                          ▼
                                              ┌──────────────────┐
                                              │  rqt_image_view  │
                                              │  web_video_server│
                                              │  or other viewer │
                                              └──────────────────┘
```

### Annotation Style

```
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│   ┌─────────────────────┐                                   │
│   │ ████████████████████│  ← Red box for hazard classes     │
│   │ █                  █│    (person, dog, cat, etc.)       │
│   │ █     PERSON       █│                                   │
│   │ █     0.92         █│  ← Class name + confidence        │
│   │ █                  █│                                   │
│   │ ████████████████████│                                   │
│   └─────────────────────┘                                   │
│                                                             │
│            ┌──────────┐                                     │
│            │ ████████ │  ← Green box for non-hazard objects │
│            │ █ chair █│                                     │
│            │ █ 0.78  █│                                     │
│            │ ████████ │                                     │
│            └──────────┘                                     │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Files to Modify

### 1. `ros2_nodes/yolo_hailo_bridge/yolo_hailo_bridge/bridge_node.py`

**Changes:**
- Add `cv2` import for drawing
- Add `cv_bridge` import for ROS2 ↔ OpenCV conversion
- Add parameter `publish_annotated` (default: True)
- Add publisher for `/yolo/annotated_image` (sensor_msgs/Image)
- Add `_draw_annotations()` method
- Add `_publish_annotated_image()` method
- Call annotation methods in `image_callback()` after receiving detections

**New imports:**
```python
import cv2
from cv_bridge import CvBridge
```

**New attributes:**
```python
self.cv_bridge = CvBridge()
self.annotated_publisher = self.create_publisher(Image, '/yolo/annotated_image', 10)
```

**New method `_draw_annotations()`:**
```python
def _draw_annotations(self, cv_image: np.ndarray, detections: list) -> np.ndarray:
    """Draw bounding boxes and labels on image."""
    for det in detections:
        # Calculate bbox corners from center + size
        cx, cy = int(det['cx']), int(det['cy'])
        w, h = int(det['width']), int(det['height'])
        x1, y1 = cx - w // 2, cy - h // 2
        x2, y2 = cx + w // 2, cy + h // 2

        # Red for hazards, green for others
        is_hazard = det['class'] in self.hazard_classes
        color = (0, 0, 255) if is_hazard else (0, 255, 0)  # BGR

        # Draw box
        cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)

        # Draw label background
        label = f"{det['class']} {det['score']:.2f}"
        (label_w, label_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(cv_image, (x1, y1 - label_h - 4), (x1 + label_w, y1), color, -1)

        # Draw label text
        cv2.putText(cv_image, label, (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return cv_image
```

**New method `_publish_annotated_image()`:**
```python
def _publish_annotated_image(self, original_msg: Image, detections: list):
    """Draw annotations and publish annotated image."""
    try:
        # Convert ROS image to OpenCV
        cv_image = self.cv_bridge.imgmsg_to_cv2(original_msg, desired_encoding='bgr8')

        # Draw annotations
        annotated = self._draw_annotations(cv_image, detections)

        # Convert back to ROS and publish
        annotated_msg = self.cv_bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = original_msg.header
        self.annotated_publisher.publish(annotated_msg)
    except Exception as e:
        self.get_logger().warn(f'Failed to publish annotated image: {e}')
```

**Lines to modify:**
| Location | Change |
|----------|--------|
| Line 2-10 | Update docstring to mention annotated output |
| Line 18 | Add cv2 import |
| After line 20 | Add cv_bridge import |
| Line 80-88 | Add annotated publisher |
| Line 168-172 | Add call to `_publish_annotated_image()` |
| After line 239 | Add new methods |

### 2. `ros2_nodes/yolo_hailo_bridge/package.xml`

**Changes:**
- Add `cv_bridge` dependency

**Lines to add:**
```xml
<depend>cv_bridge</depend>
```

### 3. `docker/docker-compose.yml`

No changes needed - `cv_bridge` is already available in ROS2 Humble base image.

## Implementation Steps

### Phase 1: Update Bridge Node
1. Add imports (cv2, cv_bridge)
2. Add CvBridge instance
3. Add annotated image publisher
4. Implement `_draw_annotations()` method
5. Implement `_publish_annotated_image()` method
6. Call annotation methods in `image_callback()`

### Phase 2: Update Package Dependencies
1. Add cv_bridge to package.xml

### Phase 3: Deploy and Test
1. Deploy: `uv run python deploy_ros2_stack.py deploy`
2. Start exploration: `uv run python deploy_explorer.py start --yolo-hailo --duration 2`
3. Verify topic: `ssh robot "docker exec landerpi-ros2 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic hz /yolo/annotated_image'"`

## Performance Expectations

| Metric | Value |
|--------|-------|
| Annotation overhead | ~1-2ms per frame |
| Additional bandwidth | ~1.8 MB/s (640x480 BGR @ 15 FPS) |
| Impact on FPS | Negligible (<5% reduction) |

## Viewing Options (Future)

The topic will be available for various viewers:

1. **rqt_image_view** - via X11 forwarding
2. **web_video_server** - browser-based (requires adding package to Docker)
3. **Custom script** - save frames locally via ROS2 bag or opencv

User chose: "Just publish topic" - viewing setup is out of scope for this implementation.

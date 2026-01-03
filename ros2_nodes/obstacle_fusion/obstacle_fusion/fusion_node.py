#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
import json
import math
import time
import numpy as np

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        # Parameters
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fov_horizontal', 60.0)
        self.declare_parameter('hazard_classes', ['person', 'cup', 'bottle', 'stop sign', 'dog', 'cat'])
        self.declare_parameter('hazard_distance', 1.5)  # meters - max distance to report hazard
        self.declare_parameter('use_depth_camera', True)  # Use depth camera for distance (more accurate)
        self.declare_parameter('depth_scale', 0.001)  # Depth image scale (1000 = mm to meters)

        self.img_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.img_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.fov_h = self.get_parameter('fov_horizontal').get_parameter_value().double_value
        self.hazard_classes = self.get_parameter('hazard_classes').get_parameter_value().string_array_value
        self.hazard_dist_thresh = self.get_parameter('hazard_distance').get_parameter_value().double_value
        self.use_depth = self.get_parameter('use_depth_camera').get_parameter_value().bool_value
        self.depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value

        # State
        self.latest_scan = None
        self.latest_detections = None
        self.latest_depth = None
        self.last_detection_time = 0
        self.last_scan_time = 0
        self.last_depth_time = 0

        # ROS Communications
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Detection2DArray, '/yolo/detections', self.detection_callback, 10)
        self.create_subscription(Image, '/aurora/depth/image_raw', self.depth_callback, 10)
        self.hazard_pub = self.create_publisher(String, '/hazards', 10)

        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        mode = "depth camera" if self.use_depth else "lidar"
        self.get_logger().info(f'Fusion Node initialized. Distance source: {mode}')

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.last_scan_time = time.time()

    def detection_callback(self, msg):
        self.latest_detections = msg
        self.last_detection_time = time.time()

    def depth_callback(self, msg):
        """Store latest depth image."""
        try:
            # Convert depth image to numpy array
            if msg.encoding in ('16UC1', 'mono16'):
                # 16-bit unsigned, millimeters
                self.latest_depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            elif msg.encoding == '32FC1':
                # 32-bit float, meters
                self.latest_depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            else:
                self.get_logger().warn(f'Unknown depth encoding: {msg.encoding}')
                return
            self.last_depth_time = time.time()
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')

    def timer_callback(self):
        # Check if data is fresh (within 1s)
        now = time.time()
        if (now - self.last_detection_time > 1.0):
            return

        # Need either depth or lidar for distance
        if self.use_depth:
            if (now - self.last_depth_time > 1.0) or self.latest_depth is None:
                return
        else:
            if (now - self.last_scan_time > 1.0) or self.latest_scan is None:
                return

        if not self.latest_detections:
            return

        hazards = []

        # Parse Detections
        for detection in self.latest_detections.detections:
            # Get class hypothesis
            if not detection.results:
                continue

            hypothesis = detection.results[0].hypothesis
            class_name = hypothesis.class_id
            score = hypothesis.score

            if class_name not in self.hazard_classes:
                continue

            # Get bounding box center
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            bbox_width = detection.bbox.size_x
            bbox_height = detection.bbox.size_y

            # Calculate angle from center (for logging)
            angle_from_center_deg = ((self.img_width / 2.0) - cx) / (self.img_width / 2.0) * (self.fov_h / 2.0)

            # Get distance - prefer depth camera, fallback to lidar
            if self.use_depth and self.latest_depth is not None:
                dist = self.get_depth_distance(cx, cy, bbox_width, bbox_height)
            else:
                dist = self.get_lidar_distance(angle_from_center_deg)

            if dist < self.hazard_dist_thresh:
                hazards.append({
                    "type": class_name,
                    "distance": round(dist, 2),
                    "angle": round(angle_from_center_deg, 1),
                    "score": round(score, 2),
                    "source": "depth" if self.use_depth else "lidar"
                })

        # Publish Hazards
        if hazards:
            msg = String()
            msg.data = json.dumps({"hazards": hazards})
            self.hazard_pub.publish(msg)
            # self.get_logger().info(f"Hazards: {msg.data}")

    def get_depth_distance(self, cx, cy, bbox_width, bbox_height):
        """
        Get distance from depth camera at bounding box location.

        Samples multiple regions of the bbox and returns the minimum valid depth:
        1. Center region (inner 50%)
        2. Lower region (bottom 30% of bbox) - catches floor-level objects
        3. Full bbox with sparse sampling

        This multi-region approach handles cases where:
        - Camera is higher than object (sees background through center)
        - Object is thin/narrow (center might miss the actual object)
        """
        if self.latest_depth is None:
            return float('inf')

        depth_img = self.latest_depth
        h, w = depth_img.shape

        # Convert bbox coordinates to depth image coordinates
        scale_x = w / self.img_width
        scale_y = h / self.img_height

        cx_depth = int(cx * scale_x)
        cy_depth = int(cy * scale_y)
        bbox_w_depth = int(bbox_width * scale_x)
        bbox_h_depth = int(bbox_height * scale_y)

        all_valid_depths = []

        # Region 1: Center of bbox (inner 50%) - original approach
        sample_w = max(int(bbox_w_depth * 0.5), 10)
        sample_h = max(int(bbox_h_depth * 0.5), 10)
        x1 = max(0, cx_depth - sample_w // 2)
        x2 = min(w, cx_depth + sample_w // 2)
        y1 = max(0, cy_depth - sample_h // 2)
        y2 = min(h, cy_depth + sample_h // 2)
        roi_center = depth_img[y1:y2, x1:x2]
        if roi_center.size > 0:
            valid = roi_center[(roi_center > 100) & (roi_center < 10000)]
            if valid.size > 0:
                all_valid_depths.extend(valid.tolist())

        # Region 2: Lower portion of bbox (bottom 30%)
        # This catches floor-level objects that camera looks "over"
        bbox_top = int((cy - bbox_height / 2) * scale_y)
        bbox_bottom = int((cy + bbox_height / 2) * scale_y)
        lower_y1 = max(0, bbox_bottom - int(bbox_h_depth * 0.3))
        lower_y2 = min(h, bbox_bottom)
        lower_x1 = max(0, cx_depth - sample_w // 2)
        lower_x2 = min(w, cx_depth + sample_w // 2)
        if lower_y2 > lower_y1 and lower_x2 > lower_x1:
            roi_lower = depth_img[lower_y1:lower_y2, lower_x1:lower_x2]
            if roi_lower.size > 0:
                valid = roi_lower[(roi_lower > 100) & (roi_lower < 10000)]
                if valid.size > 0:
                    all_valid_depths.extend(valid.tolist())

        # Region 3: Vertical strip through bbox center (for tall thin objects)
        strip_w = max(int(bbox_w_depth * 0.2), 5)
        strip_x1 = max(0, cx_depth - strip_w // 2)
        strip_x2 = min(w, cx_depth + strip_w // 2)
        strip_y1 = max(0, bbox_top)
        strip_y2 = min(h, bbox_bottom)
        if strip_y2 > strip_y1 and strip_x2 > strip_x1:
            roi_strip = depth_img[strip_y1:strip_y2, strip_x1:strip_x2]
            if roi_strip.size > 0:
                valid = roi_strip[(roi_strip > 100) & (roi_strip < 10000)]
                if valid.size > 0:
                    all_valid_depths.extend(valid.tolist())

        if not all_valid_depths:
            return float('inf')

        # Return minimum depth (closest point) converted to meters
        min_depth_mm = min(all_valid_depths)
        return float(min_depth_mm) * self.depth_scale

    def get_lidar_distance(self, angle_deg):
        """Get the minimum distance from LiDAR within +/- 5 degrees of target angle."""
        scan = self.latest_scan
        if not scan:
            return float('inf')

        # Lidar ranges are typically CCW (counter-clockwise)
        # angle_min usually -PI to +PI or similar.
        # Need to check frame. Assuming standard ROS convention: 0 is front, + is Left, - is Right.

        target_rad = math.radians(angle_deg)
        margin_rad = math.radians(10.0) # Check a cone

        min_d = float('inf')

        # Iterate relevant indices
        # Optimize: calculate index range directly
        # angle = min + i * inc
        # i = (angle - min) / inc

        start_angle = target_rad - margin_rad
        end_angle = target_rad + margin_rad

        # Wrap angles if needed (not strictly needed if adhering to -pi, pi limits but good practice)

        start_idx = int((start_angle - scan.angle_min) / scan.angle_increment)
        end_idx = int((end_angle - scan.angle_min) / scan.angle_increment)

        # Clamp indices
        start_idx = max(0, min(start_idx, len(scan.ranges)-1))
        end_idx = max(0, min(end_idx, len(scan.ranges)-1))

        # Determine step (could be negative if start > end, though logic above implies start < end)
        if start_idx > end_idx:
            # This should generally not happen with simple math unless wrap around logic is tricky
            # Just swap or fallback to full scan if unsure.
            # For now, simplistic approach:
             pass

        for i in range(start_idx, end_idx + 1):
             r = scan.ranges[i]
             if scan.range_min < r < scan.range_max:
                 if r < min_d:
                     min_d = r

        return min_d

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

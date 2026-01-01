#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
import json
import math
import time

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        # Parameters
        self.declare_parameter('image_width', 640)
        self.declare_parameter('fov_horizontal', 60.0)
        self.declare_parameter('hazard_classes', ['person', 'cup', 'bottle', 'stop sign'])
        self.declare_parameter('hazard_distance', 1.0) # meters

        self.img_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.fov_h = self.get_parameter('fov_horizontal').get_parameter_value().double_value
        self.hazard_classes = self.get_parameter('hazard_classes').get_parameter_value().string_array_value
        self.hazard_dist_thresh = self.get_parameter('hazard_distance').get_parameter_value().double_value

        # State
        self.latest_scan = None
        self.latest_detections = None
        self.last_detection_time = 0
        self.last_scan_time = 0

        # ROS Communications
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Detection2DArray, '/yolo/detections', self.detection_callback, 10)
        self.hazard_pub = self.create_publisher(String, '/hazards', 10)

        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        self.get_logger().info('Fusion Node initialized.')

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.last_scan_time = time.time()

    def detection_callback(self, msg):
        self.latest_detections = msg
        self.last_detection_time = time.time()

    def timer_callback(self):
        # Check if data is fresh (within 1s)
        now = time.time()
        if (now - self.last_scan_time > 1.0) or (now - self.last_detection_time > 1.0):
            return

        if not self.latest_scan or not self.latest_detections:
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

            # Calculate Angle
            # Center x in pixels
            cx = detection.bbox.center.position.x
            
            # Map [0, Width] -> [FOV/2, -FOV/2]
            # Center (Width/2) = 0 degrees
            # Left (0) = +FOV/2
            # Right (Width) = -FOV/2
            angle_from_center_deg = ((self.img_width / 2.0) - cx) / (self.img_width / 2.0) * (self.fov_h / 2.0)
            
            # Get distance from LiDAR at this angle
            dist = self.get_lidar_distance(angle_from_center_deg)

            if dist < self.hazard_dist_thresh:
                hazards.append({
                    "type": class_name,
                    "distance": round(dist, 2),
                    "angle": round(angle_from_center_deg, 1),
                    "score": round(score, 2)
                })

        # Publish Hazards
        if hazards:
            msg = String()
            msg.data = json.dumps({"hazards": hazards})
            self.hazard_pub.publish(msg)
            # self.get_logger().info(f"Hazards: {msg.data}")

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

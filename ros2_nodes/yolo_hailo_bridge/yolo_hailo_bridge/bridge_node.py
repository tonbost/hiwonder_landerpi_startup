#!/usr/bin/env python3
"""
YOLO Hailo Bridge Node - Connects ROS2 to host Hailo inference server.

Subscribes to camera images, sends to host via ZeroMQ for Hailo inference,
publishes detections back to ROS2 topics including annotated images.

Architecture:
  Docker (this node) ←→ ZeroMQ ←→ Host (hailo_inference_server.py + HailoRT)

Published Topics:
  /yolo/detections - Detection2DArray with bounding boxes
  /hazards - JSON string with hazard info for exploration
  /yolo/annotated_image - Camera frames with bounding boxes drawn
"""
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D

try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False


class YoloHailoBridgeNode(Node):
    def __init__(self):
        super().__init__('yolo_hailo_bridge')

        if not ZMQ_AVAILABLE:
            self.get_logger().error('pyzmq not available. Install with: pip install pyzmq')
            raise RuntimeError('pyzmq not available')

        # Parameters
        self.declare_parameter('server_host', 'localhost')
        self.declare_parameter('server_port', 5555)
        self.declare_parameter('timeout_ms', 500)

        server_host = self.get_parameter('server_host').get_parameter_value().string_value
        server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        self.timeout_ms = self.get_parameter('timeout_ms').get_parameter_value().integer_value

        # Load hazard config
        self.hazard_classes = ["person", "dog", "cat", "cup", "bottle", "stop sign"]
        self.hazard_distance = 2.5
        self.semantic_stop_distance = 0.8
        self._load_hazard_config()

        # Initialize ZeroMQ client
        self._zmq_ctx = zmq.Context()
        self.socket = self._zmq_ctx.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self.socket.setsockopt(zmq.LINGER, 0)

        server_addr = f"tcp://{server_host}:{server_port}"
        self.socket.connect(server_addr)
        self.get_logger().info(f'Connecting to Hailo server at {server_addr}')

        # Test connection
        if not self._ping_server():
            self.get_logger().warn('Hailo server not responding, will retry on first image')

        # Performance tracking
        self.frame_count = 0
        self.total_roundtrip_time = 0.0
        self.last_report_time = time.time()
        self.consecutive_failures = 0

        # ROS Communications
        self.subscription = self.create_subscription(
            Image,
            '/aurora/rgb/image_raw',
            self.image_callback,
            10)

        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10)

        # NOTE: Hazards are published by obstacle_fusion node which correlates
        # detections with depth camera for accurate distance. We only publish
        # /yolo/detections for obstacle_fusion to consume.
        # self.hazards_publisher = self.create_publisher(String, '/hazards', 10)

        self.annotated_publisher = self.create_publisher(
            Image,
            '/yolo/annotated_image',
            10)

        # CV Bridge for image conversion
        self.cv_bridge = CvBridge()

        self.get_logger().info('YOLO Hailo Bridge Node initialized')
        self.get_logger().info(f'Hazard classes: {self.hazard_classes}')

    def _load_hazard_config(self):
        """Load hazard classes from config file if available."""
        config_paths = [
            Path('/ros2_ws/config/yolo_hazards.json'),
            Path.home() / 'landerpi/config/yolo_hazards.json',
        ]
        for config_path in config_paths:
            if config_path.exists():
                try:
                    with open(config_path) as f:
                        config = json.load(f)
                    self.hazard_classes = config.get('hazard_classes', self.hazard_classes)
                    self.hazard_distance = config.get('hazard_distance', self.hazard_distance)
                    self.semantic_stop_distance = config.get('semantic_stop_distance', self.semantic_stop_distance)
                    self.get_logger().info(f'Loaded config from {config_path}')
                    break
                except Exception as e:
                    self.get_logger().warn(f'Failed to load hazard config: {e}')

    def _ping_server(self) -> bool:
        """Test if server is responding."""
        try:
            self.socket.send(json.dumps({'type': 'ping'}).encode())
            response = json.loads(self.socket.recv().decode())
            return response.get('status') == 'ok'
        except zmq.ZMQError:
            return False

    def _reconnect(self):
        """Reconnect to server after failures."""
        try:
            self.socket.close()
            self.socket = self._zmq_ctx.socket(zmq.REQ)
            self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
            self.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
            self.socket.setsockopt(zmq.LINGER, 0)

            server_host = self.get_parameter('server_host').get_parameter_value().string_value
            server_port = self.get_parameter('server_port').get_parameter_value().integer_value
            self.socket.connect(f"tcp://{server_host}:{server_port}")
            self.get_logger().info('Reconnected to server')
        except Exception as e:
            self.get_logger().error(f'Reconnect failed: {e}')

    def image_callback(self, msg):
        """Process incoming image."""
        start_time = time.time()

        try:
            # Build request
            request = {
                'type': 'infer',
                'width': msg.width,
                'height': msg.height,
                'channels': 3,
                'encoding': msg.encoding,
                'data': bytes(msg.data).hex(),
            }

            # Send to server
            self.socket.send(json.dumps(request).encode())
            response = json.loads(self.socket.recv().decode())

            if response.get('status') != 'ok':
                self.get_logger().warn(f"Server error: {response.get('message')}")
                return

            # Process results
            detections = response.get('detections', [])
            roundtrip_time = time.time() - start_time

            self.frame_count += 1
            self.total_roundtrip_time += roundtrip_time
            self.consecutive_failures = 0

            # Publish detections (obstacle_fusion will correlate with depth for /hazards)
            self._publish_detections(detections, msg.header)

            # Publish annotated image
            self._publish_annotated_image(msg, detections)

            # Report performance periodically
            if time.time() - self.last_report_time > 10.0:
                avg_fps = self.frame_count / self.total_roundtrip_time if self.total_roundtrip_time > 0 else 0
                inference_ms = response.get('inference_time_ms', 0)
                self.get_logger().info(
                    f'Performance: {avg_fps:.1f} FPS, {inference_ms:.1f}ms inference, {self.frame_count} frames'
                )
                self.last_report_time = time.time()

        except zmq.ZMQError as e:
            self.consecutive_failures += 1
            if self.consecutive_failures == 1:
                self.get_logger().warn(f'Hailo server communication error: {e}')
            if self.consecutive_failures >= 5:
                self.get_logger().warn('Multiple failures, attempting reconnect')
                self._reconnect()
                self.consecutive_failures = 0

    def _publish_detections(self, detections: list, header):
        """Publish Detection2DArray message."""
        msg = Detection2DArray()
        msg.header = header

        for det in detections:
            detection = Detection2D()
            detection.header = header

            bbox = BoundingBox2D()
            bbox.center.position.x = float(det['cx'])
            bbox.center.position.y = float(det['cy'])
            bbox.size_x = float(det['width'])
            bbox.size_y = float(det['height'])
            detection.bbox = bbox

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = det['class']
            hypothesis.hypothesis.score = float(det['score'])
            detection.results.append(hypothesis)

            msg.detections.append(detection)

        self.detection_publisher.publish(msg)

    def _draw_annotations(self, cv_image: np.ndarray, detections: list) -> np.ndarray:
        """Draw bounding boxes and labels on image.

        Args:
            cv_image: OpenCV image (BGR format)
            detections: List of detection dicts with cx, cy, width, height, class, score

        Returns:
            Annotated image with bounding boxes drawn
        """
        for det in detections:
            # Calculate bbox corners from center + size
            cx, cy = int(det['cx']), int(det['cy'])
            w, h = int(det['width']), int(det['height'])
            x1, y1 = cx - w // 2, cy - h // 2
            x2, y2 = cx + w // 2, cy + h // 2

            # Red for hazards, green for others (BGR format)
            is_hazard = det['class'] in self.hazard_classes
            color = (0, 0, 255) if is_hazard else (0, 255, 0)

            # Draw bounding box
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)

            # Draw label with background
            label = f"{det['class']} {det['score']:.2f}"
            (label_w, label_h), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            # Label background
            cv2.rectangle(
                cv_image,
                (x1, y1 - label_h - 6),
                (x1 + label_w + 4, y1),
                color,
                -1
            )
            # Label text (white)
            cv2.putText(
                cv_image,
                label,
                (x1 + 2, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )

        return cv_image

    def _publish_annotated_image(self, original_msg: Image, detections: list):
        """Draw annotations on image and publish.

        Args:
            original_msg: Original ROS2 Image message
            detections: List of detection dicts from Hailo server
        """
        try:
            # Convert ROS image to OpenCV BGR
            cv_image = self.cv_bridge.imgmsg_to_cv2(original_msg, desired_encoding='bgr8')

            # Draw annotations
            annotated = self._draw_annotations(cv_image, detections)

            # Convert back to ROS message and publish
            annotated_msg = self.cv_bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = original_msg.header
            self.annotated_publisher.publish(annotated_msg)

        except Exception as e:
            # Don't spam logs, just skip this frame
            pass

    def destroy_node(self):
        """Cleanup on shutdown."""
        self.socket.close()
        self._zmq_ctx.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloHailoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

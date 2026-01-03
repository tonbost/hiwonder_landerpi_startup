#!/usr/bin/env python3
"""
YOLO Object Detection Node for LanderPi.

Subscribes to camera images, runs YOLOv11 inference, publishes detections.
Optional debug logging saves annotated images and detection history.
"""
import json
import os
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Model parameters
        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('device', 'cpu')

        # Logging parameters (for debugging)
        self.declare_parameter('enable_logging', False)
        self.declare_parameter('log_dir', str(Path.home() / 'yolo_logs'))
        self.declare_parameter('log_all_frames', False)  # False = only log frames with detections
        self.declare_parameter('max_log_images', 500)  # Auto-cleanup old images

        # Get parameters
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_thres = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        device = self.get_parameter('device').get_parameter_value().string_value

        self.enable_logging = self.get_parameter('enable_logging').get_parameter_value().bool_value
        self.log_dir = Path(self.get_parameter('log_dir').get_parameter_value().string_value)
        self.log_all_frames = self.get_parameter('log_all_frames').get_parameter_value().bool_value
        self.max_log_images = self.get_parameter('max_log_images').get_parameter_value().integer_value

        # Load model
        self.get_logger().info(f'Loading YOLO model: {model_path} on {device}...')
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise e
        self.get_logger().info('Model loaded successfully.')

        # Setup logging if enabled
        self.session_dir = None
        self.detections_file = None
        self.frame_count = 0
        self.detection_count = 0

        if self.enable_logging:
            self._setup_logging()

        # ROS Communications
        self.subscription = self.create_subscription(
            Image,
            '/aurora/rgb/image_raw',
            self.image_callback,
            10)

        self.publisher = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10)

        log_status = "enabled" if self.enable_logging else "disabled"
        self.get_logger().info(f'YOLO Detector Node initialized. Logging: {log_status}')

    def _setup_logging(self):
        """Setup logging directory and files."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session_dir = self.log_dir / f'session_{timestamp}'
        self.session_dir.mkdir(parents=True, exist_ok=True)

        # Create images subdirectory
        (self.session_dir / 'images').mkdir(exist_ok=True)

        # Open detections log file
        self.detections_file = open(self.session_dir / 'detections.jsonl', 'w')

        # Write session metadata
        metadata = {
            'start_time': timestamp,
            'model': self.get_parameter('model_path').get_parameter_value().string_value,
            'confidence_threshold': self.conf_thres,
            'log_all_frames': self.log_all_frames,
        }
        with open(self.session_dir / 'metadata.json', 'w') as f:
            json.dump(metadata, f, indent=2)

        self.get_logger().info(f'Logging to: {self.session_dir}')

    def _cleanup_old_images(self):
        """Remove oldest images if exceeding max_log_images."""
        if not self.session_dir:
            return

        images_dir = self.session_dir / 'images'
        images = sorted(images_dir.glob('*.jpg'), key=lambda p: p.stat().st_mtime)

        while len(images) > self.max_log_images:
            oldest = images.pop(0)
            oldest.unlink()
            self.get_logger().debug(f'Cleaned up old image: {oldest.name}')

    def imgmsg_to_cv2(self, msg):
        """Convert ROS Image to OpenCV image without cv_bridge."""
        height = msg.height
        width = msg.width
        encoding = msg.encoding

        if encoding == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif encoding == 'bgr8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
        elif encoding == 'mono8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            self.get_logger().warn(f'Unknown encoding: {encoding}, attempting generic conversion')
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, -1)
            if img.shape[2] == 3:
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        return img

    def _draw_detections(self, image, detections):
        """Draw bounding boxes and labels on image."""
        annotated = image.copy()

        for det in detections:
            cx, cy = det['cx'], det['cy']
            w, h = det['width'], det['height']
            x1 = int(cx - w / 2)
            y1 = int(cy - h / 2)
            x2 = int(cx + w / 2)
            y2 = int(cy + h / 2)

            # Draw box
            color = (0, 255, 0)  # Green
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            # Draw label
            label = f"{det['class']} {det['score']:.2f}"
            (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(annotated, (x1, y1 - label_h - 4), (x1 + label_w, y1), color, -1)
            cv2.putText(annotated, label, (x1, y1 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        return annotated

    def _log_frame(self, image, detections, timestamp):
        """Log frame and detections to disk."""
        if not self.enable_logging or not self.session_dir:
            return

        self.frame_count += 1
        has_detections = len(detections) > 0

        # Skip if no detections and not logging all frames
        if not has_detections and not self.log_all_frames:
            return

        # Generate filename
        ts_str = datetime.now().strftime('%H%M%S_%f')[:-3]  # HHMMSSmmm
        filename = f'frame_{self.frame_count:06d}_{ts_str}'

        # Save annotated image
        if has_detections:
            self.detection_count += 1
            annotated = self._draw_detections(image, detections)
            image_path = self.session_dir / 'images' / f'{filename}_det.jpg'
        else:
            annotated = image
            image_path = self.session_dir / 'images' / f'{filename}.jpg'

        cv2.imwrite(str(image_path), annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])

        # Log detection record
        record = {
            'timestamp': timestamp,
            'frame': self.frame_count,
            'image': image_path.name,
            'detections': detections,
        }
        self.detections_file.write(json.dumps(record) + '\n')
        self.detections_file.flush()

        # Cleanup old images periodically
        if self.frame_count % 100 == 0:
            self._cleanup_old_images()

    def image_callback(self, msg):
        try:
            cv_image = self.imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        timestamp = time.time()

        # Inference
        results = self.model(cv_image, verbose=False, conf=self.conf_thres, device='cpu')

        # Process results
        detection_msg = Detection2DArray()
        detection_msg.header = msg.header
        detections_list = []  # For logging

        if results and len(results) > 0:
            result = results[0]

            boxes = result.boxes.xywh.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy()
            scores = result.boxes.conf.cpu().numpy()
            names = result.names

            for i in range(len(boxes)):
                box = boxes[i]
                cls_id = int(classes[i])
                score = float(scores[i])
                class_name = names[cls_id]

                # ROS message
                detection = Detection2D()
                detection.header = msg.header

                bbox = BoundingBox2D()
                bbox.center.position.x = float(box[0])
                bbox.center.position.y = float(box[1])
                bbox.size_x = float(box[2])
                bbox.size_y = float(box[3])
                detection.bbox = bbox

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(class_name)
                hypothesis.hypothesis.score = score
                detection.results.append(hypothesis)

                detection_msg.detections.append(detection)

                # For logging
                detections_list.append({
                    'class': class_name,
                    'score': round(score, 3),
                    'cx': float(box[0]),
                    'cy': float(box[1]),
                    'width': float(box[2]),
                    'height': float(box[3]),
                })

        self.publisher.publish(detection_msg)

        # Log if enabled
        if self.enable_logging:
            self._log_frame(cv_image, detections_list, timestamp)

    def destroy_node(self):
        """Cleanup on shutdown."""
        if self.detections_file:
            # Write summary
            summary = {
                'end_time': datetime.now().strftime('%Y%m%d_%H%M%S'),
                'total_frames': self.frame_count,
                'frames_with_detections': self.detection_count,
            }
            self.detections_file.write(f'# SUMMARY: {json.dumps(summary)}\n')
            self.detections_file.close()
            self.get_logger().info(f'Logging complete: {self.frame_count} frames, {self.detection_count} with detections')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

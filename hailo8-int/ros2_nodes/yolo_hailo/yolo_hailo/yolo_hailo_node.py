#!/usr/bin/env python3
"""
Hailo-accelerated YOLO Object Detection Node for LanderPi.

Subscribes to camera images, runs YOLOv11 inference on Hailo-8, publishes detections.
Same interface as yolo_node.py (CPU version) for drop-in replacement.
"""
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D

# Hailo imports
try:
    from hailo_platform import (
        HEF, VDevice, HailoStreamInterface, ConfigureParams,
        InferVStreams, InputVStreamParams, OutputVStreamParams, FormatType
    )
    HAILO_AVAILABLE = True
except ImportError:
    HAILO_AVAILABLE = False


# COCO class names (same as ultralytics YOLOv11)
COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]


class YoloHailoNode(Node):
    def __init__(self):
        super().__init__('yolo_hailo_detector')

        if not HAILO_AVAILABLE:
            self.get_logger().error('hailo_platform not available. Install with: pip install hailort')
            raise RuntimeError('Hailo platform not available')

        # Model parameters
        self.declare_parameter('hef_path', str(Path.home() / 'landerpi/hailo/models/yolov11n.hef'))
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)

        hef_path = self.get_parameter('hef_path').get_parameter_value().string_value
        self.conf_thres = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.nms_thres = self.get_parameter('nms_threshold').get_parameter_value().double_value
        self.input_width = self.get_parameter('input_width').get_parameter_value().integer_value
        self.input_height = self.get_parameter('input_height').get_parameter_value().integer_value

        # Load hazard config
        self.hazard_classes = ["person", "dog", "cat", "cup", "bottle", "stop sign"]
        self.hazard_distance = 2.5
        self.semantic_stop_distance = 0.8
        self._load_hazard_config()

        # Initialize Hailo device
        self.get_logger().info(f'Loading HEF model: {hef_path}')
        try:
            self.hef = HEF(hef_path)
            self.vdevice = VDevice()

            # Configure network
            configure_params = ConfigureParams.create_from_hef(self.hef, interface=HailoStreamInterface.PCIe)
            self.network_group = self.vdevice.configure(self.hef, configure_params)[0]
            self.network_group_params = self.network_group.create_params()

            # Get input/output info
            self.input_vstream_info = self.hef.get_input_vstream_infos()[0]
            self.output_vstream_infos = self.hef.get_output_vstream_infos()

            self.get_logger().info(f'Model loaded. Input shape: {self.input_vstream_info.shape}')
            self.get_logger().info(f'Output layers: {len(self.output_vstream_infos)}')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize Hailo: {e}')
            raise

        # Performance tracking
        self.frame_count = 0
        self.total_inference_time = 0.0
        self.last_fps_report = time.time()

        # ROS Communications (same interface as CPU node)
        self.subscription = self.create_subscription(
            Image,
            '/aurora/rgb/image_raw',
            self.image_callback,
            10)

        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10)

        self.hazards_publisher = self.create_publisher(
            String,
            '/hazards',
            10)

        self.get_logger().info('Hailo YOLO Detector Node initialized')
        self.get_logger().info(f'Hazard classes: {self.hazard_classes}')

    def _load_hazard_config(self):
        """Load hazard classes from config file if available."""
        config_paths = [
            Path.home() / 'landerpi/config/yolo_hazards.json',
            Path('/ros2_ws/config/yolo_hazards.json'),
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

    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """
        Preprocess image for Hailo inference.

        Args:
            image: BGR image from camera (H, W, 3)

        Returns:
            Preprocessed image ready for Hailo (1, H, W, 3) float32 RGB
        """
        # Resize to model input size
        resized = cv2.resize(image, (self.input_width, self.input_height))

        # Convert BGR to RGB
        resized = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        # Normalize to 0-1 range (YOLO expects normalized input)
        normalized = resized.astype(np.float32) / 255.0

        # Add batch dimension
        return np.expand_dims(normalized, axis=0)

    def postprocess(self, outputs: dict, orig_width: int, orig_height: int) -> list:
        """
        Postprocess Hailo outputs to detections.

        Args:
            outputs: Dictionary of output tensors from Hailo
            orig_width: Original image width
            orig_height: Original image height

        Returns:
            List of detections with format:
            [{'class': str, 'score': float, 'cx': float, 'cy': float, 'width': float, 'height': float}, ...]
        """
        detections = []

        # YOLOv11 output format: typically one output tensor with shape (1, num_detections, 84)
        # where 84 = 4 bbox coords + 80 class scores
        # This depends on the specific HEF export configuration

        # Get the main output tensor (usually the first one)
        if not outputs:
            return detections

        output_name = list(outputs.keys())[0]
        output_data = outputs[output_name]

        # Expected shape: (1, num_detections, 84) or (1, 84, num_detections)
        # Need to transpose if necessary
        if output_data.shape[-1] != 84 and output_data.shape[1] == 84:
            output_data = np.transpose(output_data, (0, 2, 1))

        # Remove batch dimension
        if len(output_data.shape) == 3:
            output_data = output_data[0]  # Shape: (num_detections, 84)

        # Parse detections
        for detection in output_data:
            # First 4 values are bbox (cx, cy, w, h) in normalized coordinates
            cx, cy, w, h = detection[:4]

            # Remaining 80 values are class scores
            class_scores = detection[4:]

            # Get best class
            class_id = np.argmax(class_scores)
            confidence = float(class_scores[class_id])

            # Filter by confidence threshold
            if confidence < self.conf_thres:
                continue

            # Convert normalized coords to pixel coords
            cx_pixel = cx * orig_width
            cy_pixel = cy * orig_height
            w_pixel = w * orig_width
            h_pixel = h * orig_height

            # Get class name
            if class_id < len(COCO_CLASSES):
                class_name = COCO_CLASSES[class_id]
            else:
                class_name = f"class_{class_id}"

            detections.append({
                'class': class_name,
                'score': confidence,
                'cx': float(cx_pixel),
                'cy': float(cy_pixel),
                'width': float(w_pixel),
                'height': float(h_pixel),
            })

        # Apply NMS if multiple detections
        if len(detections) > 1:
            detections = self._apply_nms(detections)

        return detections

    def _apply_nms(self, detections: list) -> list:
        """
        Apply Non-Maximum Suppression to remove overlapping detections.

        Args:
            detections: List of detection dictionaries

        Returns:
            Filtered list of detections
        """
        if not detections:
            return detections

        # Convert to format for cv2.dnn.NMSBoxes
        boxes = []
        scores = []

        for det in detections:
            # Convert center format to corner format
            x1 = det['cx'] - det['width'] / 2
            y1 = det['cy'] - det['height'] / 2
            boxes.append([x1, y1, det['width'], det['height']])
            scores.append(det['score'])

        # Apply NMS
        indices = cv2.dnn.NMSBoxes(boxes, scores, self.conf_thres, self.nms_thres)

        # Filter detections
        if len(indices) > 0:
            indices = indices.flatten()
            return [detections[i] for i in indices]

        return []

    def image_callback(self, msg):
        """Process incoming image."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.imgmsg_to_cv2(msg)

        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        orig_height, orig_width = cv_image.shape[:2]
        timestamp = time.time()

        try:
            # Preprocess
            input_data = self.preprocess(cv_image)

            # Run inference on Hailo
            start_time = time.time()

            with InferVStreams(self.network_group, self.network_group_params) as infer_pipeline:
                input_dict = {self.input_vstream_info.name: input_data}
                output_dict = infer_pipeline.infer(input_dict)

            inference_time = time.time() - start_time
            self.total_inference_time += inference_time
            self.frame_count += 1

            # Postprocess
            detections = self.postprocess(output_dict, orig_width, orig_height)

            # Publish detections (same format as CPU node)
            self._publish_detections(detections, msg.header)

            # Publish hazards (same format as CPU node)
            self._publish_hazards(detections)

            # Report FPS periodically
            if time.time() - self.last_fps_report > 10.0:
                avg_fps = self.frame_count / self.total_inference_time if self.total_inference_time > 0 else 0
                avg_latency_ms = (self.total_inference_time / self.frame_count * 1000) if self.frame_count > 0 else 0
                self.get_logger().info(
                    f'Performance: {avg_fps:.1f} FPS, {avg_latency_ms:.1f}ms latency, {self.frame_count} frames'
                )
                self.last_fps_report = time.time()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def _publish_detections(self, detections: list, header):
        """
        Publish Detection2DArray message.

        Same format as CPU node for drop-in replacement.
        """
        msg = Detection2DArray()
        msg.header = header

        for det in detections:
            detection = Detection2D()
            detection.header = header

            bbox = BoundingBox2D()
            bbox.center.position.x = det['cx']
            bbox.center.position.y = det['cy']
            bbox.size_x = det['width']
            bbox.size_y = det['height']
            detection.bbox = bbox

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = det['class']
            hypothesis.hypothesis.score = det['score']
            detection.results.append(hypothesis)

            msg.detections.append(detection)

        self.detection_publisher.publish(msg)

    def _publish_hazards(self, detections: list):
        """
        Publish hazards for exploration controller.

        Same format as CPU node for drop-in replacement.
        """
        hazards = []
        for det in detections:
            if det['class'] in self.hazard_classes:
                # Estimate distance based on bounding box size
                # Larger boxes = closer objects
                # This is a heuristic; actual distance would require depth data
                bbox_area = det['width'] * det['height']
                estimated_distance = min(self.hazard_distance, 3000.0 / max(bbox_area, 100.0))

                # Estimate angle based on horizontal position
                # Assuming camera FOV ~60 degrees
                image_center_x = 320  # Typical for 640x480
                angle_deg = (det['cx'] - image_center_x) / image_center_x * 30.0

                hazards.append({
                    'type': det['class'],
                    'distance': estimated_distance,
                    'angle': angle_deg,
                    'score': det['score'],
                    'source': 'hailo'
                })

        if hazards:
            msg = String()
            msg.data = json.dumps({'hazards': hazards})
            self.hazards_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloHailoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

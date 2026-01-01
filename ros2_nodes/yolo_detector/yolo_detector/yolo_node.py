#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Parameters
        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('device', 'cpu') # Use 'cpu' for RPi, or 'cuda' if using Jetson (RPi has no CUDA)
        
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_thres = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        device = self.get_parameter('device').get_parameter_value().string_value

        self.get_logger().info(f'Loading YOLO model: {model_path} on {device}...')
        try:
            self.model = YOLO(model_path)
            # Export to ONNX or other formats if needed for speed, but pure pt is easiest to start
            # self.model.export(format='onnx') 
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise e

        self.get_logger().info('Model loaded successfully.')
        
        # ROS Communications
        self.subscription = self.create_subscription(
            Image,
            '/aurora/rgb/image_color',
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10)

        self.bridge = CvBridge()
        self.get_logger().info('YOLO Detector Node initialized.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Inference
        results = self.model(cv_image, verbose=False, conf=self.conf_thres, device='cpu') 
        
        # Process results
        detection_msg = Detection2DArray()
        detection_msg.header = msg.header
        
        if results and len(results) > 0:
            result = results[0] # Single image batch
            
            # Boxes in xywh format (center_x, center_y, width, height)
            boxes = result.boxes.xywh.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy()
            scores = result.boxes.conf.cpu().numpy()
            names = result.names
            
            for i in range(len(boxes)):
                box = boxes[i]
                cls_id = int(classes[i])
                score = float(scores[i])
                class_name = names[cls_id]
                
                detection = Detection2D()
                detection.header = msg.header
                
                # Bounding Box
                bbox = BoundingBox2D()
                bbox.center.position.x = float(box[0])
                bbox.center.position.y = float(box[1])
                bbox.size_x = float(box[2])
                bbox.size_y = float(box[3])
                detection.bbox = bbox
                
                # Hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(class_name) # ROS2 vision_msgs uses string class_id
                hypothesis.hypothesis.score = score
                detection.results.append(hypothesis)
                
                detection_msg.detections.append(detection)

        self.publisher.publish(detection_msg)

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

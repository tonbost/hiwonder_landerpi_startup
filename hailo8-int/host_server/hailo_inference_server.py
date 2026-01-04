#!/usr/bin/env python3
"""
Hailo Inference Server - Runs on host with HailoRT.

Receives images via ZeroMQ, runs YOLOv11 inference on Hailo-8 NPU,
returns detections. Designed to bridge Docker container (ROS2) with
host-only HailoRT Python bindings.

Usage:
    python3 hailo_inference_server.py [--port 5555] [--model yolov11n.hef]
"""
import argparse
import json
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import zmq

# Hailo imports
try:
    from hailo_platform import (
        HEF, VDevice, HailoStreamInterface, ConfigureParams,
        InferVStreams, InputVStreamParams, OutputVStreamParams
    )
    HAILO_AVAILABLE = True
except ImportError:
    HAILO_AVAILABLE = False
    print("ERROR: hailo_platform not available. Install HailoRT.")
    sys.exit(1)


# COCO class names (YOLOv11)
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


class HailoInferenceServer:
    def __init__(self, model_path: str, port: int = 5555,
                 conf_threshold: float = 0.4, nms_threshold: float = 0.45):
        self.port = port
        self.conf_thres = conf_threshold
        self.nms_thres = nms_threshold
        self.input_width = 640
        self.input_height = 640
        self.running = False

        # Performance tracking
        self.frame_count = 0
        self.total_inference_time = 0.0
        self.last_report_time = time.time()

        # Initialize Hailo
        print(f"Loading HEF model: {model_path}")
        self.hef = HEF(model_path)
        self.vdevice = VDevice()

        configure_params = ConfigureParams.create_from_hef(
            self.hef, interface=HailoStreamInterface.PCIe
        )
        self.network_group = self.vdevice.configure(self.hef, configure_params)[0]
        self.network_group_params = self.network_group.create_params()

        self.input_vstream_info = self.hef.get_input_vstream_infos()[0]
        self.output_vstream_infos = self.hef.get_output_vstream_infos()

        # Create vstream params for HailoRT 4.23+ API
        self.input_vstream_params = InputVStreamParams.make(self.network_group)
        self.output_vstream_params = OutputVStreamParams.make(self.network_group)

        print(f"Model loaded. Input: {self.input_vstream_info.shape}")
        print(f"Output layers: {len(self.output_vstream_infos)}")

        # Activate network group (must be context manager wrapping InferVStreams)
        self.ng_params = self.network_group.create_params()
        self.activated_ng = self.network_group.activate(self.ng_params)
        self.activated_ng.__enter__()

        # Create persistent inference pipeline
        self.infer_pipeline = InferVStreams(self.network_group, self.input_vstream_params, self.output_vstream_params)
        self.infer_pipeline.__enter__()
        print("Hailo inference pipeline activated")

        # Initialize ZeroMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://*:{port}")
        print(f"ZeroMQ server listening on port {port}")

    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """Preprocess image for Hailo inference (uint8 format)."""
        resized = cv2.resize(image, (self.input_width, self.input_height))
        resized = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        # Hailo expects uint8, not float32
        return np.expand_dims(resized, axis=0).astype(np.uint8)

    def postprocess(self, outputs: dict, orig_width: int, orig_height: int) -> list:
        """Postprocess Hailo NMS outputs to detections.

        Hailo yolov8_nms_postprocess output format:
        - output[0] = list of 80 classes
        - output[0][class_id] = numpy array of shape (N, 5) for N detections
          where each detection is [y1, x1, y2, x2, score] normalized to 0-1
        """
        detections = []

        if not outputs:
            return detections

        output_name = list(outputs.keys())[0]
        output_data = outputs[output_name]

        # Handle Hailo NMS postprocess format: list of class detections
        if isinstance(output_data, list):
            # output_data[0] is list of 80 classes
            batch_result = output_data[0] if len(output_data) > 0 else []

            for class_id, class_detections in enumerate(batch_result):
                if not isinstance(class_detections, np.ndarray) or len(class_detections) == 0:
                    continue

                class_name = COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES) else f"class_{class_id}"

                for det in class_detections:
                    # Hailo NMS format: [y1, x1, y2, x2, score]
                    y1, x1, y2, x2, score = det[:5]

                    if score < self.conf_thres:
                        continue

                    # Convert to center format and scale to original image
                    cx_pixel = (x1 + x2) / 2 * orig_width
                    cy_pixel = (y1 + y2) / 2 * orig_height
                    w_pixel = (x2 - x1) * orig_width
                    h_pixel = (y2 - y1) * orig_height

                    detections.append({
                        'class': class_name,
                        'score': round(float(score), 3),
                        'cx': round(float(cx_pixel), 1),
                        'cy': round(float(cy_pixel), 1),
                        'width': round(float(w_pixel), 1),
                        'height': round(float(h_pixel), 1),
                    })
        else:
            # Fallback for raw tensor output (no NMS)
            if hasattr(output_data, 'shape'):
                if output_data.shape[-1] != 84 and len(output_data.shape) > 2 and output_data.shape[1] == 84:
                    output_data = np.transpose(output_data, (0, 2, 1))
                if len(output_data.shape) == 3:
                    output_data = output_data[0]

                for detection in output_data:
                    cx, cy, w, h = detection[:4]
                    class_scores = detection[4:]
                    class_id = np.argmax(class_scores)
                    confidence = float(class_scores[class_id])

                    if confidence < self.conf_thres:
                        continue

                    cx_pixel = cx * orig_width
                    cy_pixel = cy * orig_height
                    w_pixel = w * orig_width
                    h_pixel = h * orig_height

                    class_name = COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES) else f"class_{class_id}"

                    detections.append({
                        'class': class_name,
                        'score': round(confidence, 3),
                        'cx': round(float(cx_pixel), 1),
                        'cy': round(float(cy_pixel), 1),
                        'width': round(float(w_pixel), 1),
                        'height': round(float(h_pixel), 1),
                    })

        # NMS already applied by Hailo for list format, apply for raw tensor
        if not isinstance(outputs[output_name], list) and len(detections) > 1:
            detections = self._apply_nms(detections)

        return detections

    def _apply_nms(self, detections: list) -> list:
        """Apply Non-Maximum Suppression."""
        if not detections:
            return detections

        boxes = []
        scores = []
        for det in detections:
            x1 = det['cx'] - det['width'] / 2
            y1 = det['cy'] - det['height'] / 2
            boxes.append([x1, y1, det['width'], det['height']])
            scores.append(det['score'])

        indices = cv2.dnn.NMSBoxes(boxes, scores, self.conf_thres, self.nms_thres)

        if len(indices) > 0:
            indices = indices.flatten()
            return [detections[i] for i in indices]
        return []

    def infer(self, image: np.ndarray) -> list:
        """Run inference on image."""
        orig_height, orig_width = image.shape[:2]

        input_data = self.preprocess(image)

        start_time = time.time()
        input_dict = {self.input_vstream_info.name: input_data}
        output_dict = self.infer_pipeline.infer(input_dict)
        inference_time = time.time() - start_time

        self.frame_count += 1
        self.total_inference_time += inference_time

        detections = self.postprocess(output_dict, orig_width, orig_height)
        return detections, inference_time

    def handle_request(self, message: bytes) -> bytes:
        """Handle incoming inference request."""
        try:
            # Decode request
            request = json.loads(message.decode('utf-8'))

            if request.get('type') == 'ping':
                return json.dumps({'status': 'ok', 'type': 'pong'}).encode()

            if request.get('type') == 'stats':
                avg_fps = self.frame_count / self.total_inference_time if self.total_inference_time > 0 else 0
                return json.dumps({
                    'status': 'ok',
                    'frames': self.frame_count,
                    'avg_fps': round(avg_fps, 1),
                }).encode()

            if request.get('type') != 'infer':
                return json.dumps({'status': 'error', 'message': 'Unknown request type'}).encode()

            # Decode image
            width = request['width']
            height = request['height']
            channels = request.get('channels', 3)
            encoding = request.get('encoding', 'rgb8')

            img_data = bytes.fromhex(request['data'])
            image = np.frombuffer(img_data, dtype=np.uint8).reshape(height, width, channels)

            if encoding == 'rgb8':
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # Run inference
            detections, inference_time = self.infer(image)

            # Return result
            return json.dumps({
                'status': 'ok',
                'detections': detections,
                'inference_time_ms': round(inference_time * 1000, 1),
            }).encode()

        except Exception as e:
            return json.dumps({'status': 'error', 'message': str(e)}).encode()

    def run(self):
        """Main server loop."""
        self.running = True
        print("Hailo inference server running. Press Ctrl+C to stop.")

        while self.running:
            try:
                # Wait for request with timeout
                if self.socket.poll(1000):  # 1 second timeout
                    message = self.socket.recv()
                    response = self.handle_request(message)
                    self.socket.send(response)

                # Report stats periodically
                if time.time() - self.last_report_time > 30.0 and self.frame_count > 0:
                    avg_fps = self.frame_count / self.total_inference_time
                    avg_ms = self.total_inference_time / self.frame_count * 1000
                    print(f"Stats: {self.frame_count} frames, {avg_fps:.1f} FPS, {avg_ms:.1f}ms avg")
                    self.last_report_time = time.time()

            except zmq.ZMQError as e:
                if e.errno == zmq.ETERM:
                    break
                print(f"ZMQ error: {e}")

    def shutdown(self):
        """Clean shutdown."""
        self.running = False
        self.infer_pipeline.__exit__(None, None, None)
        self.activated_ng.__exit__(None, None, None)
        self.socket.close()
        self.context.term()
        print("Server shutdown complete.")


def main():
    parser = argparse.ArgumentParser(description='Hailo Inference Server')
    parser.add_argument('--port', type=int, default=5555, help='ZeroMQ port')
    parser.add_argument('--model', type=str,
                        default=str(Path.home() / 'landerpi/hailo/models/yolov11n.hef'),
                        help='Path to HEF model')
    parser.add_argument('--conf', type=float, default=0.4, help='Confidence threshold')
    parser.add_argument('--nms', type=float, default=0.45, help='NMS threshold')
    args = parser.parse_args()

    server = HailoInferenceServer(
        model_path=args.model,
        port=args.port,
        conf_threshold=args.conf,
        nms_threshold=args.nms,
    )

    def signal_handler(sig, frame):
        print("\nShutting down...")
        server.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    server.run()


if __name__ == '__main__':
    main()

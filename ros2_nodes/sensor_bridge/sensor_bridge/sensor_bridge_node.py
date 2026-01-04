#!/usr/bin/env python3
"""
Sensor Bridge Node - Bridges ROS2 topics to JSON files for low-latency reading.

This node subscribes to sensor topics and writes their data to JSON files
on a shared volume. The host Python code can read these files with ~10ms
latency instead of ~400-500ms for ros2 topic echo subprocess calls.

Topics bridged:
- /scan -> /landerpi_data/lidar.json
- /hazards -> /landerpi_data/hazards.json
- /depth_stats -> /landerpi_data/depth_stats.json
- /battery -> /landerpi_data/battery.json

Uses atomic writes (tmp file + rename) to prevent partial reads.
"""

import json
import os
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32


class SensorBridgeNode(Node):
    """Bridges ROS2 sensor topics to JSON files for low-latency host reading."""

    def __init__(self):
        super().__init__('sensor_bridge')

        # Parameters
        self.declare_parameter('data_dir', '/landerpi_data')
        self.declare_parameter('lidar_throttle_hz', 20.0)
        self.declare_parameter('hazard_throttle_hz', 20.0)
        self.declare_parameter('depth_throttle_hz', 10.0)
        self.declare_parameter('battery_throttle_hz', 1.0)

        self.data_dir = Path(self.get_parameter('data_dir').get_parameter_value().string_value)
        self.lidar_min_interval = 1.0 / self.get_parameter('lidar_throttle_hz').get_parameter_value().double_value
        self.hazard_min_interval = 1.0 / self.get_parameter('hazard_throttle_hz').get_parameter_value().double_value
        self.depth_min_interval = 1.0 / self.get_parameter('depth_throttle_hz').get_parameter_value().double_value
        self.battery_min_interval = 1.0 / self.get_parameter('battery_throttle_hz').get_parameter_value().double_value

        # Ensure data directory exists
        self.data_dir.mkdir(parents=True, exist_ok=True)

        # Throttle state
        self._last_lidar_write = 0.0
        self._last_hazard_write = 0.0
        self._last_depth_write = 0.0
        self._last_battery_write = 0.0

        # File paths
        self.lidar_file = self.data_dir / 'lidar.json'
        self.hazards_file = self.data_dir / 'hazards.json'
        self.depth_stats_file = self.data_dir / 'depth_stats.json'
        self.battery_file = self.data_dir / 'battery.json'

        # QoS for sensor data - best effort for lidar (high frequency)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Reliable QoS for less frequent topics
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.create_subscription(String, '/hazards', self.hazards_callback, reliable_qos)
        self.create_subscription(String, '/depth_stats', self.depth_stats_callback, reliable_qos)
        self.create_subscription(Float32, '/battery', self.battery_callback, reliable_qos)

        self.get_logger().info(f'Sensor Bridge initialized. Writing to: {self.data_dir}')

    def _write_json_atomic(self, file_path: Path, data: dict) -> bool:
        """Write JSON atomically using tmp file and rename.

        This prevents partial reads by the host Python code.
        """
        tmp_path = file_path.with_suffix('.tmp')
        try:
            with open(tmp_path, 'w') as f:
                json.dump(data, f)
            os.rename(tmp_path, file_path)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to write {file_path}: {e}')
            return False

    def scan_callback(self, msg: LaserScan):
        """Handle lidar scan messages."""
        now = time.time()
        if now - self._last_lidar_write < self.lidar_min_interval:
            return
        self._last_lidar_write = now

        # Convert ranges to list (from array)
        ranges = list(msg.ranges)

        data = {
            'timestamp': now,
            'ranges': ranges,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
        }
        self._write_json_atomic(self.lidar_file, data)

    def hazards_callback(self, msg: String):
        """Handle hazards messages from obstacle_fusion node."""
        now = time.time()
        if now - self._last_hazard_write < self.hazard_min_interval:
            return
        self._last_hazard_write = now

        try:
            # Parse existing JSON from message
            hazard_data = json.loads(msg.data)
            hazards = hazard_data.get('hazards', [])
        except json.JSONDecodeError:
            hazards = []

        data = {
            'timestamp': now,
            'hazards': hazards,
        }
        self._write_json_atomic(self.hazards_file, data)

    def depth_stats_callback(self, msg: String):
        """Handle depth stats messages from depth_stats node."""
        now = time.time()
        if now - self._last_depth_write < self.depth_min_interval:
            return
        self._last_depth_write = now

        try:
            # Parse existing JSON from message
            stats = json.loads(msg.data)
        except json.JSONDecodeError:
            stats = {}

        data = {
            'timestamp': now,
            'min_depth_m': stats.get('min_depth_m', 0.0),
            'avg_depth_m': stats.get('avg_depth_m', 0.0),
            'valid_percent': stats.get('valid_percent', 0.0),
        }
        self._write_json_atomic(self.depth_stats_file, data)

    def battery_callback(self, msg: Float32):
        """Handle battery voltage messages."""
        now = time.time()
        if now - self._last_battery_write < self.battery_min_interval:
            return
        self._last_battery_write = now

        data = {
            'timestamp': now,
            'voltage': msg.data,
        }
        self._write_json_atomic(self.battery_file, data)


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

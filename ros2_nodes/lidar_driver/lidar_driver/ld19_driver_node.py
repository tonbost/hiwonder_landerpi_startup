#!/usr/bin/env python3
"""
ROS2 driver for LD19/MS200 lidar.

Publishes to: /scan (sensor_msgs/LaserScan)

Reads serial data from /dev/ttyUSB0 or /dev/ttyUSB1 at 230400 baud.
Accumulates 360 degrees of scan data before publishing.
"""

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    import serial
except ImportError:
    serial = None


class LD19Driver(Node):
    """LD19/MS200 lidar driver - accumulates full 360 scans before publishing."""

    PACKET_HEADER = 0x54
    PACKET_VER_LEN = 0x2C  # 12-point packet
    PACKET_SIZE = 47

    def __init__(self):
        super().__init__('lidar_driver')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('port_fallback', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('frame_id', 'laser_frame')

        port = self.get_parameter('port').value
        port_fallback = self.get_parameter('port_fallback').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publisher
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        # Scan accumulator (360 degrees)
        self.ranges = [float('inf')] * 360
        self.running = True

        # Try to open serial port
        if serial is None:
            self.get_logger().error('pyserial not installed')
            self.serial = None
            return

        self.serial = None
        for p in [port, port_fallback]:
            try:
                self.serial = serial.Serial(p, self.baudrate, timeout=1)
                self.get_logger().info(f'Opened {p} at {self.baudrate} baud')
                break
            except Exception as e:
                self.get_logger().warn(f'Failed to open {p}: {e}')

        if self.serial is None:
            self.get_logger().error('Could not open any serial port')
            return

        # Start reading thread
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

        # Publish timer at 10Hz
        self.timer = self.create_timer(0.1, self.publish_scan)

        self.get_logger().info('Lidar driver started')

    def read_loop(self):
        """Read and parse lidar data packets."""
        buffer = bytearray()
        while self.running and self.serial:
            try:
                data = self.serial.read(512)
                if data:
                    buffer.extend(data)
                    self.process_buffer(buffer)
            except Exception:
                time.sleep(0.05)

    def process_buffer(self, buffer):
        """Process buffer for LD19 packets."""
        while len(buffer) >= self.PACKET_SIZE:
            # Find packet header
            try:
                idx = buffer.index(self.PACKET_HEADER)
                if idx > 0:
                    del buffer[:idx]
                    continue
            except ValueError:
                buffer.clear()
                return

            if len(buffer) < self.PACKET_SIZE:
                return

            # Validate packet type
            if buffer[1] != self.PACKET_VER_LEN:
                del buffer[:1]
                continue

            # Parse packet
            try:
                start_angle = struct.unpack('<H', bytes(buffer[4:6]))[0] / 100.0
                end_angle = struct.unpack('<H', bytes(buffer[42:44]))[0] / 100.0

                # Calculate angle step
                if end_angle < start_angle:
                    angle_span = (360.0 - start_angle) + end_angle
                else:
                    angle_span = end_angle - start_angle

                if angle_span <= 0 or angle_span > 60:
                    del buffer[:1]
                    continue

                angle_step = angle_span / 11.0  # 12 points = 11 steps

                # Extract 12 distance measurements
                for i in range(12):
                    offset = 6 + i * 3
                    dist_mm = struct.unpack('<H', bytes(buffer[offset:offset + 2]))[0]

                    angle = (start_angle + i * angle_step) % 360.0
                    angle_idx = int(angle) % 360

                    if 50 < dist_mm < 12000:  # Valid range: 5cm to 12m
                        self.ranges[angle_idx] = dist_mm / 1000.0

                del buffer[:self.PACKET_SIZE]

            except Exception:
                del buffer[:1]

    def publish_scan(self):
        """Publish accumulated scan data."""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = 2.0 * math.pi / 360.0
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.05
        msg.range_max = 12.0
        msg.ranges = list(self.ranges)

        self.publisher.publish(msg)

    def destroy_node(self):
        self.running = False
        if self.serial:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LD19Driver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

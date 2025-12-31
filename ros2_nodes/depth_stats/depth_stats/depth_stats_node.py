#!/usr/bin/env python3
"""
Depth Stats Node - Processes Aurora 930 depth images and publishes simple stats.

Subscribes to /aurora/depth/image_raw (sensor_msgs/Image, mono16)
Publishes to /depth_stats (std_msgs/String as JSON)

This node properly handles mono16 depth images by:
1. Converting raw bytes to uint16 numpy array
2. Extracting center region (configurable)
3. Calculating min/avg/valid_percent for obstacle detection
"""

import json
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class DepthStatsNode(Node):
    """Processes depth images and publishes stats."""

    def __init__(self):
        super().__init__('depth_stats')

        # Parameters
        self.declare_parameter('center_width', 200)  # pixels from center horizontally
        self.declare_parameter('center_height', 160)  # pixels from center vertically
        self.declare_parameter('min_valid_depth', 150)  # mm
        self.declare_parameter('max_valid_depth', 3000)  # mm
        self.declare_parameter('publish_rate', 10.0)  # Hz

        self.center_width = self.get_parameter('center_width').value
        self.center_height = self.get_parameter('center_height').value
        self.min_valid = self.get_parameter('min_valid_depth').value
        self.max_valid = self.get_parameter('max_valid_depth').value

        # Publisher
        self.stats_pub = self.create_publisher(String, '/depth_stats', 10)

        # Subscriber
        self.depth_sub = self.create_subscription(
            Image, '/aurora/depth/image_raw', self.depth_callback, 10
        )

        # Rate limiting
        self.last_publish_time = 0.0
        self.publish_interval = 1.0 / self.get_parameter('publish_rate').value

        # Stats cache for logging
        self.last_stats = None

        self.get_logger().info(
            f'Depth stats node started. '
            f'Center region: {self.center_width}x{self.center_height}, '
            f'Valid range: {self.min_valid}-{self.max_valid}mm'
        )

    def depth_callback(self, msg: Image):
        """Process depth image and publish stats."""
        import time
        now = time.time()

        # Rate limit publishing
        if now - self.last_publish_time < self.publish_interval:
            return
        self.last_publish_time = now

        try:
            # Verify encoding
            if msg.encoding != 'mono16':
                self.get_logger().warn(
                    f'Unexpected encoding: {msg.encoding}, expected mono16'
                )
                return

            # Convert to numpy array
            data = np.frombuffer(bytes(msg.data), dtype=np.uint16)
            data = data.reshape(msg.height, msg.width)

            # Extract center region
            h, w = data.shape
            ch, cw = h // 2, w // 2
            half_h = min(self.center_height // 2, ch)
            half_w = min(self.center_width // 2, cw)

            center = data[ch - half_h:ch + half_h, cw - half_w:cw + half_w]

            # Calculate stats for valid pixels
            valid_mask = (center >= self.min_valid) & (center <= self.max_valid)
            valid_pixels = center[valid_mask]

            stats = {
                'timestamp': now,
                'width': int(msg.width),
                'height': int(msg.height),
                'center_region': f'{half_w*2}x{half_h*2}',
                'total_pixels': int(center.size),
                'valid_count': int(len(valid_pixels)),
                'valid_percent': round(100.0 * len(valid_pixels) / center.size, 1),
            }

            if len(valid_pixels) > 0:
                stats['min_depth_mm'] = int(valid_pixels.min())
                stats['avg_depth_mm'] = int(valid_pixels.mean())
                stats['max_depth_mm'] = int(valid_pixels.max())
                # Also provide meters for convenience
                stats['min_depth_m'] = round(valid_pixels.min() / 1000.0, 3)
                stats['avg_depth_m'] = round(valid_pixels.mean() / 1000.0, 3)
            else:
                stats['min_depth_mm'] = 0
                stats['avg_depth_mm'] = 0
                stats['max_depth_mm'] = 0
                stats['min_depth_m'] = 0.0
                stats['avg_depth_m'] = 0.0

            # Publish
            msg_out = String()
            msg_out.data = json.dumps(stats)
            self.stats_pub.publish(msg_out)

            # Log occasionally
            if self.last_stats is None or now - self.last_stats > 10.0:
                self.get_logger().info(
                    f'Depth: {stats["valid_percent"]:.1f}% valid, '
                    f'min={stats["min_depth_m"]:.2f}m, avg={stats["avg_depth_m"]:.2f}m'
                )
                self.last_stats = now

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthStatsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

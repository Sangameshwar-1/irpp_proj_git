#!/usr/bin/env python3
"""
Scan Debug Node - Monitors LaserScan messages and logs frame_id
This helps diagnose why slam_toolbox might not be receiving scans
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanDebugNode(Node):
    def __init__(self):
        super().__init__('scan_debug_node')
        
        self.scan_count = 0
        self.last_frame = None
        
        # Subscribe to scan topic
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        
        self.get_logger().info("Scan Debug Node started - monitoring /scan")
    
    def scan_callback(self, msg):
        """Monitor and log scan messages"""
        self.scan_count += 1
        
        if self.scan_count == 1 or msg.header.frame_id != self.last_frame:
            self.get_logger().info(
                f"Scan #{self.scan_count}: frame_id='{msg.header.frame_id}', "
                f"ranges={len(msg.ranges)}, range=[{msg.range_min:.2f}-{msg.range_max:.2f}]"
            )
            self.last_frame = msg.header.frame_id
        elif self.scan_count % 100 == 0:
            self.get_logger().info(f"Received {self.scan_count} scans...")


def main(args=None):
    rclpy.init(args=args)
    node = ScanDebugNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

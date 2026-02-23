#!/usr/bin/env python3
"""
Odometry TF Publisher Node
Publishes the odom -> base_footprint transform from odometry messages
Skips stale messages from Gazebo bridge buffer replay
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__("odom_tf_publisher")
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_count = 0
        self.skipped_count = 0
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        
        self.get_logger().info("Odom TF Publisher started")
    
    def odom_callback(self, msg):
        """Publish odom -> base_footprint transform, skipping stale messages"""
        try:
            now = self.get_clock().now()
            # Don't publish if sim clock not yet initialized
            if now.nanoseconds == 0:
                return
            msg_time = Time.from_msg(msg.header.stamp)
            age_sec = (now - msg_time).nanoseconds / 1e9
            if age_sec > 2.0:  # Skip messages older than 2 seconds
                self.skipped_count += 1
                if self.skipped_count == 1:
                    self.get_logger().warn(
                        f"Skipping stale odom (age={age_sec:.2f}s). Waiting for fresh data..."
                    )
                elif self.skipped_count % 200 == 0:
                    self.get_logger().warn(
                        f"Still skipping stale odom (count={self.skipped_count}, age={age_sec:.2f}s)"
                    )
                return
            if self.skipped_count > 0:
                self.get_logger().info(
                    f"Now receiving fresh odom (skipped {self.skipped_count} stale msgs)"
                )
                self.skipped_count = 0
        except Exception as e:
            # Clock not ready yet — skip to avoid publishing with wrong timestamp
            return
        
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
        
        self.tf_count += 1
        if self.tf_count == 1:
            self.get_logger().info(
                f"First VALID odom: x={msg.pose.pose.position.x:.3f}, "
                f"y={msg.pose.pose.position.y:.3f}"
            )
        elif self.tf_count % 200 == 0:
            self.get_logger().info(f"Odom TF publishing OK ({self.tf_count} transforms)")


def main():
    rclpy.init()
    node = OdomTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Camera View 360 Node
Combines 4 camera views into a single 360-degree panoramic view
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

try:
    from cv_bridge import CvBridge
    import cv2
    HAS_CV = True
except ImportError:
    HAS_CV = False


class CameraView360(Node):
    def __init__(self):
        super().__init__("camera_view_360")
        
        if not HAS_CV:
            self.get_logger().warn("OpenCV/cv_bridge not available. Camera view disabled.")
            return
        
        self.bridge = CvBridge()
        
        # Camera images storage
        self.front_img = None
        self.right_img = None
        self.back_img = None
        self.left_img = None
        
        # Publisher for combined view
        self.panorama_pub = self.create_publisher(Image, "/camera/panorama", 10)
        
        # Subscribers for each camera
        self.front_sub = self.create_subscription(
            Image, "/camera/front/image", self.front_callback, 10
        )
        self.right_sub = self.create_subscription(
            Image, "/camera/right/image", self.right_callback, 10
        )
        self.back_sub = self.create_subscription(
            Image, "/camera/back/image", self.back_callback, 10
        )
        self.left_sub = self.create_subscription(
            Image, "/camera/left/image", self.left_callback, 10
        )
        
        # Timer to publish combined view
        self.timer = self.create_timer(0.2, self.publish_panorama)
        
        self.get_logger().info("360 Camera View node started")
    
    def front_callback(self, msg):
        try:
            self.front_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info("Front camera received", throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f"Front camera error: {e}")
    
    def right_callback(self, msg):
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Right camera error: {e}")
    
    def back_callback(self, msg):
        try:
            self.back_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Back camera error: {e}")
    
    def left_callback(self, msg):
        try:
            self.left_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Left camera error: {e}")
    
    def publish_panorama(self):
        if not HAS_CV:
            return
            
        # Create placeholder images if cameras not ready
        h, w = 240, 320  # Reduced resolution for panorama
        
        def resize_or_placeholder(img, label):
            if img is not None:
                try:
                    return cv2.resize(img, (w, h))
                except:
                    pass
            # Create placeholder with label
            placeholder = np.zeros((h, w, 3), dtype=np.uint8)
            placeholder[:, :] = [50, 50, 50]  # Dark gray
            cv2.putText(placeholder, label, (w//3, h//2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            return placeholder
        
        front = resize_or_placeholder(self.front_img, "FRONT")
        right = resize_or_placeholder(self.right_img, "RIGHT")
        back = resize_or_placeholder(self.back_img, "BACK")
        left = resize_or_placeholder(self.left_img, "LEFT")
        
        # Create 2x2 grid: [Front, Right]
        #                  [Left,  Back]
        top_row = np.hstack([front, right])
        bottom_row = np.hstack([left, back])
        panorama = np.vstack([top_row, bottom_row])
        
        # Add labels
        cv2.putText(panorama, "FRONT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(panorama, "RIGHT", (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(panorama, "LEFT", (10, h + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(panorama, "BACK", (w + 10, h + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Publish
        try:
            panorama_msg = self.bridge.cv2_to_imgmsg(panorama, "bgr8")
            panorama_msg.header.stamp = self.get_clock().now().to_msg()
            panorama_msg.header.frame_id = "camera_link"
            self.panorama_pub.publish(panorama_msg)
        except Exception as e:
            self.get_logger().error(f"Panorama publish error: {e}")


def main():
    rclpy.init()
    node = CameraView360()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

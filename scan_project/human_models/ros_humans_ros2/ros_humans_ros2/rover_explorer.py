#!/usr/bin/env python3
"""
Autonomous Rover Explorer Node
Makes the TurtleBot navigate autonomously while avoiding obstacles
Only moves in valid regions within the room bounds
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import random


class RoverExplorer(Node):
    def __init__(self):
        super().__init__("rover_explorer")
        
        # Parameters - Room is 25m x 25m, walls at ±12.5
        self.declare_parameter("linear_speed", 0.25)
        self.declare_parameter("angular_speed", 0.4)
        self.declare_parameter("obstacle_distance", 1.0)
        self.declare_parameter("room_min_x", -10.0)
        self.declare_parameter("room_max_x", 10.0)
        self.declare_parameter("room_min_y", -10.0)
        self.declare_parameter("room_max_y", 10.0)
        self.declare_parameter("wall_margin", 1.0)
        
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.obstacle_distance = self.get_parameter("obstacle_distance").value
        self.room_min_x = self.get_parameter("room_min_x").value
        self.room_max_x = self.get_parameter("room_max_x").value
        self.room_min_y = self.get_parameter("room_min_y").value
        self.room_max_y = self.get_parameter("room_max_y").value
        self.wall_margin = self.get_parameter("wall_margin").value
        
        # State variables
        self.scan_data = None
        self.current_x = 0.0
        self.current_y = -8.0  # Start position (center-south)
        self.current_yaw = 1.5708  # Facing north
        self.state = "EXPLORE"
        self.turn_start_time = None
        self.turn_duration = 0.0
        self.backup_start_time = None
        
        # Exploration waypoints for full room coverage (valid regions only)
        self.exploration_targets = [
            (0.0, 0.0),      # Center
            (8.0, 8.0),      # NE
            (-8.0, 8.0),     # NW
            (-8.0, -8.0),    # SW
            (8.0, -8.0),     # SE
            (0.0, 8.0),      # N
            (0.0, -8.0),     # S
            (8.0, 0.0),      # E
            (-8.0, 0.0),     # W
            (4.0, 4.0),      # Inner NE
            (-4.0, 4.0),     # Inner NW
            (-4.0, -4.0),    # Inner SW
            (4.0, -4.0),     # Inner SE
        ]
        self.current_target_idx = 0
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/model/turtlebot3_rover/cmd_vel", 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/model/turtlebot3_rover/odometry", self.odom_callback, 10
        )
        
        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Rover Explorer started - autonomous navigation in valid regions only")
    
    def scan_callback(self, msg):
        self.scan_data = msg
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def is_in_valid_region(self, x, y):
        """Check if position is within valid movable region"""
        margin = self.wall_margin
        return (self.room_min_x + margin < x < self.room_max_x - margin and
                self.room_min_y + margin < y < self.room_max_y - margin)
    
    def get_sector_min(self, ranges, start_angle, end_angle, angle_min, angle_increment):
        """Get minimum distance in a sector defined by angles (radians)"""
        if not ranges:
            return float('inf')
        
        valid_ranges = []
        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            
            if start_angle <= angle <= end_angle:
                if 0.1 < r < 12.0 and not math.isinf(r) and not math.isnan(r):
                    valid_ranges.append(r)
        
        return min(valid_ranges) if valid_ranges else float('inf')
    
    def get_direction_to_target(self):
        """Calculate angle to current exploration target"""
        target = self.exploration_targets[self.current_target_idx]
        dx = target[0] - self.current_x
        dy = target[1] - self.current_y
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        return angle_diff
    
    def distance_to_target(self):
        target = self.exploration_targets[self.current_target_idx]
        return math.hypot(target[0] - self.current_x, target[1] - self.current_y)
    
    def control_loop(self):
        cmd = Twist()
        
        if self.scan_data is None:
            self.cmd_vel_pub.publish(cmd)
            return
        
        ranges = list(self.scan_data.ranges)
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment
        
        # Calculate distances in different sectors
        front_min = self.get_sector_min(ranges, -0.4, 0.4, angle_min, angle_increment)
        front_left_min = self.get_sector_min(ranges, 0.4, 1.0, angle_min, angle_increment)
        front_right_min = self.get_sector_min(ranges, -1.0, -0.4, angle_min, angle_increment)
        left_min = self.get_sector_min(ranges, 1.0, 2.0, angle_min, angle_increment)
        right_min = self.get_sector_min(ranges, -2.0, -1.0, angle_min, angle_increment)
        
        now = self.get_clock().now()
        
        # Check if near room boundaries (MUST stay in valid region)
        near_boundary = not self.is_in_valid_region(self.current_x, self.current_y)
        
        if self.state == "EXPLORE":
            if front_min < self.obstacle_distance or near_boundary:
                # Obstacle or boundary - must turn
                if near_boundary:
                    # Near wall - turn toward room center
                    center_angle = math.atan2(-self.current_y, -self.current_x)
                    angle_diff = center_angle - self.current_yaw
                    while angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    while angle_diff < -math.pi:
                        angle_diff += 2 * math.pi
                    
                    self.state = "TURN_LEFT" if angle_diff > 0 else "TURN_RIGHT"
                    self.get_logger().info(f"Near boundary at ({self.current_x:.1f}, {self.current_y:.1f})! Turning to center")
                elif left_min > right_min:
                    self.state = "TURN_LEFT"
                else:
                    self.state = "TURN_RIGHT"
                
                self.turn_start_time = now
                self.turn_duration = random.uniform(0.8, 2.0)
                
            elif front_left_min < self.obstacle_distance * 0.6:
                cmd.linear.x = self.linear_speed * 0.5
                cmd.angular.z = -self.angular_speed * 0.4
                
            elif front_right_min < self.obstacle_distance * 0.6:
                cmd.linear.x = self.linear_speed * 0.5
                cmd.angular.z = self.angular_speed * 0.4
                
            else:
                # Clear ahead - move toward exploration target
                dist = self.distance_to_target()
                if dist < 2.0:
                    self.current_target_idx = (self.current_target_idx + 1) % len(self.exploration_targets)
                    self.get_logger().info(f"Reached target! Next: {self.exploration_targets[self.current_target_idx]}")
                
                angle_to_target = self.get_direction_to_target()
                cmd.linear.x = self.linear_speed
                cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, angle_to_target * 0.5))
        
        elif self.state == "TURN_LEFT":
            elapsed = (now - self.turn_start_time).nanoseconds / 1e9
            if elapsed < self.turn_duration and front_min < self.obstacle_distance * 1.5:
                cmd.angular.z = self.angular_speed
            else:
                self.state = "EXPLORE"
        
        elif self.state == "TURN_RIGHT":
            elapsed = (now - self.turn_start_time).nanoseconds / 1e9
            if elapsed < self.turn_duration and front_min < self.obstacle_distance * 1.5:
                cmd.angular.z = -self.angular_speed
            else:
                self.state = "EXPLORE"
        
        elif self.state == "BACKUP":
            elapsed = (now - self.backup_start_time).nanoseconds / 1e9
            if elapsed < 1.5:
                cmd.linear.x = -self.linear_speed * 0.5
            else:
                self.state = "TURN_LEFT"
                self.turn_start_time = now
                self.turn_duration = random.uniform(1.5, 3.0)
        
        self.cmd_vel_pub.publish(cmd)


def main():
    rclpy.init()
    node = RoverExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

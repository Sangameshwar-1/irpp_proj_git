#!/usr/bin/env python3
"""
Autonomous Explorer Node for SLAM Mapping
Automatically drives the robot around to build a complete map
Uses frontier-based exploration with wall-following fallback
"""

import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np


class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__("autonomous_explorer")
        
        # Parameters
        self.declare_parameter("linear_speed", 0.25)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("obstacle_distance", 0.5)
        self.declare_parameter("wall_follow_distance", 0.6)
        self.declare_parameter("exploration_time", 300.0)  # 5 minutes default
        self.declare_parameter("save_map_on_complete", True)
        self.declare_parameter("map_name", "auto_map")
        
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.obstacle_distance = self.get_parameter("obstacle_distance").value
        self.wall_follow_distance = self.get_parameter("wall_follow_distance").value
        self.exploration_time = self.get_parameter("exploration_time").value
        self.save_map_on_complete = self.get_parameter("save_map_on_complete").value
        self.map_name = self.get_parameter("map_name").value
        
        # State
        self.scan_data = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.map_data = None
        self.start_time = None
        self.state = "INITIALIZING"  # INITIALIZING, EXPLORING, WALL_FOLLOW, TURNING, DONE
        self.turn_direction = 1  # 1 = left, -1 = right
        self.turn_start_yaw = 0.0
        self.turn_target = 0.0
        self.visited_cells = set()
        self.stuck_counter = 0
        self.last_x = 0.0
        self.last_y = 0.0
        self.exploration_complete = False
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        
        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)
        self.progress_timer = self.create_timer(10.0, self.report_progress)
        self.stuck_check_timer = self.create_timer(5.0, self.check_if_stuck)
        
        self.get_logger().info("Autonomous Explorer started!")
        self.get_logger().info(f"Exploration time: {self.exploration_time} seconds")
        self.get_logger().info("Waiting for sensor data...")
    
    def scan_callback(self, msg):
        self.scan_data = msg
        if self.state == "INITIALIZING":
            self.state = "EXPLORING"
            self.start_time = self.get_clock().now()
            self.get_logger().info("Scan data received. Starting exploration!")
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Track visited positions (discretized to 0.5m cells)
        cell = (int(self.current_x * 2), int(self.current_y * 2))
        self.visited_cells.add(cell)
    
    def map_callback(self, msg):
        self.map_data = msg
    
    def check_if_stuck(self):
        """Check if robot is stuck and not making progress"""
        if self.state in ["INITIALIZING", "DONE"]:
            return
            
        dist = math.sqrt((self.current_x - self.last_x)**2 + 
                        (self.current_y - self.last_y)**2)
        
        if dist < 0.1:  # Moved less than 10cm in 5 seconds
            self.stuck_counter += 1
            if self.stuck_counter >= 3:
                self.get_logger().warn("Robot appears stuck! Attempting recovery...")
                self.state = "TURNING"
                self.turn_direction = random.choice([-1, 1])
                self.turn_start_yaw = self.current_yaw
                self.turn_target = random.uniform(1.5, 2.5)  # Turn 90-150 degrees
                self.stuck_counter = 0
        else:
            self.stuck_counter = 0
        
        self.last_x = self.current_x
        self.last_y = self.current_y
    
    def report_progress(self):
        """Report exploration progress"""
        if self.state == "DONE" or self.start_time is None:
            return
            
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        remaining = max(0, self.exploration_time - elapsed)
        
        map_coverage = "N/A"
        if self.map_data is not None:
            data = np.array(self.map_data.data)
            known = np.sum(data >= 0)
            total = len(data)
            map_coverage = f"{100*known/total:.1f}%"
        
        self.get_logger().info(
            f"Progress: {elapsed:.0f}s elapsed, {remaining:.0f}s remaining, "
            f"visited {len(self.visited_cells)} cells, map coverage: {map_coverage}"
        )
        
        # Check if exploration time is complete
        if elapsed >= self.exploration_time:
            self.complete_exploration()
    
    def complete_exploration(self):
        """Complete the exploration and optionally save the map"""
        if self.exploration_complete:
            return
            
        self.exploration_complete = True
        self.state = "DONE"
        
        # Stop the robot
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("EXPLORATION COMPLETE!")
        self.get_logger().info(f"Visited {len(self.visited_cells)} unique cells")
        
        if self.save_map_on_complete:
            self.get_logger().info(f"Saving map as: ~/maps/{self.map_name}")
            self.get_logger().info("Run this command to save the map:")
            self.get_logger().info(f"  ros2 run nav2_map_server map_saver_cli -f ~/maps/{self.map_name}")
        
        self.get_logger().info("=" * 50)
    
    def get_scan_ranges(self):
        """Get scan ranges divided into regions"""
        if self.scan_data is None:
            return None, None, None, None, None
        
        ranges = list(self.scan_data.ranges)
        n = len(ranges)
        
        # Divide scan into 5 regions (front, front-left, left, front-right, right)
        # Assuming 360 degree scan
        segment = n // 8
        
        # Front: -22.5 to +22.5 degrees
        front = ranges[-segment:] + ranges[:segment]
        
        # Front-left: +22.5 to +67.5 degrees
        front_left = ranges[segment:2*segment]
        
        # Left: +67.5 to +112.5 degrees
        left = ranges[2*segment:3*segment]
        
        # Front-right: -67.5 to -22.5 degrees
        front_right = ranges[-2*segment:-segment]
        
        # Right: -112.5 to -67.5 degrees
        right = ranges[-3*segment:-2*segment]
        
        def safe_min(data):
            valid = [r for r in data if self.scan_data.range_min < r < self.scan_data.range_max]
            return min(valid) if valid else float('inf')
        
        return safe_min(front), safe_min(front_left), safe_min(left), safe_min(front_right), safe_min(right)
    
    def control_loop(self):
        """Main control loop"""
        if self.state == "INITIALIZING":
            return
        
        if self.state == "DONE":
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
        
        if self.scan_data is None:
            return
        
        front, front_left, left, front_right, right = self.get_scan_ranges()
        
        if front is None:
            return
        
        cmd = Twist()
        
        if self.state == "TURNING":
            # Execute turn
            yaw_diff = abs(self.current_yaw - self.turn_start_yaw)
            if yaw_diff > math.pi:
                yaw_diff = 2 * math.pi - yaw_diff
            
            if yaw_diff >= self.turn_target:
                self.state = "EXPLORING"
            else:
                cmd.angular.z = self.angular_speed * self.turn_direction
        
        elif self.state == "WALL_FOLLOW":
            # Wall following behavior
            if front < self.obstacle_distance:
                # Obstacle in front, turn away
                cmd.angular.z = self.angular_speed * self.turn_direction
                cmd.linear.x = 0.05
            elif right < self.wall_follow_distance * 0.5:
                # Too close to wall, turn left
                cmd.angular.z = self.angular_speed * 0.5
                cmd.linear.x = self.linear_speed * 0.5
            elif right > self.wall_follow_distance * 1.5:
                # Too far from wall, turn right
                cmd.angular.z = -self.angular_speed * 0.3
                cmd.linear.x = self.linear_speed
            else:
                # Good distance, go straight
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0
            
            # Switch to exploration if no wall nearby
            if right > 2.0 and front > 1.5:
                self.state = "EXPLORING"
        
        else:  # EXPLORING
            if front < self.obstacle_distance:
                # Obstacle ahead - decide which way to turn
                if front_left > front_right:
                    self.turn_direction = 1  # Turn left
                else:
                    self.turn_direction = -1  # Turn right
                
                # Check if we should switch to wall following
                if right < self.wall_follow_distance * 2:
                    self.state = "WALL_FOLLOW"
                else:
                    self.state = "TURNING"
                    self.turn_start_yaw = self.current_yaw
                    self.turn_target = random.uniform(0.8, 1.5)
                
                cmd.angular.z = self.angular_speed * self.turn_direction
                cmd.linear.x = 0.0
            
            elif front_right < self.obstacle_distance * 0.8:
                # Obstacle on front-right, turn slightly left
                cmd.angular.z = self.angular_speed * 0.3
                cmd.linear.x = self.linear_speed * 0.7
            
            elif front_left < self.obstacle_distance * 0.8:
                # Obstacle on front-left, turn slightly right
                cmd.angular.z = -self.angular_speed * 0.3
                cmd.linear.x = self.linear_speed * 0.7
            
            else:
                # Clear ahead - go forward with slight random turns for coverage
                cmd.linear.x = self.linear_speed
                
                # Add small random turns for better coverage
                if random.random() < 0.05:
                    cmd.angular.z = random.uniform(-0.2, 0.2)
                
                # Prefer turning toward unexplored areas (simple heuristic)
                if left > right * 1.5:
                    cmd.angular.z += 0.1
                elif right > left * 1.5:
                    cmd.angular.z -= 0.1
        
        self.cmd_vel_pub.publish(cmd)


def main():
    rclpy.init()
    node = AutonomousExplorer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exploration interrupted by user")
    finally:
        # Stop the robot
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

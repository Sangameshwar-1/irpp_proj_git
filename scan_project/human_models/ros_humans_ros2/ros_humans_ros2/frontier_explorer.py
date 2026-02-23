#!/usr/bin/env python3
"""
Frontier-Based Explorer Node for SLAM Mapping
Uses proper frontier detection to systematically explore unknown areas
and build a complete occupancy grid map.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import tf2_ros
from tf2_ros import Buffer, TransformListener


class FrontierExplorer(Node):
    """
    Frontier-based exploration node that:
    1. Detects frontiers (boundaries between known and unknown space)
    2. Selects the best frontier to explore
    3. Navigates to the frontier using obstacle avoidance
    4. Repeats until no frontiers remain
    """
    
    def __init__(self):
        super().__init__("frontier_explorer")
        
        # Parameters
        self.declare_parameter("linear_speed", 0.22)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("obstacle_threshold", 0.4)
        self.declare_parameter("frontier_threshold", 0.3)
        self.declare_parameter("exploration_timeout", 300.0)
        self.declare_parameter("goal_tolerance", 0.5)
        self.declare_parameter("min_frontier_size", 5)
        
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.obstacle_threshold = self.get_parameter("obstacle_threshold").value
        self.frontier_threshold = self.get_parameter("frontier_threshold").value
        self.exploration_timeout = self.get_parameter("exploration_timeout").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.min_frontier_size = self.get_parameter("min_frontier_size").value
        
        # State variables
        self.map_data = None
        self.map_info = None
        self.scan_data = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.current_goal = None
        self.frontiers = []
        self.exploration_complete = False
        self.start_time = None
        self.stuck_counter = 0
        self.last_position = (0.0, 0.0)
        
        # State machine
        self.state = "WAITING"  # WAITING, FINDING_FRONTIER, NAVIGATING, AVOIDING, DONE
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # QoS for map topic (transient local)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.frontier_markers_pub = self.create_publisher(
            MarkerArray, "/frontier_markers", 10
        )
        self.goal_marker_pub = self.create_publisher(Marker, "/exploration_goal", 10)
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, map_qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.frontier_timer = self.create_timer(2.0, self.find_frontiers)
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("Frontier Explorer Started!")
        self.get_logger().info(f"Exploration timeout: {self.exploration_timeout}s")
        self.get_logger().info("Waiting for map and sensor data...")
        self.get_logger().info("=" * 50)
    
    def map_callback(self, msg):
        """Store map data for frontier detection"""
        self.map_data = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width)
        )
        self.map_info = msg.info
        
        if self.state == "WAITING":
            self.get_logger().info("Map received! Starting exploration...")
            self.state = "FINDING_FRONTIER"
            self.start_time = self.get_clock().now()
    
    def scan_callback(self, msg):
        """Store scan data for obstacle avoidance"""
        self.scan_data = msg
    
    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def find_frontiers(self):
        """Detect frontier cells in the occupancy grid"""
        if self.map_data is None or self.state == "DONE":
            return
        
        frontiers = []
        height, width = self.map_data.shape
        
        # Find frontier cells (free cells adjacent to unknown cells)
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                # Check if current cell is free (0)
                if self.map_data[y, x] == 0:
                    # Check if any neighbor is unknown (-1)
                    neighbors = [
                        self.map_data[y-1, x], self.map_data[y+1, x],
                        self.map_data[y, x-1], self.map_data[y, x+1],
                        self.map_data[y-1, x-1], self.map_data[y-1, x+1],
                        self.map_data[y+1, x-1], self.map_data[y+1, x+1],
                    ]
                    if -1 in neighbors:
                        # Convert to world coordinates
                        wx = self.map_info.origin.position.x + (x + 0.5) * self.map_info.resolution
                        wy = self.map_info.origin.position.y + (y + 0.5) * self.map_info.resolution
                        frontiers.append((wx, wy))
        
        # Cluster frontiers
        self.frontiers = self.cluster_frontiers(frontiers)
        
        # Publish frontier markers for visualization
        self.publish_frontier_markers()
        
        if len(self.frontiers) == 0 and self.state != "WAITING":
            self.get_logger().info("No frontiers found! Exploration complete!")
            self.exploration_complete = True
            self.state = "DONE"
    
    def cluster_frontiers(self, frontier_cells):
        """Cluster frontier cells into frontier regions and return centroids"""
        if not frontier_cells:
            return []
        
        # Simple clustering by proximity
        clusters = []
        used = set()
        cluster_distance = 0.3  # 30cm
        
        for i, (x1, y1) in enumerate(frontier_cells):
            if i in used:
                continue
            
            cluster = [(x1, y1)]
            used.add(i)
            
            for j, (x2, y2) in enumerate(frontier_cells):
                if j in used:
                    continue
                
                # Check if close to any point in cluster
                for cx, cy in cluster:
                    dist = math.sqrt((x2 - cx)**2 + (y2 - cy)**2)
                    if dist < cluster_distance:
                        cluster.append((x2, y2))
                        used.add(j)
                        break
            
            # Only keep clusters larger than minimum size
            if len(cluster) >= self.min_frontier_size:
                # Calculate centroid
                cx = sum(p[0] for p in cluster) / len(cluster)
                cy = sum(p[1] for p in cluster) / len(cluster)
                clusters.append((cx, cy, len(cluster)))
        
        return clusters
    
    def select_best_frontier(self):
        """Select the best frontier to explore based on distance and size"""
        if not self.frontiers:
            return None
        
        best_frontier = None
        best_score = float('inf')
        
        for fx, fy, size in self.frontiers:
            # Distance to frontier
            dist = math.sqrt((fx - self.robot_x)**2 + (fy - self.robot_y)**2)
            
            # Skip frontiers that are too close (already explored)
            if dist < 0.5:
                continue
            
            # Score: prefer closer and larger frontiers
            # Lower score is better
            score = dist / (math.log(size + 1) + 1)
            
            if score < best_score:
                best_score = score
                best_frontier = (fx, fy)
        
        return best_frontier
    
    def control_loop(self):
        """Main control loop for exploration"""
        if self.state == "WAITING" or self.state == "DONE":
            self.stop_robot()
            return
        
        # Check timeout
        if self.start_time is not None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > self.exploration_timeout:
                self.get_logger().info(f"Exploration timeout ({self.exploration_timeout}s) reached!")
                self.state = "DONE"
                self.stop_robot()
                return
        
        # Check if stuck
        dist_moved = math.sqrt(
            (self.robot_x - self.last_position[0])**2 + 
            (self.robot_y - self.last_position[1])**2
        )
        if dist_moved < 0.01:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
            self.last_position = (self.robot_x, self.robot_y)
        
        # State machine
        if self.state == "FINDING_FRONTIER":
            self.current_goal = self.select_best_frontier()
            if self.current_goal:
                self.get_logger().info(
                    f"New goal: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})"
                )
                self.publish_goal_marker()
                self.state = "NAVIGATING"
            else:
                # No valid frontier, try rotating to discover more
                self.rotate_in_place()
        
        elif self.state == "NAVIGATING":
            if self.current_goal is None:
                self.state = "FINDING_FRONTIER"
                return
            
            # Check if goal reached
            dist_to_goal = math.sqrt(
                (self.current_goal[0] - self.robot_x)**2 + 
                (self.current_goal[1] - self.robot_y)**2
            )
            
            if dist_to_goal < self.goal_tolerance:
                self.get_logger().info("Goal reached! Finding next frontier...")
                self.current_goal = None
                self.state = "FINDING_FRONTIER"
                return
            
            # Check for obstacles
            if self.check_obstacle_ahead():
                self.state = "AVOIDING"
                return
            
            # Check if stuck
            if self.stuck_counter > 30:  # 3 seconds stuck
                self.get_logger().warn("Robot stuck! Recovering...")
                self.recover_from_stuck()
                return
            
            # Navigate to goal
            self.navigate_to_goal()
        
        elif self.state == "AVOIDING":
            if not self.check_obstacle_ahead():
                self.state = "NAVIGATING"
                return
            self.avoid_obstacle()
    
    def check_obstacle_ahead(self):
        """Check if there's an obstacle in the path"""
        if self.scan_data is None:
            return False
        
        ranges = np.array(self.scan_data.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, 10.0)
        
        # Check front arc (roughly -30 to +30 degrees)
        n_ranges = len(ranges)
        front_start = int(n_ranges * 0.4)  # -30 deg
        front_end = int(n_ranges * 0.6)    # +30 deg
        
        front_ranges = ranges[front_start:front_end]
        min_front = np.min(front_ranges) if len(front_ranges) > 0 else 10.0
        
        return min_front < self.obstacle_threshold
    
    def navigate_to_goal(self):
        """Navigate toward the current goal"""
        if self.current_goal is None:
            return
        
        # Calculate angle to goal
        dx = self.current_goal[0] - self.robot_x
        dy = self.current_goal[1] - self.robot_y
        target_yaw = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = target_yaw - self.robot_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        cmd = Twist()
        
        # If angle is large, rotate first
        if abs(angle_diff) > 0.3:
            cmd.angular.z = self.angular_speed * (1.0 if angle_diff > 0 else -1.0)
            cmd.linear.x = 0.05  # Slow forward while turning
        else:
            # Move forward with proportional angular correction
            cmd.linear.x = self.linear_speed
            cmd.angular.z = angle_diff * 1.5  # Proportional steering
        
        self.cmd_vel_pub.publish(cmd)
    
    def avoid_obstacle(self):
        """Avoid obstacle using reactive control"""
        if self.scan_data is None:
            return
        
        ranges = np.array(self.scan_data.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, 10.0)
        
        n_ranges = len(ranges)
        left_ranges = ranges[:n_ranges//3]
        right_ranges = ranges[2*n_ranges//3:]
        
        left_min = np.min(left_ranges) if len(left_ranges) > 0 else 10.0
        right_min = np.min(right_ranges) if len(right_ranges) > 0 else 10.0
        
        cmd = Twist()
        cmd.linear.x = 0.0
        
        # Turn toward the side with more space
        if left_min > right_min:
            cmd.angular.z = self.angular_speed
        else:
            cmd.angular.z = -self.angular_speed
        
        self.cmd_vel_pub.publish(cmd)
    
    def rotate_in_place(self):
        """Rotate in place to discover new areas"""
        cmd = Twist()
        cmd.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(cmd)
    
    def recover_from_stuck(self):
        """Recover when robot is stuck"""
        cmd = Twist()
        cmd.linear.x = -0.1  # Back up
        cmd.angular.z = self.angular_speed * (1.0 if np.random.random() > 0.5 else -1.0)
        self.cmd_vel_pub.publish(cmd)
        self.stuck_counter = 0
        self.current_goal = None
        self.state = "FINDING_FRONTIER"
    
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def publish_frontier_markers(self):
        """Publish frontier visualization markers"""
        marker_array = MarkerArray()
        
        # Clear old markers
        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        for i, (fx, fy, size) in enumerate(self.frontiers):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "frontiers"
            marker.id = i + 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = fx
            marker.pose.position.y = fy
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)
        
        self.frontier_markers_pub.publish(marker_array)
    
    def publish_goal_marker(self):
        """Publish current goal marker"""
        if self.current_goal is None:
            return
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = self.current_goal[0]
        marker.pose.position.y = self.current_goal[1]
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.goal_marker_pub.publish(marker)


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

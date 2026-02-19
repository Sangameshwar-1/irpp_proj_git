#!/usr/bin/env python3
"""
A* Path Planner Node for TurtleBot Rover
Implements A* algorithm for finding the shortest path between two points
Uses occupancy grid from LiDAR data for obstacle avoidance
"""

import math
import heapq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np


class AStarPathPlanner(Node):
    def __init__(self):
        super().__init__("astar_path_planner")
        
        # Parameters
        self.declare_parameter("linear_speed", 0.3)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("goal_tolerance", 0.3)
        self.declare_parameter("grid_resolution", 0.2)  # meters per cell
        self.declare_parameter("grid_size", 100)  # cells (50m x 50m area)
        self.declare_parameter("obstacle_inflation", 0.4)  # meters
        self.declare_parameter("room_min_x", -10.0)
        self.declare_parameter("room_max_x", 10.0)
        self.declare_parameter("room_min_y", -10.0)
        self.declare_parameter("room_max_y", 10.0)
        self.declare_parameter("auto_start", True)  # Auto start navigation
        self.declare_parameter("default_goal_x", 5.0)
        self.declare_parameter("default_goal_y", 5.0)
        
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.grid_resolution = self.get_parameter("grid_resolution").value
        self.grid_size = self.get_parameter("grid_size").value
        self.obstacle_inflation = self.get_parameter("obstacle_inflation").value
        self.room_min_x = self.get_parameter("room_min_x").value
        self.room_max_x = self.get_parameter("room_max_x").value
        self.room_min_y = self.get_parameter("room_min_y").value
        self.room_max_y = self.get_parameter("room_max_y").value
        self.auto_start = self.get_parameter("auto_start").value
        self.default_goal_x = self.get_parameter("default_goal_x").value
        self.default_goal_y = self.get_parameter("default_goal_y").value
        
        # State variables
        self.current_x = 0.0
        self.current_y = -8.0
        self.current_yaw = 1.5708
        self.goal_x = None
        self.goal_y = None
        self.path = []
        self.current_path_idx = 0
        self.scan_data = None
        self.state = "WAITING"  # WAITING, IDLE, PLANNING, FOLLOWING, REACHED
        self.odom_received = False
        self.scan_received = False
        self.init_timer_count = 0
        
        # Occupancy grid for path planning
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)
        self.grid_origin_x = -self.grid_size * self.grid_resolution / 2
        self.grid_origin_y = -self.grid_size * self.grid_resolution / 2
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/path_markers", 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/local_costmap", 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )
        
        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)
        self.grid_timer = self.create_timer(1.0, self.publish_grid)
        
        self.get_logger().info("A* Path Planner started!")
        self.get_logger().info("Waiting for odometry and scan data...")
        if self.auto_start:
            self.get_logger().info(f"Auto-start enabled. Will navigate to ({self.default_goal_x}, {self.default_goal_y}) once ready.")
        else:
            self.get_logger().info("Publish goal to /goal_pose to set destination")
    
    def scan_callback(self, msg):
        """Process LiDAR scan and update occupancy grid"""
        if not self.scan_received:
            self.get_logger().info("Scan data received!")
            self.scan_received = True
        self.scan_data = msg
        self.update_occupancy_grid(msg)
    
    def odom_callback(self, msg):
        """Update robot position from odometry"""
        if not self.odom_received:
            self.get_logger().info(f"Odometry received! Position: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")
            self.odom_received = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def goal_callback(self, msg):
        """Handle new goal request"""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        
        # Clamp goal to valid region
        self.goal_x = max(self.room_min_x + 1, min(self.room_max_x - 1, self.goal_x))
        self.goal_y = max(self.room_min_y + 1, min(self.room_max_y - 1, self.goal_y))
        
        self.get_logger().info(f"New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})")
        
        # Only plan if we have sensor data
        if self.odom_received and self.scan_received:
            self.state = "PLANNING"
            self.plan_path()
        else:
            self.get_logger().warn("Waiting for sensor data before planning...")
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices"""
        gx = int((x - self.grid_origin_x) / self.grid_resolution)
        gy = int((y - self.grid_origin_y) / self.grid_resolution)
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        """Convert grid indices to world coordinates"""
        x = gx * self.grid_resolution + self.grid_origin_x + self.grid_resolution / 2
        y = gy * self.grid_resolution + self.grid_origin_y + self.grid_resolution / 2
        return x, y
    
    def is_valid_cell(self, gx, gy):
        """Check if grid cell is within bounds"""
        return 0 <= gx < self.grid_size and 0 <= gy < self.grid_size
    
    def update_occupancy_grid(self, scan):
        """Update occupancy grid from laser scan"""
        if scan is None:
            return
        
        # Decay old obstacles slightly
        self.occupancy_grid = np.clip(self.occupancy_grid - 1, 0, 100)
        
        angle = scan.angle_min
        for i, r in enumerate(scan.ranges):
            if scan.range_min < r < scan.range_max:
                # Calculate obstacle position in world frame
                obs_x = self.current_x + r * math.cos(self.current_yaw + angle)
                obs_y = self.current_y + r * math.sin(self.current_yaw + angle)
                
                # Mark obstacle in grid with inflation
                gx, gy = self.world_to_grid(obs_x, obs_y)
                inflation_cells = int(self.obstacle_inflation / self.grid_resolution)
                
                for dx in range(-inflation_cells, inflation_cells + 1):
                    for dy in range(-inflation_cells, inflation_cells + 1):
                        ngx, ngy = gx + dx, gy + dy
                        if self.is_valid_cell(ngx, ngy):
                            dist = math.hypot(dx, dy) * self.grid_resolution
                            if dist <= self.obstacle_inflation:
                                self.occupancy_grid[ngy, ngx] = 100
            
            angle += scan.angle_increment
    
    def heuristic(self, a, b):
        """A* heuristic: Euclidean distance"""
        return math.hypot(a[0] - b[0], a[1] - b[1])
    
    def get_neighbors(self, node):
        """Get valid neighboring cells for A*"""
        neighbors = []
        # 8-connected grid
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),  # Cardinal
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # Diagonal
        ]
        
        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if self.is_valid_cell(nx, ny):
                if self.occupancy_grid[ny, nx] < 50:  # Not obstacle
                    # Cost is higher for diagonal moves
                    cost = math.hypot(dx, dy)
                    neighbors.append(((nx, ny), cost))
        
        return neighbors
    
    def astar(self, start, goal):
        """A* pathfinding algorithm"""
        self.get_logger().info(f"A* planning from {start} to {goal}")
        
        # Priority queue: (f_score, g_score, node)
        open_set = []
        heapq.heappush(open_set, (0, 0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        visited = set()
        iterations = 0
        max_iterations = self.grid_size * self.grid_size
        
        while open_set and iterations < max_iterations:
            iterations += 1
            current = heapq.heappop(open_set)[2]
            
            if current in visited:
                continue
            visited.add(current)
            
            # Check if goal reached (within 2 cells)
            if self.heuristic(current, goal) < 2:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                self.get_logger().info(f"A* found path with {len(path)} waypoints in {iterations} iterations")
                return path
            
            for neighbor, cost in self.get_neighbors(current):
                if neighbor in visited:
                    continue
                    
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, tentative_g, neighbor))
        
        self.get_logger().warn(f"A* could not find path after {iterations} iterations")
        return None
    
    def plan_path(self):
        """Plan path from current position to goal using A*"""
        if self.goal_x is None or self.goal_y is None:
            self.get_logger().warn("No goal set!")
            return
        
        start_grid = self.world_to_grid(self.current_x, self.current_y)
        goal_grid = self.world_to_grid(self.goal_x, self.goal_y)
        
        self.get_logger().info(f"Planning path: ({self.current_x:.2f}, {self.current_y:.2f}) -> ({self.goal_x:.2f}, {self.goal_y:.2f})")
        self.get_logger().info(f"Grid coords: {start_grid} -> {goal_grid}")
        
        # Check if start and goal are valid
        if not self.is_valid_cell(*start_grid) or not self.is_valid_cell(*goal_grid):
            self.get_logger().error("Start or goal outside grid!")
            self.state = "IDLE"
            return
        
        # Run A* algorithm
        grid_path = self.astar(start_grid, goal_grid)
        
        if grid_path is None:
            self.get_logger().error("A* could not find a path!")
            self.state = "IDLE"
            return
        
        # Convert grid path to world coordinates
        self.path = []
        for gx, gy in grid_path:
            wx, wy = self.grid_to_world(gx, gy)
            self.path.append((wx, wy))
        
        # Simplify path by removing intermediate points on straight lines
        self.path = self.simplify_path(self.path)
        
        # Skip the first waypoint if it's very close to current position (starting point)
        if len(self.path) > 1:
            first_wp = self.path[0]
            dist_to_first = math.hypot(first_wp[0] - self.current_x, first_wp[1] - self.current_y)
            if dist_to_first < self.goal_tolerance:
                self.path = self.path[1:]  # Skip the starting point
                self.get_logger().info(f"Skipping start waypoint (distance {dist_to_first:.3f})")
        
        self.current_path_idx = 0
        self.state = "FOLLOWING"
        
        # Publish path for visualization
        self.publish_path()
        self.publish_markers()
        
        self.get_logger().info(f"Path planned with {len(self.path)} waypoints")
    
    def simplify_path(self, path):
        """Remove unnecessary waypoints from path"""
        if len(path) <= 2:
            return path
        
        simplified = [path[0]]
        
        for i in range(1, len(path) - 1):
            prev = simplified[-1]
            curr = path[i]
            next_pt = path[i + 1]
            
            # Check if curr is on the line from prev to next
            d1 = math.atan2(curr[1] - prev[1], curr[0] - prev[0])
            d2 = math.atan2(next_pt[1] - curr[1], next_pt[0] - curr[0])
            
            # If direction changes significantly, keep the waypoint
            if abs(d1 - d2) > 0.3:  # ~17 degrees
                simplified.append(curr)
        
        simplified.append(path[-1])
        return simplified
    
    def publish_path(self):
        """Publish planned path for RViz visualization"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for wx, wy in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def publish_markers(self):
        """Publish path markers for RViz visualization"""
        markers = MarkerArray()
        
        # Path line
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "path_line"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.1
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        
        for wx, wy in self.path:
            p = Point()
            p.x = wx
            p.y = wy
            p.z = 0.1
            line_marker.points.append(p)
        
        markers.markers.append(line_marker)
        
        # Goal marker
        if self.goal_x is not None:
            goal_marker = Marker()
            goal_marker.header.frame_id = "map"
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = "goal"
            goal_marker.id = 1
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = self.goal_x
            goal_marker.pose.position.y = self.goal_y
            goal_marker.pose.position.z = 0.5
            goal_marker.scale.x = 0.5
            goal_marker.scale.y = 0.5
            goal_marker.scale.z = 0.5
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 1.0
            markers.markers.append(goal_marker)
        
        # Start marker
        start_marker = Marker()
        start_marker.header.frame_id = "map"
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.ns = "start"
        start_marker.id = 2
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = self.current_x
        start_marker.pose.position.y = self.current_y
        start_marker.pose.position.z = 0.5
        start_marker.scale.x = 0.4
        start_marker.scale.y = 0.4
        start_marker.scale.z = 0.4
        start_marker.color.r = 0.0
        start_marker.color.g = 0.0
        start_marker.color.b = 1.0
        start_marker.color.a = 1.0
        markers.markers.append(start_marker)
        
        self.marker_pub.publish(markers)
    
    def publish_grid(self):
        """Publish occupancy grid for visualization"""
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = "map"
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.info.resolution = self.grid_resolution
        grid_msg.info.width = self.grid_size
        grid_msg.info.height = self.grid_size
        grid_msg.info.origin.position.x = self.grid_origin_x
        grid_msg.info.origin.position.y = self.grid_origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        grid_msg.data = self.occupancy_grid.flatten().tolist()
        self.grid_pub.publish(grid_msg)
    
    def control_loop(self):
        """Main control loop for path following"""
        cmd = Twist()
        
        # Wait for sensor data before starting
        if self.state == "WAITING":
            self.init_timer_count += 1
            if self.odom_received and self.scan_received:
                # Wait a bit more for stable data
                if self.init_timer_count > 50:  # 5 seconds at 10Hz
                    self.get_logger().info("Sensors ready! Starting navigation...")
                    if self.auto_start:
                        self.goal_x = self.default_goal_x
                        self.goal_y = self.default_goal_y
                        self.get_logger().info(f"Auto-navigating to ({self.goal_x}, {self.goal_y})")
                        self.state = "PLANNING"
                        self.plan_path()
                    else:
                        self.state = "IDLE"
                        self.get_logger().info("Waiting for goal on /goal_pose topic...")
            elif self.init_timer_count % 20 == 0:
                self.get_logger().warn(f"Waiting for sensors... Odom: {self.odom_received}, Scan: {self.scan_received}")
            self.cmd_vel_pub.publish(cmd)
            return
        
        if self.state == "IDLE":
            # Just wait for goal
            self.cmd_vel_pub.publish(cmd)
            return
        
        if self.state == "PLANNING":
            # Planning in progress
            self.cmd_vel_pub.publish(cmd)
            return
        
        if self.state == "REACHED":
            self.get_logger().info("Goal reached! Waiting for new goal...")
            self.state = "IDLE"
            self.cmd_vel_pub.publish(cmd)
            return
        
        if self.state == "FOLLOWING":
            if not self.path or self.current_path_idx >= len(self.path):
                self.state = "REACHED"
                self.cmd_vel_pub.publish(cmd)
                return
            
            # Get current target waypoint
            target_x, target_y = self.path[self.current_path_idx]
            
            # Calculate distance and angle to target
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            
            # Log progress periodically (every ~2 seconds at 10Hz)
            self.progress_counter = getattr(self, 'progress_counter', 0) + 1
            if self.progress_counter % 20 == 0:
                self.get_logger().info(f"Following: wp {self.current_path_idx+1}/{len(self.path)}, "
                                      f"pos=({self.current_x:.2f},{self.current_y:.2f}), "
                                      f"target=({target_x:.2f},{target_y:.2f}), dist={distance:.2f}")
            
            # Angle difference
            angle_diff = target_angle - self.current_yaw
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Check if reached current waypoint
            if distance < self.goal_tolerance:
                self.current_path_idx += 1
                if self.current_path_idx >= len(self.path):
                    self.get_logger().info(f"Reached final goal at ({self.goal_x:.2f}, {self.goal_y:.2f})!")
                    self.state = "REACHED"
                    self.cmd_vel_pub.publish(cmd)
                    return
                else:
                    self.get_logger().info(f"Waypoint {self.current_path_idx}/{len(self.path)} reached, pos=({self.current_x:.2f}, {self.current_y:.2f})")
                    # Continue to next waypoint immediately - don't return
                    target_x, target_y = self.path[self.current_path_idx]
                    dx = target_x - self.current_x
                    dy = target_y - self.current_y
                    distance = math.hypot(dx, dy)
                    target_angle = math.atan2(dy, dx)
                    angle_diff = target_angle - self.current_yaw
                    while angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    while angle_diff < -math.pi:
                        angle_diff += 2 * math.pi
            
            # Check for obstacles (emergency stop) - DISABLED for now
            # The obstacle detection was too sensitive and needs tuning
            # Uncomment below to re-enable with proper threshold tuning
            """
            self.replan_cooldown = getattr(self, 'replan_cooldown', 0)
            if self.replan_cooldown > 0:
                self.replan_cooldown -= 1
            
            if self.scan_data is not None and self.replan_cooldown == 0:
                front_clear = True
                obstacle_count = 0
                for i, r in enumerate(self.scan_data.ranges):
                    angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                    if -0.4 < angle < 0.4:  # Front sector (narrower)
                        if 0.1 < r < 0.3:  # Only very close obstacles (reduced threshold)
                            obstacle_count += 1
                
                # Need multiple readings to confirm obstacle (noise filtering)
                if obstacle_count > 5:
                    front_clear = False
                
                if not front_clear:
                    # Obstacle ahead - replan
                    self.get_logger().warn("Obstacle detected! Replanning...")
                    self.state = "PLANNING"
                    self.plan_path()
                    self.replan_cooldown = 30  # Wait ~3 seconds before checking again
                    self.cmd_vel_pub.publish(cmd)
                    return
            """
            
            # Path following control - always send velocity commands
            if abs(angle_diff) > 0.3:
                # Turn in place first
                cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                cmd.linear.x = 0.05  # Small forward motion while turning
            else:
                # Move toward target
                cmd.linear.x = min(self.linear_speed, max(0.1, distance * 0.5))
                cmd.angular.z = angle_diff * 1.5  # Proportional steering
            
            # Debug log velocity commands occasionally
            if self.progress_counter % 20 == 0:
                self.get_logger().info(f"CMD: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}, angle_diff={angle_diff:.2f}")
        
        self.cmd_vel_pub.publish(cmd)


def main():
    rclpy.init()
    node = AStarPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

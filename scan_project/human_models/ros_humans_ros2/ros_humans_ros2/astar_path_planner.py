#!/usr/bin/env python3
"""
A* Path Planner Node for TurtleBot Rover
Implements A* algorithm for finding the shortest path between two points
Uses occupancy grid from LiDAR data for obstacle avoidance
"""

import math
import heapq
import rclpy
import rclpy.qos
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point, PoseArray
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
        self.declare_parameter("obstacle_inflation", 0.7)  # meters - increased from 0.35
        self.declare_parameter("room_min_x", -10.0)
        self.declare_parameter("room_max_x", 10.0)
        self.declare_parameter("room_min_y", -10.0)
        self.declare_parameter("room_max_y", 10.0)
        self.declare_parameter("auto_start", True)  # Auto start navigation
        self.declare_parameter("default_goal_x", 5.0)
        self.declare_parameter("default_goal_y", 5.0)
        # Pure pursuit path following parameters (anti-drift)
        self.declare_parameter("look_ahead_dist", 0.8)  # metres look-ahead - increased for smoother tracking
        self.declare_parameter("steering_gain", 1.2)     # proportional steering gain - reduced to prevent oscillation
        self.declare_parameter("max_cross_track_error", 0.5)  # max allowed cross-track error before correction
        # Human-aware planning parameters
        self.declare_parameter("human_inflation_radius", 1.2)  # metres around detected humans - increased from 0.8
        self.declare_parameter("human_predict_horizon", 3.0)   # seconds to predict ahead
        self.declare_parameter("human_costmap_weight", 80)     # cost injected for human cells - increased from 60
        self.declare_parameter("human_replan_dist", 4.0)       # replan if human within this dist of robot
        
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
        # Pure pursuit parameters for drift correction
        self.look_ahead_dist = self.get_parameter("look_ahead_dist").value
        self.steering_gain = self.get_parameter("steering_gain").value
        self.max_cross_track_error = self.get_parameter("max_cross_track_error").value
        # Cross-track error integral (for drift correction)
        self.cross_track_integral = 0.0
        self.integral_gain = 0.1  # small integral term to correct cumulative drift
        self.human_inflation = self.get_parameter("human_inflation_radius").value
        self.human_predict_horizon = self.get_parameter("human_predict_horizon").value
        self.human_cost_weight = self.get_parameter("human_costmap_weight").value
        self.human_replan_dist = self.get_parameter("human_replan_dist").value
        # Robot spawn position in world/map frame (odom starts at 0,0 at spawn).
        # Add this offset to convert odom coords → world/map coords.
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", -8.0)  # rover spawns at y=-8 in large_messy_room
        self.declare_parameter("spawn_yaw", 1.5708)  # rover faces +Y at spawn
        self.spawn_x = self.get_parameter("spawn_x").value
        self.spawn_y = self.get_parameter("spawn_y").value
        self.spawn_yaw = self.get_parameter("spawn_yaw").value
        # Precompute rotation constants for odom→world conversion
        self.spawn_cos = math.cos(self.spawn_yaw)
        self.spawn_sin = math.sin(self.spawn_yaw)
        
        # State variables (initialised to spawn position so they are valid
        # before the first /odom message arrives)
        self.current_x = self.spawn_x
        self.current_y = self.spawn_y
        self.current_yaw = self.spawn_yaw
        self.goal_x = None
        self.goal_y = None
        self.path = []
        self.current_path_idx = 0
        self.scan_data = None
        self.state = "WAITING"  # WAITING, IDLE, PLANNING, FOLLOWING, REACHED
        self.odom_received = False
        self.scan_received = False
        self.init_timer_count = 0
        
        # Occupancy grid for path planning (will be re-sized when static map arrives)
        self.grid_width = self.grid_size    # columns
        self.grid_height = self.grid_size   # rows
        self.grid_origin_x = -self.grid_size * self.grid_resolution / 2
        self.grid_origin_y = -self.grid_size * self.grid_resolution / 2
        self.occupancy_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        self.static_map_info = None  # set once the world map is received
        self.static_grid = None      # permanent copy of world-map obstacles (never decayed)
        self.replan_cooldown = 0     # control cycles before next allowed replan
        self.decay_counter = 0       # throttles costmap decay to every 5 scans
        
        # Human tracking state (from CV detector)
        self.detected_humans = {}     # dict {id: (x, y)} – ID-keyed to survive race condition
        self.human_velocities = {}     # dict {id: (vx, vy)}
        self.human_grid = None         # dynamic human obstacle layer
        self.human_replan_cooldown = 0 # cycles before next human-triggered replan
        self.human_speed_factor = 1.0  # dynamic speed multiplier (reduced near humans)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.path2_pub = self.create_publisher(Path, "/planned_path_2", 10)
        self.path3_pub = self.create_publisher(Path, "/planned_path_3", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/path_markers", 10)
        # /local_costmap is published with the SAME origin as the static /map
        # so all three layers stack perfectly in RViz
        self.grid_pub = self.create_publisher(OccupancyGrid, "/local_costmap", 10)
        
        # Store all 3 candidate paths and their distances
        self.all_paths = []       # list of [(x,y), ...] paths
        self.all_distances = []   # corresponding total distances in metres
        
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
        # Subscribe to CV-detected human positions and velocities
        self.human_sub = self.create_subscription(
            PoseArray, "/detected_humans", self.humans_callback, 10
        )
        self.human_vel_sub = self.create_subscription(
            PoseArray, "/human_velocities", self.human_vel_callback, 10
        )
        # Subscribe to the static world map so the costmap can align with it
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.static_map_callback, 
            rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
            )
        )
        
        # Control loop timer - 20 Hz for tight path following (was 10 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.grid_timer = self.create_timer(1.0, self.publish_grid)
        
        self.get_logger().info("A* Path Planner started! (human-aware mode)")
        self.get_logger().info("Waiting for odometry and scan data...")
        self.get_logger().info(f"Human avoidance: inflate={self.human_inflation}m, "
                              f"predict={self.human_predict_horizon}s, "
                              f"replan_dist={self.human_replan_dist}m")
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
        # Snapshot robot pose at this exact moment so obstacle world-frame
        # positions are consistent with the scan (prevents stale-yaw drift).
        self.update_occupancy_grid(msg, self.current_x, self.current_y, self.current_yaw)
    
    def odom_callback(self, msg):
        """Update robot position from odometry.
        The DiffDrive odom starts at (0,0,yaw=0) at the robot's spawn pose,
        so we must apply a full rotation+translation to convert odom coords
        into world/map coords:
            world = R(spawn_yaw) · odom + (spawn_x, spawn_y)
            world_yaw = odom_yaw + spawn_yaw
        """
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        # Rotate odom position by spawn_yaw, then translate
        self.current_x = self.spawn_x + ox * self.spawn_cos - oy * self.spawn_sin
        self.current_y = self.spawn_y + ox * self.spawn_sin + oy * self.spawn_cos
        if not self.odom_received:
            self.get_logger().info(
                f"Odometry received! World position: ({self.current_x:.2f}, {self.current_y:.2f})")
            self.odom_received = True
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        odom_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_yaw = odom_yaw + self.spawn_yaw
    
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
    
    def static_map_callback(self, msg):
        """Align planner grid with the static world map on first receipt."""
        if self.static_map_info is not None:
            return  # only initialise once

        self.static_map_info = msg.info
        # Use the static map's origin so the costmap frame matches /map
        self.grid_origin_x = msg.info.origin.position.x
        self.grid_origin_y = msg.info.origin.position.y

        # Compute new grid dimensions that cover the same metric area
        map_w_m = msg.info.width  * msg.info.resolution
        map_h_m = msg.info.height * msg.info.resolution
        new_w = max(1, min(300, int(map_w_m / self.grid_resolution)))
        new_h = max(1, min(300, int(map_h_m / self.grid_resolution)))
        self.grid_width  = new_w
        self.grid_height = new_h

        # Build a fresh grid seeded with static-map obstacles
        static = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )
        # Vectorised sampling: for each planner cell find the nearest
        # static-map cell and mark it occupied when value >= 50
        gx_idx = np.arange(new_w)
        gy_idx = np.arange(new_h)
        wx = gx_idx * self.grid_resolution + self.grid_origin_x
        wy = gy_idx * self.grid_resolution + self.grid_origin_y
        mx = np.clip(
            ((wx - msg.info.origin.position.x) / msg.info.resolution).astype(int),
            0, msg.info.width - 1
        )
        my = np.clip(
            ((wy - msg.info.origin.position.y) / msg.info.resolution).astype(int),
            0, msg.info.height - 1
        )
        mx_grid, my_grid = np.meshgrid(mx, my)  # shape (new_h, new_w)
        sampled = static[my_grid, mx_grid]
        self.occupancy_grid = np.where(sampled >= 50, 100, 0).astype(np.int8)
        # Keep a permanent copy: LiDAR decay must never erase world-map walls
        self.static_grid = self.occupancy_grid.copy()

        self.get_logger().info(
            f"Costmap aligned with static map: {new_w}x{new_h} cells @ "
            f"{self.grid_resolution}m/cell, origin "
            f"({self.grid_origin_x:.2f}, {self.grid_origin_y:.2f})"
        )

    # ── Human detection callbacks ─────────────────────────────────────────

    def humans_callback(self, msg):
        """Receive detected human positions – keyed by tracker ID (orientation.w)."""
        self.detected_humans = {
            int(p.orientation.w): (p.position.x, p.position.y)
            for p in msg.poses
        }
        # Drop velocity entries for humans no longer tracked
        stale = [k for k in self.human_velocities if k not in self.detected_humans]
        for k in stale:
            del self.human_velocities[k]
        # Inject human obstacles into the costmap
        self._inject_human_costmap()

    def human_vel_callback(self, msg):
        """Receive detected human velocities – keyed by tracker ID (orientation.w)."""
        for p in msg.poses:
            hid = int(p.orientation.w)
            self.human_velocities[hid] = (p.position.x, p.position.y)

    def _inject_human_costmap(self):
        """Inject detected humans + predicted positions into the costmap.

        For each detected human:
          1. Mark current position with a high-cost inflated circle.
          2. Predict future positions at 0.5s intervals up to
             human_predict_horizon and mark them with decaying cost.
        This creates a \"sweep\" in the costmap along the human's
        predicted trajectory, encouraging A* to route around them.
        """
        if not self.detected_humans:
            return

        inflation_cells = int(self.human_inflation / self.grid_resolution)
        predict_steps = int(self.human_predict_horizon / 0.5)  # every 0.5s

        for hid, (hx, hy) in self.detected_humans.items():
            # Get velocity by ID (default to stationary if not yet received)
            vx, vy = self.human_velocities.get(hid, (0.0, 0.0))

            # Mark current position + predicted positions
            for step in range(predict_steps + 1):
                dt = step * 0.5
                px = hx + vx * dt
                py = hy + vy * dt

                # Cost decays with prediction time (less certain = lower cost)
                if step == 0:
                    cost = self.human_cost_weight  # current: high cost
                else:
                    decay = 1.0 - (dt / (self.human_predict_horizon + 0.1))
                    cost = int(self.human_cost_weight * decay * 0.7)
                cost = max(1, min(98, cost))  # keep below lethal (99)

                gx, gy = self.world_to_grid(px, py)
                for dx in range(-inflation_cells, inflation_cells + 1):
                    for dy in range(-inflation_cells, inflation_cells + 1):
                        ngx, ngy = gx + dx, gy + dy
                        if self.is_valid_cell(ngx, ngy):
                            dist_cells = math.hypot(dx, dy)
                            if dist_cells <= inflation_cells:
                                # Radial falloff within inflation zone
                                ratio = dist_cells / max(inflation_cells, 1)
                                cell_cost = int(cost * (1.0 - 0.6 * ratio))
                                cell_cost = max(1, cell_cost)
                                # Only increase, never overwrite static walls
                                if cell_cost > self.occupancy_grid[ngy, ngx]:
                                    if (self.static_grid is None or
                                            self.static_grid[ngy, ngx] < 99):
                                        self.occupancy_grid[ngy, ngx] = cell_cost

    # ── Smart human-aware replanning ─────────────────────────────────────

    def _analyze_human_threats(self):
        """Analyze detected humans relative to the upcoming path.

        For each human within range, determine the best response:
          'ignore'    – human is far from path or moving away
          'slow_down' – human is near but will clear, or far enough
          'divert'    – human blocks the path; insert local arc
          'replan'    – diversion not possible (walls), full A* needed

        Considers:
          • Human distance to upcoming path waypoints (not just to robot)
          • Velocity direction: toward / away from path & robot
          • Predicted clearance time
          • Which side to divert (go *behind* the human)
        """
        threats = []
        if not self.detected_humans or not self.path:
            return threats
        if self.current_path_idx >= len(self.path):
            return threats

        for hid, (hx, hy) in self.detected_humans.items():
            # Velocity matched by ID – no index race condition
            vx, vy = self.human_velocities.get(hid, (0.0, 0.0))
            human_speed = math.hypot(vx, vy)

            # Distance from robot to human
            robot_dist = math.hypot(hx - self.current_x, hy - self.current_y)
            # Only act on humans within the replan radius (strict 4 m)
            if robot_dist > self.human_replan_dist:
                continue

            # Find the closest UPCOMING path waypoint to the human
            min_path_dist = float('inf')
            closest_wp_idx = self.current_path_idx
            lookahead = min(self.current_path_idx + 15, len(self.path))
            for wi in range(self.current_path_idx, lookahead):
                d = math.hypot(hx - self.path[wi][0], hy - self.path[wi][1])
                if d < min_path_dist:
                    min_path_dist = d
                    closest_wp_idx = wi

            threat = {
                'idx': hid, 'hx': hx, 'hy': hy,
                'vx': vx, 'vy': vy, 'speed': human_speed,
                'robot_dist': robot_dist, 'path_dist': min_path_dist,
                'segment_idx': closest_wp_idx,
            }

            # If human is far from upcoming path → ignore
            if min_path_dist > self.human_inflation + 1.0:
                threat['action'] = 'ignore'
                threats.append(threat)
                continue

            # ── Velocity analysis ──────────────────────────────────
            wpx, wpy = self.path[closest_wp_idx]
            to_path_x, to_path_y = wpx - hx, wpy - hy
            to_path_len = math.hypot(to_path_x, to_path_y)

            # Rate at which human is approaching the path (>0 = closing)
            approach_rate = 0.0
            if to_path_len > 0.01:
                approach_rate = (vx * to_path_x + vy * to_path_y) / to_path_len

            # Rate at which human is closing on the robot (>0 = closing)
            to_robot_x = self.current_x - hx
            to_robot_y = self.current_y - hy
            closing_rate = 0.0
            if robot_dist > 0.01:
                closing_rate = (vx * to_robot_x + vy * to_robot_y) / robot_dist

            # ── Decision logic ─────────────────────────────────────
            # 1. Moving AWAY from both path and robot → ignore
            if approach_rate < -0.2 and closing_rate < -0.15:
                threat['action'] = 'ignore'
                threats.append(threat)
                continue

            # 2. Crossing the path (mostly perpendicular, will clear soon)
            if human_speed > 0.15 and abs(approach_rate) < human_speed * 0.5:
                clear_dist = max(0.0, self.human_inflation + 0.5 - min_path_dist)
                time_to_clear = (clear_dist / human_speed) if human_speed > 0.1 else 0.0
                if time_to_clear < 4.0:
                    threat['action'] = 'slow_down'
                    threat['time_to_clear'] = time_to_clear
                    threats.append(threat)
                    continue

            # 3. Human is ON or very near the path AND within 4m → divert around
            if min_path_dist < self.human_inflation + 0.5 and robot_dist <= self.human_replan_dist:
                threat['action'] = 'divert'
                threat['divert_side'] = self._pick_divert_side(
                    hx, hy, vx, vy, human_speed, closest_wp_idx)
                threats.append(threat)
                continue

            # 4. Human on path but >4m away, or off-path but close → slow down only
            threat['action'] = 'slow_down'
            threats.append(threat)

        return threats

    def _pick_divert_side(self, hx, hy, vx, vy, speed, wp_idx):
        """Choose which side (left/right of path) to divert to.

        Strategy:
          • If human is moving → go BEHIND them (opposite to velocity).
          • If stationary → pick the side with more free space.
        """
        # Get path direction at the threatened segment
        if wp_idx + 1 < len(self.path):
            seg_dx = self.path[wp_idx + 1][0] - self.path[wp_idx][0]
            seg_dy = self.path[wp_idx + 1][1] - self.path[wp_idx][1]
        elif wp_idx > 0:
            seg_dx = self.path[wp_idx][0] - self.path[wp_idx - 1][0]
            seg_dy = self.path[wp_idx][1] - self.path[wp_idx - 1][1]
        else:
            seg_dx = self.path[wp_idx][0] - self.current_x
            seg_dy = self.path[wp_idx][1] - self.current_y

        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 0.01:
            return 'left'

        if speed > 0.15:
            # Human is moving — cross product of path dir × human velocity
            # tells us which side the human is moving toward; go the OTHER side.
            cross = seg_dx * vy - seg_dy * vx
            return 'right' if cross > 0 else 'left'
        else:
            # Stationary — probe both sides and pick the clearer one
            nl_x = -seg_dy / seg_len   # left normal
            nl_y =  seg_dx / seg_len
            probe = self.human_inflation + 0.8
            lx, ly = hx + nl_x * probe, hy + nl_y * probe
            rx, ry = hx - nl_x * probe, hy - nl_y * probe
            lgx, lgy = self.world_to_grid(lx, ly)
            rgx, rgy = self.world_to_grid(rx, ry)
            l_cost = int(self.occupancy_grid[lgy, lgx]) if self.is_valid_cell(lgx, lgy) else 100
            r_cost = int(self.occupancy_grid[rgy, rgx]) if self.is_valid_cell(rgx, rgy) else 100
            return 'left' if l_cost <= r_cost else 'right'

    def _local_diversion(self, threat):
        """Insert a small arc of waypoints around a human into the current
        path.  Much cheaper than a full A* replan and keeps the rest of the
        route intact.

        Creates 3 offset waypoints (approach / pass / rejoin) on the chosen
        side of the human, accounting for the human's predicted motion.

        Returns True if the diversion was inserted, False if the detour
        cells are blocked and a full replan is needed.
        """
        hx, hy = threat['hx'], threat['hy']
        vx, vy = threat['vx'], threat['vy']
        wp_idx = threat['segment_idx']
        side = threat.get('divert_side', 'left')

        # Get path direction at the threatened waypoint
        if wp_idx + 1 < len(self.path):
            seg_dx = self.path[wp_idx + 1][0] - self.path[wp_idx][0]
            seg_dy = self.path[wp_idx + 1][1] - self.path[wp_idx][1]
        elif wp_idx > 0:
            seg_dx = self.path[wp_idx][0] - self.path[wp_idx - 1][0]
            seg_dy = self.path[wp_idx][1] - self.path[wp_idx - 1][1]
        else:
            seg_dx = self.path[wp_idx][0] - self.current_x
            seg_dy = self.path[wp_idx][1] - self.current_y

        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 0.01:
            return False

        # Unit vectors: along path and perpendicular (normal)
        udx, udy = seg_dx / seg_len, seg_dy / seg_len
        if side == 'left':
            nx, ny = -udy, udx
        else:
            nx, ny = udy, -udx

        # Predict human position 1.5 s ahead (mid-pass estimate)
        pred_hx = hx + vx * 1.5
        pred_hy = hy + vy * 1.5

        # Offset distance: enough clearance around the human
        offset = self.human_inflation + 0.6

        # Three diversion waypoints
        # 1. Approach: 1.2 m before human, offset to the side
        wp1 = (hx - udx * 1.2 + nx * offset,
               hy - udy * 1.2 + ny * offset)
        # 2. Pass: midpoint of current & predicted human pos, full offset
        mid_hx = (hx + pred_hx) / 2.0
        mid_hy = (hy + pred_hy) / 2.0
        wp2 = (mid_hx + nx * offset,
               mid_hy + ny * offset)
        # 3. Rejoin: 1.2 m after human, easing back toward original path
        wp3 = (hx + udx * 1.2 + nx * offset * 0.5,
               hy + udy * 1.2 + ny * offset * 0.5)

        # Validate all diversion waypoints are in free space
        for (wx, wy) in [wp1, wp2, wp3]:
            gx, gy = self.world_to_grid(wx, wy)
            if not self.is_valid_cell(gx, gy):
                return False
            if self.occupancy_grid[gy, gx] >= 80:
                return False
            if not (self.room_min_x < wx < self.room_max_x and
                    self.room_min_y < wy < self.room_max_y):
                return False

        # Splice the diversion into the path
        insert_start = max(self.current_path_idx, wp_idx - 1)
        rejoin_idx = min(wp_idx + 2, len(self.path))
        new_path = (list(self.path[:insert_start]) +
                    [wp1, wp2, wp3] +
                    list(self.path[rejoin_idx:]))
        self.path = new_path
        self.current_path_idx = insert_start

        # Update stored paths so RViz shows the diversion
        if self.all_paths:
            self.all_paths[0] = self.path
            if self.all_distances:
                self.all_distances[0] = self.path_distance(
                    [(x, y) for x, y in self.path])

        return True

    def _get_look_ahead_point(self):
        """Find a look-ahead point on the path for pure pursuit steering.
        
        Searches forward from the current waypoint to find a point that is
        approximately look_ahead_dist away from the robot.  This creates
        smooth steering behavior and reduces oscillation/drift.
        """
        if not self.path or self.current_path_idx >= len(self.path):
            return self.current_x, self.current_y
        
        # Start from current target waypoint
        look_ahead_x = self.path[self.current_path_idx][0]
        look_ahead_y = self.path[self.current_path_idx][1]
        
        # Search forward to find a point at look_ahead_dist
        accumulated_dist = 0.0
        prev_x, prev_y = self.current_x, self.current_y
        
        for i in range(self.current_path_idx, len(self.path)):
            wp_x, wp_y = self.path[i]
            segment_dist = math.hypot(wp_x - prev_x, wp_y - prev_y)
            accumulated_dist += segment_dist
            
            if accumulated_dist >= self.look_ahead_dist:
                # Interpolate to get exact look-ahead point
                overshoot = accumulated_dist - self.look_ahead_dist
                if segment_dist > 0.01:
                    ratio = overshoot / segment_dist
                    look_ahead_x = wp_x - ratio * (wp_x - prev_x)
                    look_ahead_y = wp_y - ratio * (wp_y - prev_y)
                else:
                    look_ahead_x, look_ahead_y = wp_x, wp_y
                break
            
            look_ahead_x, look_ahead_y = wp_x, wp_y
            prev_x, prev_y = wp_x, wp_y
        
        return look_ahead_x, look_ahead_y
    
    def _compute_cross_track_error(self):
        """Compute the perpendicular distance from robot to the current path segment.
        
        Returns the signed cross-track error and the path heading angle:
          positive = robot is to the left of the path  (needs right-steer correction)
          negative = robot is to the right of the path (needs left-steer correction)
        
        Uses the segment the robot is currently travelling on:
          path[current_path_idx - 1]  →  path[current_path_idx]  (current target)
        """
        if not self.path or self.current_path_idx >= len(self.path):
            return 0.0, self.current_yaw
        
        # Use the segment the robot is CURRENTLY ON:
        # from the previously-reached waypoint to the current target.
        if self.current_path_idx > 0:
            seg_start_x, seg_start_y = self.path[self.current_path_idx - 1]
            seg_end_x,   seg_end_y   = self.path[self.current_path_idx]
        elif len(self.path) > 1:
            # Still on the very first segment
            seg_start_x, seg_start_y = self.path[0]
            seg_end_x,   seg_end_y   = self.path[1]
        else:
            return 0.0, self.current_yaw
        
        # Vector along the current path segment
        path_dx = seg_end_x - seg_start_x
        path_dy = seg_end_y - seg_start_y
        path_len = math.hypot(path_dx, path_dy)
        
        if path_len < 0.01:
            return 0.0, self.current_yaw
        
        # Path heading angle for this segment
        path_heading = math.atan2(path_dy, path_dx)
        
        # Vector from segment start to robot
        robot_dx = self.current_x - seg_start_x
        robot_dy = self.current_y - seg_start_y
        
        # Signed perpendicular distance via 2-D cross product:
        #   positive = robot is to the LEFT  of the path direction
        #   negative = robot is to the RIGHT of the path direction
        cross_track = (path_dx * robot_dy - path_dy * robot_dx) / path_len
        
        return cross_track, path_heading

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
        return 0 <= gx < self.grid_width and 0 <= gy < self.grid_height
    
    def update_occupancy_grid(self, scan, robot_x=None, robot_y=None, robot_yaw=None):
        """Update occupancy grid from laser scan with gradient inflation.
        
        Instead of binary 0/100, cells near obstacles get a graduated cost
        that falls off with distance.  This makes A* strongly prefer paths
        that keep clearance from walls, producing much smoother routes.
        
        Cost profile (per-cell):
            distance == 0           → 100  (lethal / impassable)
            0 < distance ≤ inscribed → 99  (definitely collision)
            inscribed < d ≤ inflation → linear 80→1
        """
        if scan is None:
            return
        
        # Decay dynamic obstacles only every 5 scans (≈2 Hz instead of 10 Hz)
        # Decay every 2 scans (was every 5) — faster decay reduces pink blob buildup.
        # Decay by 3 per step (was 1) so dynamic obstacles clear in ~1s not ~5s.
        self.decay_counter += 1
        if self.decay_counter % 2 == 0:
            self.occupancy_grid = np.clip(self.occupancy_grid - 3, 0, 100)
            # Restore permanent world-map walls so they are never decayed away
            if self.static_grid is not None:
                np.maximum(self.occupancy_grid, self.static_grid, out=self.occupancy_grid)
        
        # Rover dimensions from world file:
        #   chassis box  = 0.28 × 0.18 m (L × W)
        #   wheels at y=±0.12, half-width 0.0125 → total width 0.265 m
        #   circumscribed radius = √(0.14² + 0.1325²) ≈ 0.193 m
        # Use 0.40 m lethal radius (increased for safety margin)
        # so A* never routes the robot closer than 0.40 m to any obstacle.
        inscribed_radius = 0.40  # lethal zone: A* fully avoids everything within this dist
        inflation_radius = self.obstacle_inflation  # 0.70 m from launch param
        inflation_cells = int(inflation_radius / self.grid_resolution)

        # Use the snapshot pose passed from scan_callback to avoid stale-yaw errors
        # (odom and scan callbacks may interleave at different rates).
        rx = robot_x if robot_x is not None else self.current_x
        ry = robot_y if robot_y is not None else self.current_y
        ryaw = robot_yaw if robot_yaw is not None else self.current_yaw

        angle = scan.angle_min
        for i, r in enumerate(scan.ranges):
            if scan.range_min < r < scan.range_max:
                # Calculate obstacle position in world frame using snapshot pose
                obs_x = rx + r * math.cos(ryaw + angle)
                obs_y = ry + r * math.sin(ryaw + angle)
                
                # Mark obstacle in grid with gradient inflation
                gx, gy = self.world_to_grid(obs_x, obs_y)
                
                for dx in range(-inflation_cells, inflation_cells + 1):
                    for dy in range(-inflation_cells, inflation_cells + 1):
                        ngx, ngy = gx + dx, gy + dy
                        if self.is_valid_cell(ngx, ngy):
                            dist = math.hypot(dx, dy) * self.grid_resolution
                            if dist <= 0.01:
                                cost = 100  # lethal
                            elif dist <= inscribed_radius:
                                cost = 99   # definitely collision
                            elif dist <= inflation_radius:
                                # Linear falloff from 80 to 1
                                ratio = (dist - inscribed_radius) / (inflation_radius - inscribed_radius)
                                cost = int(80 - 79 * ratio)
                            else:
                                continue
                            # Only increase cost, never decrease
                            if cost > self.occupancy_grid[ngy, ngx]:
                                self.occupancy_grid[ngy, ngx] = cost
            
            angle += scan.angle_increment
    
    def heuristic(self, a, b):
        """Weighted A* heuristic: Octile distance with optimal weighting.
        
        Octile distance is the optimal heuristic for 8-connected grids
        (accounts for diagonal moves costing √2).  The 1.05 weight slightly
        favours expansion toward the goal while maintaining near-optimal paths.
        A cross-product tie-breaker breaks symmetry to produce straighter paths.
        """
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        # Octile distance: min(dx,dy)*√2 + |dx-dy|*1
        octile = min(dx, dy) * 1.4142 + abs(dx - dy)
        # Weight = 1.05 → slight bias toward goal, still near-optimal
        weight = 1.05
        # Cross-product tie-breaker for straighter paths (relative to start→goal line)
        # This biases toward paths that stay close to the direct line
        cross = abs(dx * (b[1] - a[1]) - dy * (b[0] - a[0])) * 0.0005
        return octile * weight + cross
    
    def get_neighbors(self, node):
        """Get valid neighboring cells for A* with proximity cost.
        
        Cells with gradient costmap values are traversable but penalised.
        Cells >= 90 are now considered lethal to avoid planning near edges.
        """
        neighbors = []
        # 8-connected grid
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),  # Cardinal
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # Diagonal
        ]
        
        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if self.is_valid_cell(nx, ny):
                cell_cost = self.occupancy_grid[ny, nx]
                if cell_cost >= 90:  # lethal obstacle — impassable (lowered from 99)
                    continue
                # Base movement cost (√2 for diagonal, 1 for cardinal)
                move_cost = math.hypot(dx, dy)
                # Proximity penalty: cells near obstacles cost MUCH more to traverse
                # This makes A* route through open corridors, avoiding edges
                if cell_cost > 0:
                    # Scale 1-89 → penalty up to 4.0 (STRONG preference for clearance)
                    proximity_penalty = (cell_cost / 89.0) * 4.0
                    move_cost += proximity_penalty
                neighbors.append(((nx, ny), move_cost))
        
        return neighbors
    
    def astar(self, start, goal, blocked_nodes=None, blocked_edges=None):
        """Weighted A* with octile heuristic and proximity-aware costs.
        
        Key improvements over basic A*:
        1. Octile heuristic (tight for 8-connected grid)
        2. 1.2× weight for faster convergence
        3. Tie-breaking for cleaner paths
        4. Proximity cost makes paths avoid wall-hugging
        5. Goal tolerance of 2 cells
        
        Optional parameters for Yen's K-shortest-paths:
            blocked_nodes: set of (gx,gy) nodes that must not be visited
            blocked_edges: set of ((gx1,gy1),(gx2,gy2)) edges that must not be used
        """
        if blocked_nodes is None:
            blocked_nodes = set()
        if blocked_edges is None:
            blocked_edges = set()
        
        # Priority queue: (f_score, counter, node) — counter breaks ties
        counter = 0
        open_set = []
        heapq.heappush(open_set, (0, counter, start))
        
        came_from = {}
        g_score = {start: 0}
        
        visited = set()
        iterations = 0
        max_iterations = self.grid_width * self.grid_height * 4
        
        while open_set and iterations < max_iterations:
            iterations += 1
            f, _, current = heapq.heappop(open_set)
            
            if current in visited:
                continue
            visited.add(current)
            
            # Check if goal reached (within 2 cells)
            dist_to_goal = math.hypot(current[0] - goal[0], current[1] - goal[1])
            if dist_to_goal < 2:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            
            for neighbor, cost in self.get_neighbors(current):
                if neighbor in visited:
                    continue
                if neighbor in blocked_nodes:
                    continue
                if (current, neighbor) in blocked_edges:
                    continue
                    
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = self.heuristic(neighbor, goal)
                    f = tentative_g + h
                    counter += 1
                    heapq.heappush(open_set, (f, counter, neighbor))
        
        return None
    
    def path_distance(self, path):
        """Compute total Euclidean distance of a path in metres."""
        total = 0.0
        for i in range(1, len(path)):
            total += math.hypot(path[i][0] - path[i-1][0],
                                path[i][1] - path[i-1][1])
        return total
    
    def grid_path_cost(self, grid_path):
        """Compute the total A* traversal cost of a grid path (including
        proximity penalties), matching the same cost model used during search."""
        total = 0.0
        for i in range(1, len(grid_path)):
            prev = grid_path[i - 1]
            curr = grid_path[i]
            dx = curr[0] - prev[0]
            dy = curr[1] - prev[1]
            move_cost = math.hypot(dx, dy)
            cell_cost = self.occupancy_grid[curr[1], curr[0]]
            if cell_cost > 0 and cell_cost < 99:
                move_cost += (cell_cost / 98.0) * 2.0
            total += move_cost
        return total
    
    def k_shortest_paths(self, start_grid, goal_grid, k=3):
        """Find the K true shortest paths using Yen's algorithm.
        
        Yen's algorithm guarantees that the returned paths are the 1st,
        2nd, and 3rd shortest *loopless* paths from start to goal.
        
        Algorithm outline:
        1. Find the shortest path A[0] with normal A*.
        2. For each k from 1..K-1:
           a. For each spur node in A[k-1]:
              - Root path = A[k-1][0..spur_index]
              - Block edges leaving spur node that overlap with any
                previously found path sharing the same root.
              - Block all nodes in the root path (except spur node)
                to prevent loops.
              - Find spur path from spur node to goal.
              - Candidate = root path + spur path.
           b. Pick the candidate with the lowest total cost → A[k].
        
        Returns (paths_world, distances) sorted shortest-first.
        """
        self.get_logger().info(f"Yen's K-shortest: finding {k} paths from {start_grid} to {goal_grid}")
        
        # Step 1: Find the shortest path
        first_path = self.astar(start_grid, goal_grid)
        if first_path is None:
            return [], []
        
        # A = list of confirmed K-shortest paths (grid coords)
        A = [first_path]
        # B = min-heap of candidate paths: (cost, tiebreak, path)
        B = []
        b_counter = 0
        
        for ki in range(1, k):
            prev_path = A[ki - 1]
            
            # Try each node in the previous path as a spur node
            # (skip the last node — can't spur from the goal)
            limit = len(prev_path) - 1
            # For efficiency, only try every other spur node if path is long
            step = 1 if limit <= 30 else 2
            
            for i in range(0, limit, step):
                spur_node = prev_path[i]
                root_path = prev_path[:i + 1]
                root_cost = self.grid_path_cost(root_path)
                
                # Block edges: for every confirmed path in A that shares
                # the same root prefix, block the edge leaving the spur node.
                blocked_edges = set()
                for a_path in A:
                    if len(a_path) > i and a_path[:i + 1] == root_path:
                        # Block the edge from spur_node to the next node in this path
                        if i + 1 < len(a_path):
                            blocked_edges.add((spur_node, a_path[i + 1]))
                
                # Block nodes: all nodes in root_path except the spur node
                blocked_nodes = set(root_path[:-1])
                
                # Find spur path from spur_node to goal
                spur_path = self.astar(spur_node, goal_grid,
                                       blocked_nodes=blocked_nodes,
                                       blocked_edges=blocked_edges)
                
                if spur_path is not None:
                    # Combine: root_path[:-1] + spur_path (spur_path starts at spur_node)
                    candidate = root_path[:-1] + spur_path
                    total_cost = self.grid_path_cost(candidate)
                    
                    # Check this candidate is not a duplicate
                    is_dup = False
                    for existing in A:
                        if existing == candidate:
                            is_dup = True
                            break
                    if not is_dup:
                        b_counter += 1
                        heapq.heappush(B, (total_cost, b_counter, candidate))
            
            if not B:
                # No more alternative paths available
                self.get_logger().info(f"Yen's: only {len(A)} paths exist (requested {k})")
                break
            
            # Pop the lowest-cost candidate as the next shortest path
            _, _, next_path = heapq.heappop(B)
            A.append(next_path)
        
        # Convert all grid paths to world coords, simplify, and compute distances
        paths_world = []
        distances = []
        for grid_path in A:
            world_path = [self.grid_to_world(gx, gy) for gx, gy in grid_path]
            world_path = self.simplify_path(world_path)
            paths_world.append(world_path)
            distances.append(self.path_distance(world_path))

        # Re-sort by EUCLIDEAN distance (shortest metres first).
        # Yen's algorithm orders by costmap-weighted grid cost, so the
        # cost-cheapest path is often a longer detour around inflated zones.
        # We want all_paths[0] to be the geometrically shortest valid path.
        if len(paths_world) > 1:
            sorted_pairs = sorted(zip(paths_world, distances), key=lambda x: x[1])
            paths_world = [p for p, _ in sorted_pairs]
            distances   = [d for _, d in sorted_pairs]

        # Log results
        for idx, d in enumerate(distances):
            self.get_logger().info(f"  Path {idx+1}: {len(paths_world[idx])} waypoints, {d:.2f} m")
        
        return paths_world, distances
    
    def plan_path(self):
        """Plan 3 shortest paths from current position to goal using A*.
        
        Finds up to 3 diverse paths, visualises them in different colours
        in RViz, and follows the shortest one.
        """
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
        
        # Find up to 3 diverse shortest paths
        all_paths, all_distances = self.k_shortest_paths(start_grid, goal_grid, k=3)
        
        if not all_paths:
            self.get_logger().error("A* could not find any path!")
            self.state = "IDLE"
            return
        
        # Store all paths for visualisation
        self.all_paths = all_paths
        self.all_distances = all_distances
        
        # Log all found paths
        for idx, (p, d) in enumerate(zip(all_paths, all_distances)):
            color_names = ["GREEN (shortest)", "BLUE (2nd)", "RED (3rd)"]
            label = color_names[idx] if idx < 3 else f"Path {idx+1}"
            self.get_logger().info(f"  {label}: {len(p)} waypoints, distance = {d:.2f} m")
        
        # Use the SHORTEST path (first after sorting) for navigation
        self.path = all_paths[0]
        
        # Skip the first waypoint if it's very close to current position (starting point)
        if len(self.path) > 1:
            first_wp = self.path[0]
            dist_to_first = math.hypot(first_wp[0] - self.current_x, first_wp[1] - self.current_y)
            if dist_to_first < self.goal_tolerance:
                self.path = self.path[1:]  # Skip the starting point
                self.get_logger().info(f"Skipping start waypoint (distance {dist_to_first:.3f})")
        
        self.current_path_idx = 0
        self.state = "FOLLOWING"
        
        # Publish all paths for visualization
        self.publish_all_paths()
        self.publish_markers()
        
        self.get_logger().info(f"Following shortest path with {len(self.path)} waypoints ({all_distances[0]:.2f} m)")
    
    def line_of_sight(self, x0, y0, x1, y1, clearance_cells=1):
        """Check if there is a clear line between two world points.
        
        Uses Bresenham's line + checking a `clearance_cells` band around
        each cell on the line.  Returns True if the path is obstacle-free.
        clearance_cells=1 → 0.2 m band, matching the rover's inscribed
        radius (0.20 m) so simplified paths are still safe.
        """
        gx0, gy0 = self.world_to_grid(x0, y0)
        gx1, gy1 = self.world_to_grid(x1, y1)
        
        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy
        x, y = gx0, gy0
        
        while True:
            # Check cell and its clearance band
            for cdx in range(-clearance_cells, clearance_cells + 1):
                for cdy in range(-clearance_cells, clearance_cells + 1):
                    cx, cy = x + cdx, y + cdy
                    if self.is_valid_cell(cx, cy):
                        if self.occupancy_grid[cy, cx] >= 80:
                            return False
                    else:
                        return False  # out of bounds
            
            if x == gx1 and y == gy1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return True
    
    def simplify_path(self, path):
        """Line-of-sight path pruning + smoothing.
        
        1. LOS pruning: skip intermediate waypoints whenever a straight
           line to a farther waypoint is obstacle-free.  This removes the
           staircase pattern inherent to grid-based A*.
        2. Corner smoothing: insert midpoints around sharp turns so the
           rover follows gentle curves instead of hard pivots.
        """
        if len(path) <= 2:
            return path
        
        # ── Pass 1: Line-of-sight pruning ──
        pruned = [path[0]]
        i = 0
        while i < len(path) - 1:
            # Try to skip as far ahead as possible
            best = i + 1
            for j in range(len(path) - 1, i + 1, -1):
                if self.line_of_sight(path[i][0], path[i][1],
                                     path[j][0], path[j][1]):
                    best = j
                    break
            pruned.append(path[best])
            i = best
        
        # ── Pass 2: Corner smoothing ──
        if len(pruned) < 3:
            return pruned
        
        smoothed = [pruned[0]]
        for k in range(1, len(pruned) - 1):
            prev = smoothed[-1]
            curr = pruned[k]
            nxt  = pruned[k + 1]
            
            # Angle change at this waypoint
            d1 = math.atan2(curr[1] - prev[1], curr[0] - prev[0])
            d2 = math.atan2(nxt[1]  - curr[1], nxt[0]  - curr[0])
            angle_change = abs(d1 - d2)
            if angle_change > math.pi:
                angle_change = 2 * math.pi - angle_change
            
            if angle_change > 0.5:  # ~29° — insert approach/depart points
                # Insert a point 0.3 m before the corner (approach)
                dist_in = math.hypot(curr[0] - prev[0], curr[1] - prev[1])
                if dist_in > 0.5:
                    ratio = 0.3 / dist_in
                    ax = curr[0] + (prev[0] - curr[0]) * ratio
                    ay = curr[1] + (prev[1] - curr[1]) * ratio
                    smoothed.append((ax, ay))
                
                smoothed.append(curr)
                
                # Insert a point 0.3 m after the corner (depart)
                dist_out = math.hypot(nxt[0] - curr[0], nxt[1] - curr[1])
                if dist_out > 0.5:
                    ratio = 0.3 / dist_out
                    bx = curr[0] + (nxt[0] - curr[0]) * ratio
                    by = curr[1] + (nxt[1] - curr[1]) * ratio
                    smoothed.append((bx, by))
            else:
                smoothed.append(curr)
        
        smoothed.append(pruned[-1])
        return smoothed
    
    def publish_all_paths(self):
        """Publish all candidate paths for RViz visualization.
        
        Path 1 (shortest) → /planned_path
        Path 2             → /planned_path_2  
        Path 3             → /planned_path_3
        """
        publishers = [self.path_pub, self.path2_pub, self.path3_pub]
        
        for idx, pub in enumerate(publishers):
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            if idx < len(self.all_paths):
                for wx, wy in self.all_paths[idx]:
                    pose = PoseStamped()
                    pose.header = path_msg.header
                    pose.pose.position.x = wx
                    pose.pose.position.y = wy
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0
                    path_msg.poses.append(pose)
            
            pub.publish(path_msg)
    
    def publish_markers(self):
        """Publish path markers for RViz visualization.
        
        Shows up to 3 paths in different colours with distance labels:
            Path 1 (shortest) — GREEN   (thick)
            Path 2             — BLUE    (medium)
            Path 3             — RED     (medium)
        Each path gets a TEXT marker showing its distance in metres.
        """
        markers = MarkerArray()
        
        # First, delete all old markers to avoid ghost paths
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)
        
        # Colours for each path: (R, G, B)
        path_colours = [
            (0.0, 1.0, 0.0),   # GREEN  — shortest
            (0.0, 0.4, 1.0),   # BLUE   — 2nd
            (1.0, 0.0, 0.0),   # RED    — 3rd
        ]
        path_labels = ["Path 1 (shortest)", "Path 2", "Path 3"]
        path_widths = [0.12, 0.08, 0.08]   # shortest path is thicker
        z_offsets   = [0.15, 0.12, 0.09]   # stagger height to avoid z-fighting
        
        marker_id = 0
        
        for idx in range(len(self.all_paths)):
            path = self.all_paths[idx]
            dist = self.all_distances[idx] if idx < len(self.all_distances) else 0.0
            r, g, b = path_colours[idx] if idx < 3 else (0.5, 0.5, 0.5)
            width = path_widths[idx] if idx < 3 else 0.06
            z_off = z_offsets[idx] if idx < 3 else 0.05
            label = path_labels[idx] if idx < 3 else f"Path {idx+1}"
            
            # ── Line strip for the path ──
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = f"path_{idx+1}"
            line_marker.id = marker_id
            marker_id += 1
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = width
            line_marker.color.r = r
            line_marker.color.g = g
            line_marker.color.b = b
            line_marker.color.a = 0.9 if idx == 0 else 0.6  # shortest is more opaque
            line_marker.lifetime.sec = 0  # persist until replaced
            
            for wx, wy in path:
                p = Point()
                p.x = wx
                p.y = wy
                p.z = z_off
                line_marker.points.append(p)
            
            markers.markers.append(line_marker)
            
            # ── Distance label (TEXT) at the midpoint of the path ──
            if len(path) >= 2:
                mid_idx = len(path) // 2
                mid_x, mid_y = path[mid_idx]
                
                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.ns = f"path_{idx+1}_label"
                text_marker.id = marker_id
                marker_id += 1
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                # Offset text slightly above and beside the path
                text_marker.pose.position.x = mid_x + 0.3 * (idx + 1)
                text_marker.pose.position.y = mid_y + 0.3 * (idx + 1)
                text_marker.pose.position.z = 1.0 + 0.3 * idx
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.5   # text height
                text_marker.color.r = r
                text_marker.color.g = g
                text_marker.color.b = b
                text_marker.color.a = 1.0
                text_marker.text = f"{label}: {dist:.2f} m"
                text_marker.lifetime.sec = 0
                
                markers.markers.append(text_marker)
        
        # ── Goal marker (red sphere) ──
        if self.goal_x is not None:
            goal_marker = Marker()
            goal_marker.header.frame_id = "map"
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = "goal"
            goal_marker.id = marker_id
            marker_id += 1
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
        
        # ── Start marker (blue sphere) ──
        start_marker = Marker()
        start_marker.header.frame_id = "map"
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.ns = "start"
        start_marker.id = marker_id
        marker_id += 1
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
        """Publish costmap for visualization — origin matches the static /map."""
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = "map"
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.info.resolution = self.grid_resolution
        grid_msg.info.width  = self.grid_width
        grid_msg.info.height = self.grid_height
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
                # Also wait for the static world map so A* plans on real walls.
                # map_publisher is TRANSIENT_LOCAL so it arrives quickly; after
                # 30 s (300 ticks) we start anyway to avoid hanging forever.
                map_ready = (self.static_map_info is not None) or (self.init_timer_count > 300)
                if map_ready and self.init_timer_count > 50:  # 5 s at 10 Hz
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
            
            # ── Obstacle avoidance (LiDAR + human-aware) ────────────────────
            # Decrement replan cooldowns every cycle.
            if self.replan_cooldown > 0:
                self.replan_cooldown -= 1
            if self.human_replan_cooldown > 0:
                self.human_replan_cooldown -= 1

            # Reset speed factor each cycle (threats will reduce it below)
            self.human_speed_factor = 1.0

            # ── Human-aware response (diversion / slow-down / replan) ──
            if self.human_replan_cooldown == 0 and self.detected_humans:
                threats = self._analyze_human_threats()
                for threat in threats:
                    action = threat.get('action', 'ignore')

                    if action == 'ignore':
                        continue

                    elif action == 'slow_down':
                        # Reduce speed proportionally to proximity
                        factor = max(0.15, threat['robot_dist'] / self.human_replan_dist)
                        self.human_speed_factor = min(self.human_speed_factor, factor)
                        self.get_logger().info(
                            f"Slowing for human {threat['idx']} "
                            f"(dist={threat['robot_dist']:.1f}m, "
                            f"speed×{factor:.2f}, "
                            f"v=({threat['vx']:.1f},{threat['vy']:.1f}))",
                            throttle_duration_sec=2.0)

                    elif action == 'divert':
                        side = threat.get('divert_side', 'left')
                        self.get_logger().warn(
                            f"Diverting {side} around human {threat['idx']} "
                            f"(speed={threat['speed']:.1f}m/s, "
                            f"dir=({threat['vx']:.1f},{threat['vy']:.1f}))")
                        ok = self._local_diversion(threat)
                        if ok:
                            self.human_replan_cooldown = 30  # 3 s cooldown
                            self.publish_all_paths()
                            self.publish_markers()
                            # Also slow down while executing the diversion
                            self.human_speed_factor = min(
                                self.human_speed_factor, 0.6)
                        else:
                            # Diversion blocked (wall) → full A* replan
                            self.get_logger().warn(
                                "Local diversion blocked \u2192 full replan")
                            self.state = "PLANNING"
                            self.plan_path()
                            self.human_replan_cooldown = 50
                            self.cmd_vel_pub.publish(cmd)
                            return

            if self.scan_data is not None and self.replan_cooldown == 0:
                # 1. Forward-arc proximity check: emergency replan when anything
                #    is closer than 0.35 m in the ±30° cone ahead.
                obstacle_close = False
                close_count = 0
                for i, r in enumerate(self.scan_data.ranges):
                    if self.scan_data.range_min < r < 0.55:
                        angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                        if abs(angle) < 0.52:  # ±30°
                            close_count += 1
                            if close_count >= 3:  # need 3+ rays, not just 1
                                obstacle_close = True
                                break

                # 2. Path-ahead check: only replan if >=2 of the next 5
                #    waypoints are in lethal cells (cost >= 99).
                #    Gradient-inflated cells (1-98) do NOT trigger replanning.
                path_blocked = False
                if not obstacle_close:
                    blocked_count = 0
                    for wp_idx in range(self.current_path_idx,
                                       min(self.current_path_idx + 5, len(self.path))):
                        wp_x, wp_y = self.path[wp_idx]
                        gx, gy = self.world_to_grid(wp_x, wp_y)
                        if self.is_valid_cell(gx, gy) and self.occupancy_grid[gy, gx] >= 99:
                            blocked_count += 1
                    if blocked_count >= 2:
                        path_blocked = True

                if obstacle_close or path_blocked:
                    reason = "obstacle in forward arc" if obstacle_close else "path waypoints blocked"
                    self.get_logger().warn(f"Replanning: {reason}")
                    self.state = "PLANNING"
                    self.plan_path()  # replan with updated costmap
                    self.replan_cooldown = 25  # ~2.5 s before next replan check
                    self.cmd_vel_pub.publish(cmd)
                    return
            # ── End obstacle avoidance ──────────────────────────────────────
            
            # ── Two-Phase Path Controller ───────────────────────────────────
            cross_track_error, path_heading = self._compute_cross_track_error()

            heading_error = path_heading - self.current_yaw
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi

            # ── Phase 1: Large heading error → rotate in place first ────────
            # When heading error > 57° the robot would otherwise spin in tiny
            # circles (v/w = 0.08/0.5 = 0.16 m radius).  Stop and rotate
            # cleanly instead; Stanley resumes once heading is aligned.
            if abs(heading_error) > 1.0:
                cmd.linear.x  = 0.0
                cmd.angular.z = math.copysign(
                    min(self.angular_speed, 0.7 * abs(heading_error)),
                    heading_error)
                if self.progress_counter % 20 == 0:
                    self.get_logger().info(
                        f"ROTATE: h_err={heading_error:.2f}, "
                        f"ct={cross_track_error:.2f}, "
                        f"w={cmd.angular.z:.2f}")
                self.cmd_vel_pub.publish(cmd)
                return

            # ── Phase 2: Heading aligned → Stanley path following ───────────
            # cross_track_error: +ve = robot LEFT  of path → steer RIGHT → negate
            #                    -ve = robot RIGHT of path → steer LEFT  → negate
            stanley_k = 4.0
            softening_vel = 0.2
            cross_track_term = math.atan2(stanley_k * (-cross_track_error), softening_vel)
            stanley_steer = heading_error + cross_track_term
            stanley_steer = max(-self.angular_speed, min(self.angular_speed, stanley_steer))

            # Speed: slow down proportionally to errors, always keep minimum
            heading_factor = max(0.1, 1.0 - abs(heading_error) / math.pi)
            ct_factor      = max(0.2, 1.0 - abs(cross_track_error) / 1.0)

            # Curvature factor
            curvature = 0.0
            if self.current_path_idx + 1 < len(self.path):
                next_x, next_y = self.path[self.current_path_idx + 1]
                ahead_angle = math.atan2(next_y - target_y, next_x - target_x)
                curvature = abs(ahead_angle - path_heading)
                if curvature > math.pi:
                    curvature = 2 * math.pi - curvature
            curvature_factor = max(0.4, 1.0 - curvature * 0.5)
            approach_factor  = min(1.0, distance / 0.4)

            speed = (self.linear_speed * heading_factor * ct_factor *
                     curvature_factor * approach_factor * self.human_speed_factor)
            speed = max(0.08, min(self.linear_speed * 0.9, speed))

            cmd.linear.x  = speed
            cmd.angular.z = stanley_steer

            # Debug log
            if self.progress_counter % 40 == 0:
                self.get_logger().info(
                    f"CTRL: v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}, "
                    f"h_err={heading_error:.2f}, ct={cross_track_error:.2f}"
                )
        
        self.cmd_vel_pub.publish(cmd)


def main():
    rclpy.init()
    node = AStarPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

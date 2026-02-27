#!/usr/bin/env python3
"""
Point Cloud Mapper Node
Converts LiDAR scans to point clouds and builds an accumulated map
Also generates an occupancy grid map from the point cloud
Publishes TF transforms for the robot
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import struct
import numpy as np


class PointCloudMapper(Node):
    def __init__(self):
        super().__init__("pointcloud_mapper")
        
        # Parameters
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("max_points", 20000)
        self.declare_parameter("map_resolution", 0.1)  # 10cm per cell
        self.declare_parameter("map_size", 28.0)  # 28m x 28m map (matches static map)
        # Robot spawn position in world/map frame.
        # The DiffDrive odom starts at (0,0) where the robot spawned, so
        # map→odom translation = (spawn_x, spawn_y) to align the frames.
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", -8.0)  # rover spawns at y=-8 in large_messy_room
        self.declare_parameter("spawn_yaw", 1.5708)  # rover faces +Y at spawn
        self.declare_parameter("max_scan_range", 8.0)  # limit usable LiDAR range (m)
        
        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.max_points = self.get_parameter("max_points").value
        self.map_resolution = self.get_parameter("map_resolution").value
        self.map_size = self.get_parameter("map_size").value
        self.spawn_x = self.get_parameter("spawn_x").value
        self.spawn_y = self.get_parameter("spawn_y").value
        self.spawn_yaw = self.get_parameter("spawn_yaw").value
        self.max_scan_range = self.get_parameter("max_scan_range").value
        # Precompute rotation constants for odom→world conversion
        self.spawn_cos = math.cos(self.spawn_yaw)
        self.spawn_sin = math.sin(self.spawn_yaw)
        
        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.accumulated_points = []
        self.last_scan_points = []
        # Odom history for scan-odom time synchronization
        self.odom_history = []  # [(stamp_sec, x, y, yaw), ...]
        self.max_odom_history = 100  # ~2 seconds at 50Hz
        
        # Occupancy grid
        self.grid_width = int(self.map_size / self.map_resolution)
        self.grid_height = int(self.map_size / self.map_resolution)
        self.occupancy_grid = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)  # -1 = unknown
        self.hit_count = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)
        self.miss_count = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)
        
        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish static transforms for robot structure
        self.publish_static_transforms()
        
        # Publishers
        self.scan_cloud_pub = self.create_publisher(
            PointCloud2, "/scan_pointcloud", 10
        )
        self.map_cloud_pub = self.create_publisher(
            PointCloud2, "/map_pointcloud", 10
        )
        # Publish to /pointcloud_map so it does NOT overwrite the static
        # world map on /map — both layers are displayed in RViz simultaneously.
        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGrid, "/pointcloud_map", 10
        )
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        # Use the same /odom topic that the launch file bridges from Gazebo.
        # (The DiffDrive plugin publishes on gz 'odom'; the bridge maps that
        # to ROS2 /odom.  The old topic /model/turtlebot3_rover/odometry was
        # never bridged, so robot_x/robot_y stayed 0 → TF always at origin.)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        
        # Timer for publishing map and TF
        self.map_timer = self.create_timer(1.0, self.publish_map)
        self.tf_timer = self.create_timer(0.05, self.publish_tf)
        self.grid_timer = self.create_timer(2.0, self.publish_occupancy_grid)
        
        self.get_logger().info("Point Cloud Mapper started - publishing sensor map to /pointcloud_map")
    
    def publish_static_transforms(self):
        """Publish static transforms for robot structure"""
        transforms = []
        now = self.get_clock().now().to_msg()
        
        # map -> odom: odom frame origin is at the robot's spawn position in world,
        # AND the odom frame is rotated by spawn_yaw relative to the map frame.
        # The DiffDrive plugin always starts odom at (0,0,yaw=0) regardless of the
        # model's world pose, so map→odom must encode BOTH the translation AND the
        # rotation so that chaining map→odom→base_footprint gives the correct world
        # pose.
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = "map"
        t_map_odom.child_frame_id = "odom"
        t_map_odom.transform.translation.x = self.spawn_x
        t_map_odom.transform.translation.y = self.spawn_y
        t_map_odom.transform.translation.z = 0.0
        # Quaternion for spawn_yaw rotation about Z axis
        t_map_odom.transform.rotation.z = math.sin(self.spawn_yaw / 2.0)
        t_map_odom.transform.rotation.w = math.cos(self.spawn_yaw / 2.0)
        transforms.append(t_map_odom)
        
        # base_footprint -> base_link
        t_base = TransformStamped()
        t_base.header.stamp = now
        t_base.header.frame_id = "base_footprint"
        t_base.child_frame_id = "base_link"
        t_base.transform.translation.z = 0.08
        t_base.transform.rotation.w = 1.0
        transforms.append(t_base)
        
        # base_link -> lidar_link
        t_lidar = TransformStamped()
        t_lidar.header.stamp = now
        t_lidar.header.frame_id = "base_link"
        t_lidar.child_frame_id = "lidar_link"
        t_lidar.transform.translation.z = 0.10
        t_lidar.transform.rotation.w = 1.0
        transforms.append(t_lidar)
        
        # base_link -> camera_link
        t_camera = TransformStamped()
        t_camera.header.stamp = now
        t_camera.header.frame_id = "base_link"
        t_camera.child_frame_id = "camera_link"
        t_camera.transform.translation.z = 0.14
        t_camera.transform.rotation.w = 1.0
        transforms.append(t_camera)
        
        self.static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info("Published static transforms")
    
    def publish_tf(self):
        """Publish dynamic TF: odom -> base_footprint"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0
        
        # Convert yaw to quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.robot_yaw / 2.0)
        t.transform.rotation.w = math.cos(self.robot_yaw / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
    
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Store timestamped odom for scan synchronization
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_history.append((stamp_sec, self.robot_x, self.robot_y, self.robot_yaw))
        if len(self.odom_history) > self.max_odom_history:
            self.odom_history.pop(0)
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid cell indices"""
        gx = int((x + self.map_size / 2) / self.map_resolution)
        gy = int((y + self.map_size / 2) / self.map_resolution)
        return gx, gy
    
    def is_valid_grid(self, gx, gy):
        """Check if grid coordinates are valid"""
        return 0 <= gx < self.grid_width and 0 <= gy < self.grid_height
    
    def update_occupancy_grid(self, robot_x, robot_y, hit_x, hit_y):
        """Update occupancy grid with ray tracing"""
        # Mark cells along the ray as free (miss)
        gx0, gy0 = self.world_to_grid(robot_x, robot_y)
        gx1, gy1 = self.world_to_grid(hit_x, hit_y)
        
        # Bresenham's line algorithm for ray tracing
        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy
        
        x, y = gx0, gy0
        while True:
            if self.is_valid_grid(x, y):
                if x == gx1 and y == gy1:
                    # Hit point - mark as occupied
                    self.hit_count[y, x] += 1
                else:
                    # Free space along ray — also erode stale hits so
                    # ghost walls from odom drift are corrected quickly.
                    self.miss_count[y, x] += 1
                    if self.hit_count[y, x] > 0:
                        self.hit_count[y, x] -= 1
            
            if x == gx1 and y == gy1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def scan_callback(self, msg):
        # Convert scan to points in robot frame, then transform to map frame
        points_robot = []
        points_map = []
        
        # Find odom closest to the scan timestamp for accurate transform.
        # Without this, the robot may have moved/turned between the odom
        # callback and the scan callback, causing a small but cumulative
        # placement error in the occupancy grid.
        scan_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ox, oy, oyaw = self.robot_x, self.robot_y, self.robot_yaw
        if self.odom_history:
            best = min(self.odom_history, key=lambda t: abs(t[0] - scan_stamp))
            _, ox, oy, oyaw = best
        
        angle = msg.angle_min
        # World yaw = odom yaw + spawn yaw (odom starts at yaw=0 at spawn)
        world_yaw = oyaw + self.spawn_yaw
        cos_yaw = math.cos(world_yaw)
        sin_yaw = math.sin(world_yaw)
        
        # World position = rotate odom position by spawn_yaw, then translate
        world_x = self.spawn_x + ox * self.spawn_cos - oy * self.spawn_sin
        world_y = self.spawn_y + ox * self.spawn_sin + oy * self.spawn_cos

        usable_max = min(msg.range_max, self.max_scan_range)
        for r in msg.ranges:
            if msg.range_min < r < usable_max:
                # Point in robot frame
                x_robot = r * math.cos(angle)
                y_robot = r * math.sin(angle)
                z_robot = 0.18  # LiDAR height
                
                points_robot.append((x_robot, y_robot, z_robot))
                
                # Transform to map/world frame using rotated world position & yaw
                x_map = world_x + x_robot * cos_yaw - y_robot * sin_yaw
                y_map = world_y + x_robot * sin_yaw + y_robot * cos_yaw
                z_map = z_robot
                
                points_map.append((x_map, y_map, z_map))
                
                # Update occupancy grid using world-frame coordinates
                self.update_occupancy_grid(world_x, world_y, x_map, y_map)
            
            angle += msg.angle_increment
        
        # Publish current scan as point cloud
        scan_cloud = self.create_pointcloud2(points_robot, "lidar_link", msg.header.stamp)
        self.scan_cloud_pub.publish(scan_cloud)
        
        # Add to accumulated map (subsample to avoid too many points)
        if len(points_map) > 0:
            step = max(1, len(points_map) // 36)  # Keep ~36 points per scan
            self.accumulated_points.extend(points_map[::step])
            
            # Limit total points
            if len(self.accumulated_points) > self.max_points:
                self.accumulated_points = self.accumulated_points[-self.max_points:]
        
        self.last_scan_points = points_robot
    
    def publish_map(self):
        if self.accumulated_points:
            map_cloud = self.create_pointcloud2(
                self.accumulated_points,
                self.map_frame,
                self.get_clock().now().to_msg()
            )
            self.map_cloud_pub.publish(map_cloud)
            self.get_logger().info(
                f"Map has {len(self.accumulated_points)} points",
                throttle_duration_sec=5.0
            )
    
    def publish_occupancy_grid(self):
        """Publish occupancy grid map"""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        
        msg.info.resolution = self.map_resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height
        msg.info.origin.position.x = -self.map_size / 2
        msg.info.origin.position.y = -self.map_size / 2
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Apply temporal decay so stale observations from potentially
        # drifted odometry positions gradually fade out.  Fresh scans at
        # the current (locally accurate) position will dominate.
        # Decay 0.88 every 2 s → after 10 s old data ≈ 53%, after 20 s ≈ 28%.
        decay = 0.88
        self.hit_count = (self.hit_count.astype(np.float64) * decay).astype(np.int32)
        self.miss_count = (self.miss_count.astype(np.float64) * decay).astype(np.int32)
        
        # Calculate occupancy probabilities
        grid_data = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)
        
        total = self.hit_count + self.miss_count
        # Require at least 3 observations to classify a cell — prevents
        # noise/single-hit artefacts from odom jitter.
        mask = total > 2
        
        # Cells with observations
        with np.errstate(divide='ignore', invalid='ignore'):
            prob = np.where(mask, (self.hit_count / total) * 100, -1)
        
        grid_data[mask] = np.clip(prob[mask], 0, 100).astype(np.int8)
        
        # Threshold: >65% hit = occupied (100), <35% hit = free (0)
        grid_data[prob > 65] = 100
        grid_data[(prob >= 0) & (prob < 35)] = 0
        
        msg.data = grid_data.flatten().tolist()
        self.occupancy_grid_pub.publish(msg)
        
        occupied_cells = np.sum(grid_data == 100)
        free_cells = np.sum(grid_data == 0)
        self.get_logger().info(
            f"[/pointcloud_map] {occupied_cells} occupied, {free_cells} free cells",
            throttle_duration_sec=10.0
        )
    
    def create_pointcloud2(self, points, frame_id, stamp):
        """Create a PointCloud2 message from a list of (x, y, z) points"""
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        
        msg.height = 1
        msg.width = len(points)
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        
        # Pack point data
        data = []
        for x, y, z in points:
            data.append(struct.pack('fff', x, y, z))
        msg.data = b''.join(data)
        
        return msg


def main():
    rclpy.init()
    node = PointCloudMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

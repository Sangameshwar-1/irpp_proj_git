#!/usr/bin/env python3
"""
Localization Node
=================
Provides a corrected robot pose in the map frame by fusing odometry with
the known spawn offset and filtering odom jumps.

Current implementation (v1):
  - Applies the spawn rotation + translation to raw DiffDrive odometry
  - Filters sudden odom jumps (collision-induced wheel slip)
  - Publishes /robot_pose as PoseStamped at odometry rate

Future improvements (TODO):
  - Scan-to-map matching for drift correction (ICP or correlative)
  - Extended Kalman Filter (or use nav2_amcl / robot_localization)
  - Loop closure detection

The key benefit right now: all other nodes (global_planner, local_planner,
human_detector_red) subscribe to /robot_pose instead of each computing
their own odom→world transform.  When we improve localization later,
the interface stays the same.

Topics
------
  Subscribes:
    /odom  (nav_msgs/Odometry) – raw DiffDrive odometry from Gazebo
    /scan  (sensor_msgs/LaserScan) – for future scan matching
    /map   (nav_msgs/OccupancyGrid) – for future scan-to-map correction

  Publishes:
    /robot_pose (geometry_msgs/PoseStamped) – corrected pose in map frame
"""

import math
import rclpy
import rclpy.qos
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("localization_node")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", -8.0)
        self.declare_parameter("spawn_yaw", 1.5708)  # 90° → rover faces +Y
        # Jump filter thresholds
        self.declare_parameter("max_velocity", 0.6)   # m/s (robot max ~0.3)
        self.declare_parameter("max_yaw_rate", 2.5)   # rad/s

        self.spawn_x = self.get_parameter("spawn_x").value
        self.spawn_y = self.get_parameter("spawn_y").value
        self.spawn_yaw = self.get_parameter("spawn_yaw").value
        self.max_vel = self.get_parameter("max_velocity").value
        self.max_yr = self.get_parameter("max_yaw_rate").value

        # Precompute rotation constants
        self.cos_s = math.cos(self.spawn_yaw)
        self.sin_s = math.sin(self.spawn_yaw)

        # ── State ─────────────────────────────────────────────────────
        # Filtered pose in map frame
        self.map_x = self.spawn_x
        self.map_y = self.spawn_y
        self.map_yaw = self.spawn_yaw

        # Previous odom for jump filtering
        self.prev_ox = None
        self.prev_oy = None
        self.prev_oyaw = None
        self.prev_stamp = None
        self.jump_count = 0

        # Future: reference map for scan matching
        self.ref_map = None

        # ── Publisher ─────────────────────────────────────────────────
        self.pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10)

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        # Subscribed but not yet used — infrastructure for future scan matching
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(
            OccupancyGrid, "/map", self.map_cb,
            rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))

        self.get_logger().info(
            f"Localization started: spawn=({self.spawn_x}, {self.spawn_y}, "
            f"yaw={math.degrees(self.spawn_yaw):.1f}°)")

    # ── Odom → map-frame pose ─────────────────────────────────────────

    def odom_cb(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        oyaw = math.atan2(siny, cosy)

        # ── Jump filter ───────────────────────────────────────────
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_stamp is not None:
            dt = stamp - self.prev_stamp
            if dt > 0.001:
                vel = math.hypot(ox - self.prev_ox,
                                 oy - self.prev_oy) / dt
                dyaw = abs(oyaw - self.prev_oyaw)
                if dyaw > math.pi:
                    dyaw = 2 * math.pi - dyaw
                yaw_rate = dyaw / dt
                if vel > self.max_vel or yaw_rate > self.max_yr:
                    self.jump_count += 1
                    if self.jump_count <= 3:
                        self.get_logger().warn(
                            f"Odom jump filtered (v={vel:.2f}, "
                            f"ω={yaw_rate:.2f})",
                            throttle_duration_sec=2.0)
                    # Skip this odom reading
                    self.prev_stamp = stamp
                    return

        self.prev_ox = ox
        self.prev_oy = oy
        self.prev_oyaw = oyaw
        self.prev_stamp = stamp

        # ── Apply spawn transform ─────────────────────────────────
        # world = R(spawn_yaw) · odom + (spawn_x, spawn_y)
        self.map_x = self.spawn_x + ox * self.cos_s - oy * self.sin_s
        self.map_y = self.spawn_y + ox * self.sin_s + oy * self.cos_s
        self.map_yaw = oyaw + self.spawn_yaw

        # ── Publish ───────────────────────────────────────────────
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = msg.header.stamp
        pose.pose.position.x = self.map_x
        pose.pose.position.y = self.map_y
        pose.pose.position.z = 0.0
        # Yaw → quaternion
        half = self.map_yaw / 2.0
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)
        self.pose_pub.publish(pose)

    # ── Placeholders for future scan-matching correction ──────────────

    def scan_cb(self, msg):
        """Reserved for future scan-to-map matching."""
        pass  # TODO: ICP or correlative scan matching against self.ref_map

    def map_cb(self, msg):
        """Store reference map for future scan matching."""
        if self.ref_map is None:
            self.ref_map = msg
            self.get_logger().info(
                f"Reference map stored for future scan matching "
                f"({msg.info.width}×{msg.info.height})")


def main():
    rclpy.init()
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

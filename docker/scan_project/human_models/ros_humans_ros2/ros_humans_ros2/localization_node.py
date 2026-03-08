#!/usr/bin/env python3
"""
Localization Node — Scan-Matched Odometry
==========================================
Corrects raw DiffDrive odometry drift by matching LiDAR scans against
the known static map.

Algorithm
---------
1. Raw odom gives a **fast** pose update every ~20 ms (robot moves smoothly).
2. Every ``correction_interval`` seconds (~0.5 s) a **correlative scan
   matcher** searches a small window around the odom-predicted pose and
   finds the (dx, dy, dθ) offset that maximises agreement between the
   live LiDAR scan and the static map.
3. The correction offset is EMA-filtered so one bad match can't teleport
   the robot.
4. The filtered offset is added to every subsequent odom→map transform.

The result is a pose that tracks odom's responsiveness while gradually
correcting the long-term drift that pure odom accumulates.

Topics
------
  Subscribes:
    /odom  (nav_msgs/Odometry)       – raw DiffDrive from Gazebo
    /scan  (sensor_msgs/LaserScan)   – LiDAR ring
    /map   (nav_msgs/OccupancyGrid)  – static world map (TRANSIENT_LOCAL)

  Publishes:
    /robot_pose       (geometry_msgs/PoseStamped) – corrected pose in map frame
    /robot_pose_gt    (geometry_msgs/PoseStamped) – raw odom-only pose (for comparison)
"""

import math
import numpy as np
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
        self.declare_parameter("spawn_yaw", 1.5708)
        # Jump filter
        self.declare_parameter("max_velocity", 0.6)
        self.declare_parameter("max_yaw_rate", 2.5)
        # Scan matcher
        self.declare_parameter("correction_interval", 0.5)   # seconds
        self.declare_parameter("search_xy", 0.6)              # metres half-window
        self.declare_parameter("search_yaw", 0.15)            # radians half-window
        self.declare_parameter("xy_step", 0.05)               # metres
        self.declare_parameter("yaw_step", 0.02)              # radians
        self.declare_parameter("correction_alpha", 0.4)       # EMA blend
        self.declare_parameter("min_scan_points", 40)         # need at least N good rays
        self.declare_parameter("max_scan_range", 6.0)         # ignore rays beyond this

        self.spawn_x   = self.get_parameter("spawn_x").value
        self.spawn_y   = self.get_parameter("spawn_y").value
        self.spawn_yaw = self.get_parameter("spawn_yaw").value
        self.max_vel   = self.get_parameter("max_velocity").value
        self.max_yr    = self.get_parameter("max_yaw_rate").value
        self.corr_interval = self.get_parameter("correction_interval").value
        self.search_xy     = self.get_parameter("search_xy").value
        self.search_yaw    = self.get_parameter("search_yaw").value
        self.xy_step       = self.get_parameter("xy_step").value
        self.yaw_step      = self.get_parameter("yaw_step").value
        self.corr_alpha    = self.get_parameter("correction_alpha").value
        self.min_scan_pts  = self.get_parameter("min_scan_points").value
        self.max_scan_range = self.get_parameter("max_scan_range").value

        self.cos_s = math.cos(self.spawn_yaw)
        self.sin_s = math.sin(self.spawn_yaw)

        # ── State ─────────────────────────────────────────────────────
        # Raw odom → map (before correction)
        self.raw_map_x   = self.spawn_x
        self.raw_map_y   = self.spawn_y
        self.raw_map_yaw = self.spawn_yaw

        # Accumulated correction offset (added to raw odom→map)
        self.corr_dx   = 0.0
        self.corr_dy   = 0.0
        self.corr_dyaw = 0.0

        # Corrected pose (published)
        self.map_x   = self.spawn_x
        self.map_y   = self.spawn_y
        self.map_yaw = self.spawn_yaw

        # Odom jump filter
        self.prev_ox = None
        self.prev_oy = None
        self.prev_oyaw = None
        self.prev_stamp = None
        self.jump_count = 0

        # Scan matcher state
        self.latest_scan = None
        self.last_correction_time = 0.0
        self.ref_map = None          # OccupancyGrid message
        self.map_grid = None         # np array (H, W) of 0/1 (1=occupied)
        self.map_info = None         # MapMetaData
        self.correction_count = 0
        self.bad_match_streak = 0    # consecutive low-score matches

        # ── Publishers ────────────────────────────────────────────────
        self.pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10)
        self.gt_pub   = self.create_publisher(PoseStamped, "/robot_pose_gt", 10)

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb,
                                 rclpy.qos.QoSProfile(
                                     depth=5,
                                     reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            OccupancyGrid, "/map", self.map_cb,
            rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))

        self.get_logger().info(
            f"Localization started (scan-matched): spawn=({self.spawn_x}, "
            f"{self.spawn_y}, yaw={math.degrees(self.spawn_yaw):.1f}°), "
            f"correction every {self.corr_interval}s, "
            f"search ±{self.search_xy}m / ±{math.degrees(self.search_yaw):.1f}°")

    # ══════════════════════════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════════════════════════

    def odom_cb(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        oyaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # ── Jump filter ──────────────────────────────────────────
        if self.prev_stamp is not None:
            dt = stamp - self.prev_stamp
            if dt > 0.001:
                vel = math.hypot(ox - self.prev_ox, oy - self.prev_oy) / dt
                dyaw = abs(oyaw - self.prev_oyaw)
                if dyaw > math.pi:
                    dyaw = 2 * math.pi - dyaw
                yaw_rate = dyaw / dt
                if vel > self.max_vel or yaw_rate > self.max_yr:
                    self.jump_count += 1
                    self.get_logger().warn(
                        f"Odom jump filtered (v={vel:.2f}, ω={yaw_rate:.2f})",
                        throttle_duration_sec=2.0)
                    self.prev_stamp = stamp
                    return

        self.prev_ox = ox
        self.prev_oy = oy
        self.prev_oyaw = oyaw
        self.prev_stamp = stamp

        # ── Raw odom→map (no correction) ─────────────────────────
        self.raw_map_x   = self.spawn_x + ox * self.cos_s - oy * self.sin_s
        self.raw_map_y   = self.spawn_y + ox * self.sin_s + oy * self.cos_s
        self.raw_map_yaw = oyaw + self.spawn_yaw

        # ── Apply accumulated correction ─────────────────────────
        self.map_x   = self.raw_map_x   + self.corr_dx
        self.map_y   = self.raw_map_y   + self.corr_dy
        self.map_yaw = self.raw_map_yaw + self.corr_dyaw
        # Normalise yaw
        self.map_yaw = math.atan2(math.sin(self.map_yaw),
                                   math.cos(self.map_yaw))

        # ── Publish corrected pose ───────────────────────────────
        self._publish_pose(self.pose_pub, self.map_x, self.map_y,
                           self.map_yaw, msg.header.stamp)

        # ── Publish raw (GT-like) pose for comparison ────────────
        self._publish_pose(self.gt_pub, self.raw_map_x, self.raw_map_y,
                           self.raw_map_yaw, msg.header.stamp)

        # ── Trigger scan-match correction periodically ───────────
        if (stamp - self.last_correction_time >= self.corr_interval
                and self.map_grid is not None
                and self.latest_scan is not None):
            self.last_correction_time = stamp
            self._scan_match_correction()

    def scan_cb(self, msg):
        self.latest_scan = msg

    def map_cb(self, msg):
        if self.ref_map is None:
            self.ref_map = msg
            info = msg.info
            raw = np.array(msg.data, dtype=np.int8).reshape(
                (info.height, info.width))
            # Binary grid: 1 where occupied (value >= 50), 0 elsewhere
            self.map_grid = (raw >= 50).astype(np.int8)
            self.map_info = info
            self.get_logger().info(
                f"Reference map stored ({info.width}×{info.height}, "
                f"res={info.resolution}m)")

    # ══════════════════════════════════════════════════════════════════
    # Correlative scan matcher
    # ══════════════════════════════════════════════════════════════════

    def _scan_match_correction(self):
        """Find the (dx, dy, dθ) that best aligns the live scan with the
        static map, then EMA-blend it into the accumulated correction."""
        scan = self.latest_scan
        info = self.map_info

        # ── Extract scan endpoints in robot-local frame ──────────
        n_rays = len(scan.ranges)
        angles = np.linspace(scan.angle_min,
                             scan.angle_min + (n_rays - 1) * scan.angle_increment,
                             n_rays)
        ranges = np.array(scan.ranges, dtype=np.float64)

        valid = (ranges > scan.range_min) & (ranges < min(scan.range_max, self.max_scan_range))
        if np.sum(valid) < self.min_scan_pts:
            return

        angles = angles[valid]
        ranges = ranges[valid]

        # Subsample to at most 120 rays for speed
        if len(angles) > 120:
            idx = np.linspace(0, len(angles) - 1, 120, dtype=int)
            angles = angles[idx]
            ranges = ranges[idx]

        # Local (x, y) relative to robot
        lx = ranges * np.cos(angles)
        ly = ranges * np.sin(angles)

        # ── Search grid around current corrected pose ────────────
        base_x   = self.map_x
        base_y   = self.map_y
        base_yaw = self.map_yaw

        best_score = -1
        best_dx = 0.0
        best_dy = 0.0
        best_dyaw = 0.0

        # Pre-compute grid constants
        map_ox = info.origin.position.x
        map_oy = info.origin.position.y
        res = info.resolution
        gw = info.width
        gh = info.height

        n_xy = int(self.search_xy / self.xy_step)
        n_yaw = int(self.search_yaw / self.yaw_step)

        for di in range(-n_yaw, n_yaw + 1):
            dyaw = di * self.yaw_step
            test_yaw = base_yaw + dyaw
            cy = math.cos(test_yaw)
            sy = math.sin(test_yaw)

            # Transform scan endpoints to map frame for this yaw
            # (translate part added inside the xy loop)
            wx_base = lx * cy - ly * sy
            wy_base = lx * sy + ly * cy

            for dxi in range(-n_xy, n_xy + 1):
                dx = dxi * self.xy_step
                for dyi in range(-n_xy, n_xy + 1):
                    dy = dyi * self.xy_step
                    test_x = base_x + dx
                    test_y = base_y + dy

                    # Map-frame scan points
                    wx = wx_base + test_x
                    wy = wy_base + test_y

                    # Convert to grid indices
                    gx = ((wx - map_ox) / res).astype(int)
                    gy = ((wy - map_oy) / res).astype(int)

                    # Count how many land on occupied cells
                    in_bounds = (gx >= 0) & (gx < gw) & (gy >= 0) & (gy < gh)
                    score = 0
                    if np.any(in_bounds):
                        gxi = gx[in_bounds]
                        gyi = gy[in_bounds]
                        score = int(np.sum(self.map_grid[gyi, gxi]))

                    if score > best_score:
                        best_score = score
                        best_dx = dx
                        best_dy = dy
                        best_dyaw = dyaw

        # ── Score quality gate ─────────────────────────────────
        # Reject corrections from bad matches to prevent runaway drift.
        # A score below MIN_SCORE_RATIO of total rays means the scan
        # doesn't match the map well enough to trust the correction.
        MIN_SCORE_RATIO = 0.15   # need at least 15% of rays matching
        MIN_SCORE_ABS   = 12     # or at least 12 rays matching
        min_score = max(MIN_SCORE_ABS, int(len(lx) * MIN_SCORE_RATIO))

        self.correction_count += 1

        if best_score < min_score:
            self.bad_match_streak += 1
            if self.bad_match_streak % 5 == 1:
                self.get_logger().warn(
                    f"Scan-match #{self.correction_count}: score={best_score}/{len(lx)} "
                    f"below threshold {min_score} — SKIPPING correction "
                    f"(bad streak: {self.bad_match_streak})")
            # If we've had many consecutive bad matches, the correction
            # has probably drifted far from truth.  Reset to zero.
            if self.bad_match_streak >= 20:
                self.get_logger().warn(
                    f"Resetting correction to zero after "
                    f"{self.bad_match_streak} consecutive bad matches")
                self.corr_dx = 0.0
                self.corr_dy = 0.0
                self.corr_dyaw = 0.0
                self.bad_match_streak = 0
            return

        # Good match — reset bad streak counter
        self.bad_match_streak = 0

        # ── Blend correction with EMA ────────────────────────────
        # best_dx/dy/dyaw is the delta FROM current corrected pose to
        # the best-match pose.  We add it to the existing correction offset.
        #
        # Per-step clamp: prevent a single spurious high-score match from
        # teleporting the robot.  0.15 m per step * alpha(0.4) = max 0.06m shift.
        MAX_STEP_XY  = 0.15   # metres   — maximum per-step XY shift
        MAX_STEP_YAW = 0.05   # radians  — maximum per-step yaw shift
        MAX_TOTAL_XY = 0.5    # metres   — absolute total correction limit
        MAX_TOTAL_YAW = 0.35  # radians (~20°) — absolute total yaw correction limit

        best_dx   = max(-MAX_STEP_XY,  min(MAX_STEP_XY,  best_dx))
        best_dy   = max(-MAX_STEP_XY,  min(MAX_STEP_XY,  best_dy))
        best_dyaw = max(-MAX_STEP_YAW, min(MAX_STEP_YAW, best_dyaw))

        # Score-weighted alpha: reduce correction strength for mediocre matches.
        # High score (118/120=98%): alpha~0.39 (full). Low (54/120=45%): alpha~0.18.
        score_confidence = best_score / max(1, len(lx))
        alpha = self.corr_alpha * score_confidence
        new_dx   = self.corr_dx   + alpha * best_dx
        new_dy   = self.corr_dy   + alpha * best_dy
        new_dyaw = self.corr_dyaw + alpha * best_dyaw

        # Total correction clamp: don't let accumulated drift exceed bounds
        if math.hypot(new_dx, new_dy) <= MAX_TOTAL_XY:
            self.corr_dx   = new_dx
            self.corr_dy   = new_dy
        # Total yaw correction clamp
        if abs(new_dyaw) <= MAX_TOTAL_YAW:
            self.corr_dyaw = new_dyaw

        if self.correction_count % 10 == 1:
            total_shift = math.hypot(self.corr_dx, self.corr_dy)
            self.get_logger().info(
                f"Scan-match #{self.correction_count}: \u0394=({best_dx:.3f}, "
                f"{best_dy:.3f}, {math.degrees(best_dyaw):.1f}\u00b0), "
                f"score={best_score}/{len(lx)}, "
                f"total correction=({self.corr_dx:.3f}, {self.corr_dy:.3f}, "
                f"{math.degrees(self.corr_dyaw):.1f}\u00b0) "
                f"shift={total_shift:.3f}m")

    # ══════════════════════════════════════════════════════════════════
    # Helpers
    # ══════════════════════════════════════════════════════════════════

    def _publish_pose(self, pub, x, y, yaw, stamp):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        half = yaw / 2.0
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)
        pub.publish(pose)


def main():
    rclpy.init()
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

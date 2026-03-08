#!/usr/bin/env python3
"""
Live Visualization Node
========================
Publishes two complementary OccupancyGrid maps and overlay markers so you
can compare *ground truth* (what Gazebo knows) vs *robot perception* (what
the robot's sensors estimate).

Published topics
-----------------
  /viz/gt_occupancy        OccupancyGrid  – live grid built from Gazebo GT
                           positions of humans + static map obstacles.
  /viz/perception_occupancy OccupancyGrid – live grid built from the robot's
                           detected human positions + static map obstacles.
  /viz/robot_estimate      MarkerArray    – robot's estimated pose (cyan arrow),
                           detected humans (orange), social circles (rings).
  /viz/gt_markers          MarkerArray    – GT robot pose (green arrow),
                           GT human positions (magenta spheres).
  /viz/social_circles      MarkerArray    – ring markers for intimate / personal
                           / social zones around each detected human.

Subscribed topics
------------------
  /robot_pose              PoseStamped    – robot's estimated pose (from
                           localization_node).
  /odom                    Odometry       – raw odom (for GT comparison via
                           spawn-transform, same as localization_node).
  /detected_humans         PoseArray      – camera-detected human positions
                           (tracker-ID in orientation.w).
  /human_velocities        PoseArray      – detected human velocities.
  /human_ground_truth      PoseArray      – GT positions from move_humans.
  /map                     OccupancyGrid  – static world map (TRANSIENT_LOCAL).
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header


# ─── Social zone radii (metres) ──────────────────────────────────────────────
INTIMATE_RADIUS  = 0.5    # Hall's intimate zone
PERSONAL_RADIUS  = 1.2    # Hall's personal zone
SOCIAL_RADIUS    = 3.0    # Hall's social zone


class LiveVisualizationNode(Node):
    """Generates live occupancy maps and overlay markers."""

    def __init__(self):
        super().__init__('live_visualization_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('resolution', 0.2)       # m / cell
        self.declare_parameter('map_size', 28.0)        # metres (square)
        self.declare_parameter('publish_rate', 2.0)      # Hz
        self.declare_parameter('human_radius_cells', 3)  # inflation radius in cells
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', -8.0)
        self.declare_parameter('spawn_yaw', 1.5708)

        self.resolution = self.get_parameter('resolution').value
        self.map_size   = self.get_parameter('map_size').value
        self.pub_rate   = self.get_parameter('publish_rate').value
        self.inflate_r  = self.get_parameter('human_radius_cells').value
        self.spawn_x    = self.get_parameter('spawn_x').value
        self.spawn_y    = self.get_parameter('spawn_y').value
        self.spawn_yaw  = self.get_parameter('spawn_yaw').value

        self.grid_w = int(self.map_size / self.resolution)
        self.grid_h = int(self.map_size / self.resolution)
        self.origin_x = -self.map_size / 2.0
        self.origin_y = -self.map_size / 2.0

        # ── State ─────────────────────────────────────────────────────
        self.static_grid = None            # from /map
        self.robot_x = 0.0                 # estimated pose
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.gt_robot_x = self.spawn_x     # GT pose (from odom + spawn)
        self.gt_robot_y = self.spawn_y
        self.gt_robot_yaw = self.spawn_yaw
        self.detected_humans = {}          # {id: (x, y)}
        self.human_velocities = {}         # {id: (vx, vy)}
        self.gt_humans = []                # [(x, y), ...]

        # ── Publishers ────────────────────────────────────────────────
        self.gt_occ_pub   = self.create_publisher(OccupancyGrid, '/viz/gt_occupancy', 10)
        self.perc_occ_pub = self.create_publisher(OccupancyGrid, '/viz/perception_occupancy', 10)
        self.robot_est_pub = self.create_publisher(MarkerArray, '/viz/robot_estimate', 10)
        self.gt_marker_pub = self.create_publisher(MarkerArray, '/viz/gt_markers', 10)
        self.social_pub    = self.create_publisher(MarkerArray, '/viz/social_circles', 10)

        # ── Subscribers ───────────────────────────────────────────────
        # Static map (TRANSIENT_LOCAL so we get it even if published before us)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)

        self.create_subscription(PoseStamped, '/robot_pose', self._robot_pose_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(PoseArray, '/detected_humans', self._detected_cb, 10)
        self.create_subscription(PoseArray, '/human_velocities', self._vel_cb, 10)
        self.create_subscription(PoseArray, '/human_ground_truth', self._gt_humans_cb, 10)

        # ── Timer ─────────────────────────────────────────────────────
        self.create_timer(1.0 / self.pub_rate, self._publish_all)
        self.get_logger().info(
            f'Live visualization node started  ({self.grid_w}×{self.grid_h} grid, '
            f'{self.resolution} m/cell, {self.pub_rate} Hz)')

    # ══════════════════════════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════════════════════════

    def _map_cb(self, msg: OccupancyGrid):
        """Receive static map, resample to our grid."""
        info = msg.info
        raw = np.array(msg.data, dtype=np.int8).reshape(
            (info.height, info.width))
        # Resample onto our grid
        self.static_grid = np.zeros((self.grid_h, self.grid_w), dtype=np.int8)
        for gy in range(self.grid_h):
            for gx in range(self.grid_w):
                wx = gx * self.resolution + self.origin_x + self.resolution / 2
                wy = gy * self.resolution + self.origin_y + self.resolution / 2
                # Map world → source grid
                sx = int((wx - info.origin.position.x) / info.resolution)
                sy = int((wy - info.origin.position.y) / info.resolution)
                if 0 <= sx < info.width and 0 <= sy < info.height:
                    v = raw[sy, sx]
                    if v >= 50:
                        self.static_grid[gy, gx] = 100
        self.get_logger().info('Static map received and resampled')

    def _robot_pose_cb(self, msg: PoseStamped):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        q = msg.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _odom_cb(self, msg: Odometry):
        """Convert raw odom to GT world-frame using spawn transform."""
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        oyaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c = math.cos(self.spawn_yaw)
        s = math.sin(self.spawn_yaw)
        self.gt_robot_x = c * ox - s * oy + self.spawn_x
        self.gt_robot_y = s * ox + c * oy + self.spawn_y
        self.gt_robot_yaw = oyaw + self.spawn_yaw

    def _detected_cb(self, msg: PoseArray):
        self.detected_humans = {
            int(p.orientation.w): (p.position.x, p.position.y)
            for p in msg.poses}

    def _vel_cb(self, msg: PoseArray):
        for p in msg.poses:
            hid = int(p.orientation.w)
            self.human_velocities[hid] = (p.position.x, p.position.y)

    def _gt_humans_cb(self, msg: PoseArray):
        self.gt_humans = [(p.position.x, p.position.y) for p in msg.poses]

    # ══════════════════════════════════════════════════════════════════
    # Grid helpers
    # ══════════════════════════════════════════════════════════════════

    def _world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def _stamp_humans_on_grid(self, grid, humans):
        """Paint human positions as occupied circles on the grid."""
        r = self.inflate_r
        for hx, hy in humans:
            cx, cy = self._world_to_grid(hx, hy)
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if dx * dx + dy * dy <= r * r:
                        nx, ny = cx + dx, cy + dy
                        if 0 <= nx < self.grid_w and 0 <= ny < self.grid_h:
                            grid[ny, nx] = 100

    def _build_occupancy_msg(self, grid, stamp):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = stamp
        msg.info.resolution = float(self.resolution)
        msg.info.width = self.grid_w
        msg.info.height = self.grid_h
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        return msg

    # ══════════════════════════════════════════════════════════════════
    # Marker builders
    # ══════════════════════════════════════════════════════════════════

    @staticmethod
    def _arrow_marker(ns, mid, x, y, yaw, r, g, b, a=1.0, scale=0.8, stamp=None):
        """Arrow marker showing position + heading."""
        m = Marker()
        m.header.frame_id = 'map'
        if stamp:
            m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.25
        m.pose.orientation.z = math.sin(yaw / 2)
        m.pose.orientation.w = math.cos(yaw / 2)
        m.scale.x = scale       # length
        m.scale.y = 0.15        # width
        m.scale.z = 0.15        # height
        m.color = ColorRGBA(r=r, g=g, b=b, a=a)
        m.lifetime.sec = 1
        return m

    @staticmethod
    def _sphere_marker(ns, mid, x, y, r, g, b, a=0.9, size=0.45, stamp=None):
        m = Marker()
        m.header.frame_id = 'map'
        if stamp:
            m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.9
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = size
        m.color = ColorRGBA(r=r, g=g, b=b, a=a)
        m.lifetime.sec = 1
        return m

    @staticmethod
    def _ring_marker(ns, mid, x, y, radius, r, g, b, a=0.4, z=0.05, stamp=None):
        """Flat cylinder (ring) for a social zone."""
        m = Marker()
        m.header.frame_id = 'map'
        if stamp:
            m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = radius * 2.0
        m.scale.z = 0.02           # very thin disc
        m.color = ColorRGBA(r=r, g=g, b=b, a=a)
        m.lifetime.sec = 1
        return m

    @staticmethod
    def _text_marker(ns, mid, x, y, text, r, g, b, a=1.0, stamp=None):
        m = Marker()
        m.header.frame_id = 'map'
        if stamp:
            m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 2.0
        m.pose.orientation.w = 1.0
        m.scale.z = 0.3       # text height
        m.color = ColorRGBA(r=r, g=g, b=b, a=a)
        m.text = text
        m.lifetime.sec = 1
        return m

    # ══════════════════════════════════════════════════════════════════
    # Main publish
    # ══════════════════════════════════════════════════════════════════

    def _publish_all(self):
        stamp = self.get_clock().now().to_msg()

        # ── 1. Ground-truth occupancy grid ────────────────────────────
        gt_grid = (self.static_grid.copy()
                   if self.static_grid is not None
                   else np.zeros((self.grid_h, self.grid_w), dtype=np.int8))
        self._stamp_humans_on_grid(gt_grid, self.gt_humans)
        # Also stamp robot GT position
        rx, ry = self._world_to_grid(self.gt_robot_x, self.gt_robot_y)
        if 0 <= rx < self.grid_w and 0 <= ry < self.grid_h:
            gt_grid[ry, rx] = 50   # mark robot position distinctly
        self.gt_occ_pub.publish(self._build_occupancy_msg(gt_grid, stamp))

        # ── 2. Perception occupancy grid ──────────────────────────────
        perc_grid = (self.static_grid.copy()
                     if self.static_grid is not None
                     else np.zeros((self.grid_h, self.grid_w), dtype=np.int8))
        det_list = list(self.detected_humans.values())
        self._stamp_humans_on_grid(perc_grid, det_list)
        rx2, ry2 = self._world_to_grid(self.robot_x, self.robot_y)
        if 0 <= rx2 < self.grid_w and 0 <= ry2 < self.grid_h:
            perc_grid[ry2, rx2] = 50
        self.perc_occ_pub.publish(self._build_occupancy_msg(perc_grid, stamp))

        # ── 3. Robot estimate markers ─────────────────────────────────
        est_ma = MarkerArray()
        delete = Marker(); delete.action = Marker.DELETEALL
        est_ma.markers.append(delete)
        mid = 0
        # Cyan arrow for estimated pose
        est_ma.markers.append(self._arrow_marker(
            'robot_est', mid, self.robot_x, self.robot_y, self.robot_yaw,
            0.0, 0.9, 1.0, stamp=stamp))
        mid += 1
        # Label
        est_ma.markers.append(self._text_marker(
            'robot_est', mid, self.robot_x, self.robot_y + 0.5,
            'Robot (est)', 0.0, 0.9, 1.0, stamp=stamp))
        mid += 1
        # Detected humans — orange spheres with ID labels
        for hid, (hx, hy) in self.detected_humans.items():
            est_ma.markers.append(self._sphere_marker(
                'det_human', mid, hx, hy,
                1.0, 0.6, 0.0, stamp=stamp))
            mid += 1
            vx, vy = self.human_velocities.get(hid, (0.0, 0.0))
            spd = math.hypot(vx, vy)
            dist = math.hypot(hx - self.robot_x, hy - self.robot_y)
            est_ma.markers.append(self._text_marker(
                'det_human', mid, hx, hy + 0.4,
                f'H{hid} d={dist:.1f}m v={spd:.2f}m/s',
                1.0, 0.8, 0.0, stamp=stamp))
            mid += 1
        self.robot_est_pub.publish(est_ma)

        # ── 4. Ground truth markers ───────────────────────────────────
        gt_ma = MarkerArray()
        delete2 = Marker(); delete2.action = Marker.DELETEALL
        gt_ma.markers.append(delete2)
        mid = 0
        # Green arrow for GT robot
        gt_ma.markers.append(self._arrow_marker(
            'robot_gt', mid, self.gt_robot_x, self.gt_robot_y,
            self.gt_robot_yaw,
            0.0, 1.0, 0.0, stamp=stamp))
        mid += 1
        gt_ma.markers.append(self._text_marker(
            'robot_gt', mid, self.gt_robot_x, self.gt_robot_y + 0.5,
            'Robot (GT)', 0.0, 1.0, 0.0, stamp=stamp))
        mid += 1
        # Magenta spheres for GT humans
        for i, (hx, hy) in enumerate(self.gt_humans):
            gt_ma.markers.append(self._sphere_marker(
                'gt_human', mid, hx, hy,
                1.0, 0.0, 1.0, size=0.5, stamp=stamp))
            mid += 1
            gt_ma.markers.append(self._text_marker(
                'gt_human', mid, hx, hy + 0.4,
                f'GT Human {i+1}',
                1.0, 0.0, 1.0, stamp=stamp))
            mid += 1
        self.gt_marker_pub.publish(gt_ma)

        # ── 5. Social circle rings ────────────────────────────────────
        sc_ma = MarkerArray()
        delete3 = Marker(); delete3.action = Marker.DELETEALL
        sc_ma.markers.append(delete3)
        mid = 0
        for hid, (hx, hy) in self.detected_humans.items():
            # Social zone (outermost, teal, z=0.03)
            sc_ma.markers.append(self._ring_marker(
                'social', mid, hx, hy, SOCIAL_RADIUS,
                0.0, 0.7, 0.7, a=0.15, z=0.03, stamp=stamp))
            mid += 1
            # Personal zone (orange, z=0.04)
            sc_ma.markers.append(self._ring_marker(
                'personal', mid, hx, hy, PERSONAL_RADIUS,
                1.0, 0.6, 0.0, a=0.25, z=0.04, stamp=stamp))
            mid += 1
            # Intimate zone (innermost, red, z=0.05)
            sc_ma.markers.append(self._ring_marker(
                'intimate', mid, hx, hy, INTIMATE_RADIUS,
                1.0, 0.0, 0.0, a=0.35, z=0.05, stamp=stamp))
            mid += 1
            # Label the zones
            sc_ma.markers.append(self._text_marker(
                'zone_label', mid, hx, hy - SOCIAL_RADIUS - 0.3,
                f'Social {SOCIAL_RADIUS}m',
                0.0, 0.7, 0.7, a=0.8, stamp=stamp))
            mid += 1
        self.social_pub.publish(sc_ma)


def main():
    rclpy.init()
    node = LiveVisualizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        # In Jazzy, context invalidation on multi-node shutdown raises
        # rclpy._rclpy_pybind11.RCLError ("the given context is not valid")
        # rather than ExternalShutdownException.  Catch broadly so the node
        # exits cleanly without printing a traceback.
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

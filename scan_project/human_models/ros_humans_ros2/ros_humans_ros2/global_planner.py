#!/usr/bin/env python3
"""
Global Planner Node
===================
Owns the weighted grid and runs A* to produce /global_path.

Responsibilities
----------------
1. Build a WeightedGrid from the static /map (one-time).
2. Plan a path when a /goal_pose is received.
3. Re-plan when the local planner publishes /replan_request:
   - Apply weight zones from /weight_zones (human costs)
   - Run A* from the NEW start (current robot position)
   - Decide whether the result is a minor deviation or a full reroute
   - Publish the new path on /global_path

Topics
------
  Subscribes:
    /map              (OccupancyGrid, TRANSIENT_LOCAL)  – static world map
    /robot_pose       (PoseStamped)                     – from localization node
    /goal_pose        (PoseStamped)                     – navigation goal
    /weight_zones     (PoseArray)                       – dynamic cost zones from local planner
    /replan_request   (PoseStamped)                     – local planner asks for replan

  Publishes:
    /global_path          (Path)                   – planned path for local planner
    /weighted_grid_viz    (OccupancyGrid)           – cost grid for RViz
    /path_markers         (MarkerArray)             – path + goal/start markers for RViz
"""

import math
import rclpy
import rclpy.qos
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Point
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool

from ros_humans_ros2.weighted_grid import WeightedGrid


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__("global_planner")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("planner_resolution", 0.2)
        self.declare_parameter("inflation_radius", 0.35)
        self.declare_parameter("goal_tolerance_m", 0.3)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("default_goal_x", 5.0)
        self.declare_parameter("default_goal_y", 5.0)
        self.declare_parameter("room_min_x", -13.0)
        self.declare_parameter("room_max_x", 13.0)
        self.declare_parameter("room_min_y", -13.0)
        self.declare_parameter("room_max_y", 13.0)
        # ── Proactive monitoring parameters ───────────────────────────
        self.declare_parameter("proactive_check_rate", 1.0)       # Hz
        # proactive_replan_radius is set BELOW zone_radius so that once the
        # planner reroutes the path outside the zone, the monitor no longer
        # sees the path as a threat and stops re-triggering.
        self.declare_parameter("proactive_replan_radius", 0.75)   # m – human→path distance to trigger
        self.declare_parameter("proactive_zone_radius", 0.8)      # m – weight zone radius
        self.declare_parameter("proactive_weight", 20.0)           # weight multiplier (high = strong avoidance)
        self.declare_parameter("proactive_cooldown", 10.0)         # s between proactive replans
        self.declare_parameter("predict_horizon", 3.0)             # s – human velocity projection

        self.planner_res = self.get_parameter("planner_resolution").value
        self.inflation = self.get_parameter("inflation_radius").value
        self.goal_tol = self.get_parameter("goal_tolerance_m").value
        self.auto_start = self.get_parameter("auto_start").value
        self.default_gx = self.get_parameter("default_goal_x").value
        self.default_gy = self.get_parameter("default_goal_y").value
        self.room_min_x = self.get_parameter("room_min_x").value
        self.room_max_x = self.get_parameter("room_max_x").value
        self.room_min_y = self.get_parameter("room_min_y").value
        self.room_max_y = self.get_parameter("room_max_y").value
        self.proactive_rate = self.get_parameter("proactive_check_rate").value
        self.proactive_radius = self.get_parameter("proactive_replan_radius").value
        self.proactive_zone_r = self.get_parameter("proactive_zone_radius").value
        self.proactive_weight = self.get_parameter("proactive_weight").value
        self.proactive_cooldown = self.get_parameter("proactive_cooldown").value
        self.predict_horizon = self.get_parameter("predict_horizon").value

        # ── State ─────────────────────────────────────────────────────
        self.grid = None                    # WeightedGrid (built from /map)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.goal_x = None
        self.goal_y = None
        self.current_path_world = []        # latest path (world coords)
        self.current_path_grid = []         # latest path (grid cells)
        self.current_path_dist = 0.0
        self.map_received = False
        self.pose_received = False
        self.weight_zones = []              # latest zones from local planner

        # ── Proactive human tracking ──────────────────────────────────
        self.detected_humans = []           # [(x, y), ...] from /detected_humans
        self.detected_human_vels = []       # [(vx, vy), ...] from /human_velocities
        self.last_proactive_replan = 0.0    # monotonic time of last proactive replan
        self.proactive_human_set = set()    # track which humans already caused replans

        # ── Publishers ────────────────────────────────────────────────
        self.path_pub = self.create_publisher(Path, "/global_path", 10)
        self.grid_viz_pub = self.create_publisher(
            OccupancyGrid, "/weighted_grid_viz", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/path_markers", 10)
        # Notify local planner that a path is ready (or no path)
        self.status_pub = self.create_publisher(Bool, "/global_planner_status", 10)

        # ── Subscribers ───────────────────────────────────────────────
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_cb,
            rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        self.pose_sub = self.create_subscription(
            PoseStamped, "/robot_pose", self.pose_cb, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_cb, 10)
        self.weight_sub = self.create_subscription(
            PoseArray, "/weight_zones", self.weight_cb, 10)
        self.replan_sub = self.create_subscription(
            PoseStamped, "/replan_request", self.replan_cb, 10)

        # ── Proactive human subscriptions ─────────────────────────────
        self.create_subscription(
            PoseArray, "/detected_humans", self._humans_cb, 10)
        self.create_subscription(
            PoseArray, "/human_velocities", self._hvel_cb, 10)

        # ── Timers ────────────────────────────────────────────────────
        self.create_timer(2.0, self.publish_grid_viz)
        self.init_ticks = 0
        self.create_timer(0.1, self.init_check)
        # Proactive path monitor — continuously checks path vs humans
        if self.proactive_rate > 0:
            self.create_timer(
                1.0 / self.proactive_rate, self._proactive_monitor)

        self.get_logger().info("Global planner started (waiting for /map)…")

    # ── Callbacks ─────────────────────────────────────────────────────

    def map_cb(self, msg):
        if self.map_received:
            return
        self.get_logger().info(
            f"Static map received: {msg.info.width}×{msg.info.height} "
            f"@ {msg.info.resolution} m/px")
        self.grid = WeightedGrid.from_occupancy_grid(
            msg,
            planner_resolution=self.planner_res,
            inflation_radius=self.inflation)
        self.map_received = True
        self.get_logger().info(
            f"Weighted grid built: {self.grid.width}×{self.grid.height} cells "
            f"@ {self.planner_res} m/cell, edges initially EQUAL weight")

    def pose_cb(self, msg):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        if not self.pose_received:
            self.pose_received = True
            self.get_logger().info(
                f"Robot pose received: ({self.robot_x:.2f}, {self.robot_y:.2f})")

    def goal_cb(self, msg):
        gx = max(self.room_min_x + 0.5, min(self.room_max_x - 0.5,
                                              msg.pose.position.x))
        gy = max(self.room_min_y + 0.5, min(self.room_max_y - 0.5,
                                              msg.pose.position.y))
        self.goal_x = gx
        self.goal_y = gy
        self.get_logger().info(f"Goal set: ({gx:.2f}, {gy:.2f})")
        self.plan_path(self.robot_x, self.robot_y)

    def weight_cb(self, msg):
        """Receive dynamic weight zones from local planner.

        Each pose encodes one zone:
            position.x/y  – world centre
            position.z     – radius (m)
            orientation.w  – weight multiplier
        """
        self.weight_zones = []
        for p in msg.poses:
            self.weight_zones.append(
                (p.position.x, p.position.y,
                 p.position.z,            # radius
                 p.orientation.w))         # multiplier
        # Don't plan yet — wait for /replan_request so start point is known

    def replan_cb(self, msg):
        """Local planner asks for a replan from the given start position."""
        start_x = msg.pose.position.x
        start_y = msg.pose.position.y
        self.get_logger().info(
            f"Replan requested from ({start_x:.2f}, {start_y:.2f})",
            throttle_duration_sec=3.0)
        self.plan_path(start_x, start_y, is_replan=True)

    # ── Proactive human callbacks ─────────────────────────────────────

    def _humans_cb(self, msg):
        """Receive detected humans directly for proactive monitoring."""
        self.detected_humans = [(p.position.x, p.position.y) for p in msg.poses]

    def _hvel_cb(self, msg):
        """Receive human velocities for proactive prediction."""
        self.detected_human_vels = [
            (p.position.x, p.position.y) for p in msg.poses]

    # ── Proactive path monitoring ─────────────────────────────────────

    def _proactive_monitor(self):
        """Periodically check if any detected human threatens the current
        path.  If so, build weight zones and replan BEFORE the robot gets
        close.  This is the key difference from the reactive local-planner
        reroute — we look at the ENTIRE remaining path, not just nearby
        segments."""
        if (not self.current_path_world
                or not self.detected_humans
                or self.grid is None
                or self.goal_x is None):
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_proactive_replan < self.proactive_cooldown:
            return

        # ── Check every detected human against the FULL path ──────
        threatening = []  # [(hx, hy, vx, vy, min_dist_to_path), ...]

        for hi, (hx, hy) in enumerate(self.detected_humans):
            vx, vy = 0.0, 0.0
            if hi < len(self.detected_human_vels):
                vx, vy = self.detected_human_vels[hi]

            # Find minimum distance from this human to ANY path
            # SEGMENT (not just waypoints).  With simplified paths the
            # waypoints can be far apart while the line between them
            # passes right through the human.
            min_d = float("inf")
            path = self.current_path_world
            for si in range(len(path) - 1):
                d = self._pt_seg_dist(
                    hx, hy,
                    path[si][0], path[si][1],
                    path[si + 1][0], path[si + 1][1])
                if d < min_d:
                    min_d = d
            # Also distance to last waypoint
            d = math.hypot(hx - path[-1][0], hy - path[-1][1])
            if d < min_d:
                min_d = d

            # Also check predicted future positions
            speed = math.hypot(vx, vy)
            if speed > 0.05:
                for dt in [1.0, 2.0, 3.0]:
                    if dt > self.predict_horizon:
                        break
                    px = hx + vx * dt
                    py = hy + vy * dt
                    for si in range(len(path) - 1):
                        d = self._pt_seg_dist(
                            px, py,
                            path[si][0], path[si][1],
                            path[si + 1][0], path[si + 1][1])
                        if d < min_d:
                            min_d = d

            if min_d < self.proactive_radius:
                threatening.append((hx, hy, vx, vy, min_d))

        if not threatening:
            # No humans near path — if we previously had zones, clear them
            if self.weight_zones:
                self.weight_zones = []
                self.grid.reset_dynamic_weights()
            return

        # ── Build weight zones for ALL threatening humans ──────────
        zones = []
        for hx, hy, vx, vy, _ in threatening:
            speed = math.hypot(vx, vy)
            # Current position zone
            zones.append(
                (hx, hy, self.proactive_zone_r, self.proactive_weight))
            # Predicted future position zones (if moving)
            if speed > 0.05:
                steps = max(1, int(self.predict_horizon / 0.5))
                for s in range(1, steps + 1):
                    dt = s * 0.5
                    px = hx + vx * dt
                    py = hy + vy * dt
                    decay = 1.0 - (dt / (self.predict_horizon + 0.1)) * 0.5
                    w = max(2.0, self.proactive_weight * decay)
                    zones.append((px, py, self.proactive_zone_r, w))

        self.weight_zones = zones

        # Log the proactive replan (throttled)
        dists = [f"{d:.1f}m" for _, _, _, _, d in threatening]
        self.get_logger().info(
            f"⚡ Proactive replan: {len(threatening)} human(s) near path "
            f"(distances: {', '.join(dists)}), applying {len(zones)} zones",
            throttle_duration_sec=5.0)

        self.last_proactive_replan = now
        self.plan_path(self.robot_x, self.robot_y, is_replan=True)

    # ── Init auto-start ──────────────────────────────────────────────

    def init_check(self):
        """Wait for map + pose, then auto-start if configured."""
        self.init_ticks += 1
        if self.map_received and self.pose_received and self.init_ticks > 50:
            if self.auto_start and self.goal_x is None:
                self.goal_x = self.default_gx
                self.goal_y = self.default_gy
                self.get_logger().info(
                    f"Auto-start: planning to ({self.goal_x}, {self.goal_y})")
                self.plan_path(self.robot_x, self.robot_y)

    # ── Planning ──────────────────────────────────────────────────────

    def plan_path(self, start_x, start_y, is_replan=False):
        if self.grid is None:
            self.get_logger().warn("No grid yet — cannot plan")
            return
        if self.goal_x is None:
            self.get_logger().warn("No goal set — cannot plan")
            return

        # Apply dynamic weight zones (human costs) if any
        if self.weight_zones:
            self.grid.update_dynamic_weights(self.weight_zones)
            self.get_logger().debug(
                f"Applied {len(self.weight_zones)} weight zone(s)")
        else:
            self.grid.reset_dynamic_weights()

        start_grid = self.grid.world_to_grid(start_x, start_y)
        goal_grid = self.grid.world_to_grid(self.goal_x, self.goal_y)

        self.get_logger().debug(
            f"A* planning: ({start_x:.2f},{start_y:.2f}) → "
            f"({self.goal_x:.2f},{self.goal_y:.2f})  "
            f"grid {start_grid} → {goal_grid}")

        grid_path = self.grid.astar(start_grid, goal_grid)

        if grid_path is None:
            self.get_logger().error("A* found NO path!")
            self._publish_status(False)
            return

        # Simplify (LOS pruning + corner smoothing)
        world_path = self.grid.simplify_path(grid_path)
        new_dist = self._world_path_distance(world_path)

        # ── Deviation analysis (replan only) ──────────────────────
        if is_replan and self.current_path_world:
            old_dist = self.current_path_dist
            if old_dist > 0:
                ratio = new_dist / old_dist
                overlap = self._path_overlap(self.current_path_world, world_path)
                if overlap > 0.6:
                    self.get_logger().debug(
                        f"↳ Minor deviation  "
                        f"({new_dist:.1f}m vs {old_dist:.1f}m, "
                        f"{overlap*100:.0f}% overlap)")
                else:
                    self.get_logger().debug(
                        f"↳ Full path change  "
                        f"({new_dist:.1f}m vs {old_dist:.1f}m, "
                        f"{overlap*100:.0f}% overlap)")

        # Store and publish
        self.current_path_world = world_path
        self.current_path_grid = grid_path
        self.current_path_dist = new_dist

        self._publish_path(world_path)
        self._publish_markers(world_path, start_x, start_y)
        self._publish_status(True)

        self.get_logger().info(
            f"Path published: {len(world_path)} waypoints, {new_dist:.2f} m",
            throttle_duration_sec=3.0)

    # ── Helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _pt_seg_dist(px, py, ax, ay, bx, by):
        """Distance from point (px,py) to line segment (ax,ay)-(bx,by)."""
        dx, dy = bx - ax, by - ay
        len_sq = dx * dx + dy * dy
        if len_sq < 1e-12:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
        return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

    @staticmethod
    def _world_path_distance(path):
        d = 0.0
        for i in range(1, len(path)):
            d += math.hypot(path[i][0] - path[i - 1][0],
                            path[i][1] - path[i - 1][1])
        return d

    @staticmethod
    def _path_overlap(old_path, new_path, tolerance=1.0):
        """Fraction of old_path waypoints that have a new_path waypoint
        within *tolerance* metres.  Used to decide deviation vs reroute."""
        if not old_path:
            return 0.0
        matches = 0
        for ox, oy in old_path:
            for nx, ny in new_path:
                if math.hypot(ox - nx, oy - ny) < tolerance:
                    matches += 1
                    break
        return matches / len(old_path)

    # ── Publishers ────────────────────────────────────────────────────

    def _publish_path(self, world_path):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in world_path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def _publish_status(self, success):
        msg = Bool()
        msg.data = success
        self.status_pub.publish(msg)

    def _publish_markers(self, world_path, start_x, start_y):
        markers = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        stamp = self.get_clock().now().to_msg()
        mid = 0

        # Path line
        if world_path:
            line = Marker()
            line.header.frame_id = "map"
            line.header.stamp = stamp
            line.ns = "global_path"
            line.id = mid; mid += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.12
            line.color.r = 0.0
            line.color.g = 1.0
            line.color.b = 0.0
            line.color.a = 0.9
            for wx, wy in world_path:
                p = Point(); p.x = wx; p.y = wy; p.z = 0.15
                line.points.append(p)
            markers.markers.append(line)

            # Distance label
            mi = len(world_path) // 2
            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = stamp
            label.ns = "path_label"
            label.id = mid; mid += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = world_path[mi][0] + 0.3
            label.pose.position.y = world_path[mi][1] + 0.3
            label.pose.position.z = 1.2
            label.pose.orientation.w = 1.0
            label.scale.z = 0.5
            label.color.g = 1.0; label.color.a = 1.0
            label.text = f"{self.current_path_dist:.1f} m"
            markers.markers.append(label)

        # Goal sphere
        if self.goal_x is not None:
            g = Marker()
            g.header.frame_id = "map"; g.header.stamp = stamp
            g.ns = "goal"; g.id = mid; mid += 1
            g.type = Marker.SPHERE; g.action = Marker.ADD
            g.pose.position.x = self.goal_x
            g.pose.position.y = self.goal_y
            g.pose.position.z = 0.5
            g.pose.orientation.w = 1.0
            g.scale.x = g.scale.y = g.scale.z = 0.5
            g.color.r = 1.0; g.color.a = 1.0
            markers.markers.append(g)

        # Start sphere
        s = Marker()
        s.header.frame_id = "map"; s.header.stamp = stamp
        s.ns = "start"; s.id = mid; mid += 1
        s.type = Marker.SPHERE; s.action = Marker.ADD
        s.pose.position.x = start_x
        s.pose.position.y = start_y
        s.pose.position.z = 0.5
        s.pose.orientation.w = 1.0
        s.scale.x = s.scale.y = s.scale.z = 0.4
        s.color.b = 1.0; s.color.a = 1.0
        markers.markers.append(s)

        self.marker_pub.publish(markers)

    def publish_grid_viz(self):
        """Publish the weighted grid as an OccupancyGrid for RViz."""
        if self.grid is None:
            return
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.grid.resolution
        msg.info.width = self.grid.width
        msg.info.height = self.grid.height
        msg.info.origin.position.x = self.grid.origin_x
        msg.info.origin.position.y = self.grid.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.to_occupancy_data()
        self.grid_viz_pub.publish(msg)


def main():
    rclpy.init()
    node = GlobalPlanner()
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

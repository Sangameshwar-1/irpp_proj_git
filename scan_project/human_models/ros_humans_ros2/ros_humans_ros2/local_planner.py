#!/usr/bin/env python3
"""
Local Planner Node
==================
Follows the global path, handles dynamic obstacles (humans), and decides
whether to WAIT or REROUTE.

Decision Logic
--------------
  Human CROSSING the path (perpendicular, will clear):   → WAIT
  Human STATIONARY on the path:                          → REROUTE
  Human APPROACHING along the path (collision course):   → REROUTE
  Human MOVING AWAY from path:                           → IGNORE
  Waited too long (timeout):                             → REROUTE

When rerouting:
  1. Compute weight zones around the human (current + predicted positions)
  2. Publish zones on /weight_zones
  3. Publish current robot pose on /replan_request
  4. Global planner re-runs A* with updated weights → new /global_path

Topics
------
  Subscribes:
    /global_path        (Path)        – from global planner
    /robot_pose         (PoseStamped) – from localization node
    /scan               (LaserScan)   – LiDAR for emergency stops
    /detected_humans    (PoseArray)   – from human detector
    /human_velocities   (PoseArray)   – from human detector

  Publishes:
    /cmd_vel            (Twist)       – robot velocity
    /weight_zones       (PoseArray)   – dynamic cost zones for global planner
    /replan_request     (PoseStamped) – ask global planner to replan
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Pose, PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


class LocalPlanner(Node):
    """Reactive path follower with WAIT / REROUTE human-avoidance."""

    def __init__(self):
        super().__init__("local_planner")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("linear_speed", 0.3)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("waypoint_tolerance", 0.3)
        self.declare_parameter("human_threat_dist", 3.0)     # m from path
        self.declare_parameter("human_zone_radius", 1.5)     # m weight zone
        self.declare_parameter("weight_multiplier", 10.0)    # cost increase
        self.declare_parameter("max_wait_time", 8.0)         # seconds
        self.declare_parameter("crossing_clear_time", 5.0)   # seconds
        self.declare_parameter("predict_horizon", 3.0)       # seconds ahead
        self.declare_parameter("emergency_stop_dist", 0.4)   # m

        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.wp_tol = self.get_parameter("waypoint_tolerance").value
        self.threat_dist = self.get_parameter("human_threat_dist").value
        self.zone_radius = self.get_parameter("human_zone_radius").value
        self.weight_mult = self.get_parameter("weight_multiplier").value
        self.max_wait = self.get_parameter("max_wait_time").value
        self.cross_clear = self.get_parameter("crossing_clear_time").value
        self.predict_hz = self.get_parameter("predict_horizon").value
        self.e_stop_dist = self.get_parameter("emergency_stop_dist").value

        # ── State ─────────────────────────────────────────────────────
        self.state = "IDLE"  # IDLE | FOLLOWING | WAITING | REACHED
        self.path = []                  # list of (x, y) world waypoints
        self.path_idx = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.pose_received = False
        self.scan_data = None

        # Human tracking
        self.humans = []                # [(x, y), ...]
        self.human_vels = []            # [(vx, vy), ...]

        # Wait state
        self.wait_start = 0.0
        self.wait_human_idx = -1

        # Replan cooldown (avoid spamming replans)
        self.last_replan_time = 0.0
        self.replan_cooldown = 3.0      # seconds

        # ── Publishers ────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.weight_pub = self.create_publisher(PoseArray, "/weight_zones", 10)
        self.replan_pub = self.create_publisher(PoseStamped, "/replan_request", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/local_planner_markers", 10)

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(Path, "/global_path", self.path_cb, 10)
        self.create_subscription(PoseStamped, "/robot_pose", self.pose_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(PoseArray, "/detected_humans",
                                 self.humans_cb, 10)
        self.create_subscription(PoseArray, "/human_velocities",
                                 self.hvel_cb, 10)

        # ── Control loop (10 Hz) ──────────────────────────────────────
        self.create_timer(0.1, self.control_loop)
        self.tick = 0

        self.get_logger().info("Local planner started (WAIT / REROUTE mode)")

    # ── Callbacks ─────────────────────────────────────────────────────

    def path_cb(self, msg):
        """Receive a new global path."""
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not self.path:
            self.get_logger().warn("Received empty path")
            self.state = "IDLE"
            return
        # Skip first waypoint if it's basically where we are
        if len(self.path) > 1:
            d = math.hypot(self.path[0][0] - self.robot_x,
                           self.path[0][1] - self.robot_y)
            if d < self.wp_tol:
                self.path = self.path[1:]
        self.path_idx = 0
        self.state = "FOLLOWING"
        self.get_logger().info(
            f"New path received: {len(self.path)} waypoints → FOLLOWING")

    def pose_cb(self, msg):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)
        self.pose_received = True

    def scan_cb(self, msg):
        self.scan_data = msg

    def humans_cb(self, msg):
        self.humans = [(p.position.x, p.position.y) for p in msg.poses]

    def hvel_cb(self, msg):
        self.human_vels = [(p.position.x, p.position.y) for p in msg.poses]

    # ── Main control loop ─────────────────────────────────────────────

    def control_loop(self):
        self.tick += 1
        cmd = Twist()

        if not self.pose_received:
            self.cmd_pub.publish(cmd)
            return

        if self.state == "IDLE":
            self.cmd_pub.publish(cmd)
            return

        if self.state == "REACHED":
            if self.tick % 20 == 0:
                self.get_logger().info("Goal reached — waiting for new path")
            self.cmd_pub.publish(cmd)
            return

        if self.state == "WAITING":
            self._handle_waiting()
            self.cmd_pub.publish(cmd)  # stop while waiting
            return

        if self.state == "FOLLOWING":
            # ── Emergency LiDAR check ──
            if self._emergency_obstacle():
                self._request_replan("emergency LiDAR obstacle")
                self.cmd_pub.publish(cmd)
                return

            # ── Check human threats ──
            threat = self._find_threat()
            if threat is not None:
                action = threat["action"]
                if action == "WAIT":
                    self.state = "WAITING"
                    self.wait_start = time.time()
                    self.wait_human_idx = threat["idx"]
                    self.get_logger().info(
                        f"WAITING for human {threat['idx']} "
                        f"(crossing, est clear {threat.get('clear_time', '?')}s)")
                    self.cmd_pub.publish(cmd)
                    return
                elif action == "REROUTE":
                    self.get_logger().warn(
                        f"REROUTE around human {threat['idx']} "
                        f"(reason: {threat.get('reason', 'threat')})")
                    self._reroute_around(threat)
                    self.cmd_pub.publish(cmd)
                    return

            # ── Follow path ──
            cmd = self._follow_path()
            self.cmd_pub.publish(cmd)

    # ── Path following ────────────────────────────────────────────────

    def _follow_path(self):
        cmd = Twist()
        if self.path_idx >= len(self.path):
            self.state = "REACHED"
            self.get_logger().info("Final waypoint reached!")
            return cmd

        tx, ty = self.path[self.path_idx]
        dx = tx - self.robot_x
        dy = ty - self.robot_y
        dist = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)

        # Advance waypoint
        if dist < self.wp_tol:
            self.path_idx += 1
            if self.tick % 10 == 0:
                self.get_logger().info(
                    f"Waypoint {self.path_idx}/{len(self.path)}")
            if self.path_idx >= len(self.path):
                self.state = "REACHED"
                return cmd
            tx, ty = self.path[self.path_idx]
            dx, dy = tx - self.robot_x, ty - self.robot_y
            dist = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)

        # Angle difference
        angle_diff = target_angle - self.robot_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Curvature look-ahead
        curvature = 0.0
        if self.path_idx + 1 < len(self.path):
            nx, ny = self.path[self.path_idx + 1]
            ahead = math.atan2(ny - ty, nx - tx)
            curvature = abs(ahead - target_angle)
            if curvature > math.pi:
                curvature = 2 * math.pi - curvature

        # Slow down near detected humans (proportional)
        human_factor = self._human_proximity_factor()

        if abs(angle_diff) > 0.5:
            cmd.angular.z = (self.angular_speed if angle_diff > 0
                             else -self.angular_speed)
            cmd.linear.x = 0.05 * human_factor
        elif abs(angle_diff) > 0.15:
            cmd.linear.x = 0.10 * human_factor
            cmd.angular.z = angle_diff * 2.0
        else:
            curv_f = max(0.3, 1.0 - curvature * 0.8)
            appr_f = min(1.0, dist / 0.8)
            speed = self.linear_speed * curv_f * appr_f * human_factor
            cmd.linear.x = max(0.05, min(self.linear_speed, speed))
            cmd.angular.z = angle_diff * 2.0

        return cmd

    def _human_proximity_factor(self):
        """Speed reduction factor [0.15, 1.0] based on nearest human."""
        if not self.humans:
            return 1.0
        min_d = float("inf")
        for hx, hy in self.humans:
            d = math.hypot(hx - self.robot_x, hy - self.robot_y)
            min_d = min(min_d, d)
        if min_d > self.threat_dist:
            return 1.0
        return max(0.15, min_d / self.threat_dist)

    # ── Human threat analysis ─────────────────────────────────────────

    def _find_threat(self):
        """Analyse detected humans relative to upcoming path.

        Returns the most urgent threat dict, or None.
        """
        if not self.humans or not self.path:
            return None

        for idx, (hx, hy) in enumerate(self.humans):
            vx, vy = 0.0, 0.0
            if idx < len(self.human_vels):
                vx, vy = self.human_vels[idx]
            speed = math.hypot(vx, vy)

            # Distance from robot to human
            robot_dist = math.hypot(hx - self.robot_x, hy - self.robot_y)
            if robot_dist > self.threat_dist + 2.0:
                continue

            # Distance from human to upcoming path segments
            min_path_d = float("inf")
            closest_seg = self.path_idx
            lookahead = min(self.path_idx + 12, len(self.path))
            for wi in range(self.path_idx, lookahead):
                d = math.hypot(hx - self.path[wi][0], hy - self.path[wi][1])
                if d < min_path_d:
                    min_path_d = d
                    closest_seg = wi

            # Not near path → ignore
            if min_path_d > self.zone_radius + 0.5:
                continue

            # ── Velocity analysis ──
            wpx, wpy = self.path[closest_seg]
            to_path = (wpx - hx, wpy - hy)
            to_path_len = math.hypot(*to_path)

            # Approach rate toward path (>0 = closing)
            approach_rate = 0.0
            if to_path_len > 0.01:
                approach_rate = (vx * to_path[0] + vy * to_path[1]) / to_path_len

            # ── Decision ──

            # Moving AWAY from path → ignore
            if approach_rate < -0.15 and speed > 0.1:
                continue

            # CROSSING the path (perpendicular, will clear quickly)
            if speed > 0.15 and abs(approach_rate) < speed * 0.5:
                clear_d = max(0.0, self.zone_radius - min_path_d)
                clear_t = clear_d / speed if speed > 0.05 else 999.0
                if clear_t < self.cross_clear:
                    return {"action": "WAIT", "idx": idx,
                            "hx": hx, "hy": hy, "vx": vx, "vy": vy,
                            "clear_time": f"{clear_t:.1f}"}

            # STATIONARY or APPROACHING → reroute
            reason = "stationary on path" if speed < 0.1 else "approaching path"
            return {"action": "REROUTE", "idx": idx,
                    "hx": hx, "hy": hy, "vx": vx, "vy": vy,
                    "reason": reason}

        return None

    # ── WAIT state ────────────────────────────────────────────────────

    def _handle_waiting(self):
        """While WAITING, check if the human has cleared or timeout."""
        elapsed = time.time() - self.wait_start

        # Check if the human that caused the wait is still a threat
        cleared = True
        if self.wait_human_idx < len(self.humans):
            hx, hy = self.humans[self.wait_human_idx]
            # Check distance to upcoming path
            for wi in range(self.path_idx,
                            min(self.path_idx + 8, len(self.path))):
                d = math.hypot(hx - self.path[wi][0], hy - self.path[wi][1])
                if d < self.zone_radius:
                    cleared = False
                    break

        if cleared:
            self.get_logger().info(
                f"Human cleared after {elapsed:.1f}s → resuming FOLLOWING")
            self.state = "FOLLOWING"
            return

        if elapsed > self.max_wait:
            self.get_logger().warn(
                f"Wait timeout ({self.max_wait}s) → REROUTE")
            threat = {"idx": self.wait_human_idx,
                      "hx": self.humans[self.wait_human_idx][0],
                      "hy": self.humans[self.wait_human_idx][1],
                      "vx": 0.0, "vy": 0.0, "reason": "wait_timeout"}
            if self.wait_human_idx < len(self.human_vels):
                threat["vx"] = self.human_vels[self.wait_human_idx][0]
                threat["vy"] = self.human_vels[self.wait_human_idx][1]
            self._reroute_around(threat)

    # ── REROUTE ───────────────────────────────────────────────────────

    def _reroute_around(self, threat):
        """Update edge weights around threat and request global replan."""
        now = time.time()
        if now - self.last_replan_time < self.replan_cooldown:
            self.get_logger().info("Replan cooldown — not yet",
                                   throttle_duration_sec=2.0)
            return
        self.last_replan_time = now

        hx, hy = threat["hx"], threat["hy"]
        vx = threat.get("vx", 0.0)
        vy = threat.get("vy", 0.0)

        # Build weight zones: current position + predicted future positions
        zones = PoseArray()
        zones.header.frame_id = "map"
        zones.header.stamp = self.get_clock().now().to_msg()

        steps = max(1, int(self.predict_hz / 0.5))
        for s in range(steps + 1):
            dt = s * 0.5
            px = hx + vx * dt
            py = hy + vy * dt

            # Decay weight with prediction time (less certain = lower cost)
            decay = 1.0 - (dt / (self.predict_hz + 0.1)) * 0.5
            w = max(2.0, self.weight_mult * decay)

            pose = Pose()
            pose.position.x = px
            pose.position.y = py
            pose.position.z = self.zone_radius   # radius
            pose.orientation.w = w                # weight multiplier
            zones.poses.append(pose)

        # Publish weight zones
        self.weight_pub.publish(zones)

        # Request replan from CURRENT robot position (not original start!)
        req = PoseStamped()
        req.header.frame_id = "map"
        req.header.stamp = self.get_clock().now().to_msg()
        req.pose.position.x = self.robot_x
        req.pose.position.y = self.robot_y
        req.pose.orientation.w = 1.0
        self.replan_pub.publish(req)

        self.state = "IDLE"  # wait for new global path
        self.get_logger().info(
            f"Replan request sent (start=({self.robot_x:.1f},{self.robot_y:.1f}), "
            f"{len(zones.poses)} weight zones)")

    def _request_replan(self, reason=""):
        """Simple replan request without weight updates."""
        now = time.time()
        if now - self.last_replan_time < self.replan_cooldown:
            return
        self.last_replan_time = now

        req = PoseStamped()
        req.header.frame_id = "map"
        req.header.stamp = self.get_clock().now().to_msg()
        req.pose.position.x = self.robot_x
        req.pose.position.y = self.robot_y
        req.pose.orientation.w = 1.0
        self.replan_pub.publish(req)
        self.state = "IDLE"
        self.get_logger().warn(f"Replan requested: {reason}")

    # ── Emergency LiDAR check ─────────────────────────────────────────

    def _emergency_obstacle(self):
        """True if there's an obstacle dangerously close in the forward arc."""
        if self.scan_data is None:
            return False
        close = 0
        for i, r in enumerate(self.scan_data.ranges):
            if self.scan_data.range_min < r < self.e_stop_dist:
                angle = (self.scan_data.angle_min
                         + i * self.scan_data.angle_increment)
                if abs(angle) < 0.52:  # ±30°
                    close += 1
                    if close >= 3:
                        return True
        return False


def main():
    rclpy.init()
    node = LocalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

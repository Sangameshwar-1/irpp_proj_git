#!/usr/bin/env python3
"""
Social Navigation Planner
=========================
Pure planning test node that receives GROUND-TRUTH human positions and
velocities from human_case_controller.py and implements four social
navigation cases with full RViz visualisation.

This node bypasses perception entirely.

Cases
-----
  Case 1  Stationary human on path
          → Deviate: publish weight zone + request A* reroute

  Case 2  Human approaching head-on (opposing direction)
          → Slow down proportionally; stop if very close; resume when clear

  Case 3  Human crossing robot's forward path (collision cone)
          → Compute collision cone; slow to pass BEHIND the human
          → Required speed = dist_to_crossing / (t_human_clears + buffer)

  Case 4a Human ahead, same direction
          → Do NOT overtake; match/reduce speed within social radius

  Case 4b Human behind, same direction, faster
          → Give way: slow + apply lateral nudge so human can pass

RViz markers published on /social_nav_markers
---------------------------------------------
  • Collision cone     — fan of triangles (red=conflict, yellow=caution, green=clear)
  • Velocity arrows    — robot velocity (green), human velocity (orange)
  • Human social zones — personal (orange ring) + social (blue ring)
  • Human sphere       — orange ball at human position
  • Trajectory lines   — predicted paths 5 s ahead
  • Crossing ×         — red X where human path crosses robot path (case3)
  • Decision banner    — floating text above robot: case + action
  • Speed readout      — current target speed

Topics
------
  Sub: /detected_humans    (PoseArray)   — ground truth position
       /human_velocities   (PoseArray)   — ground truth velocity
       /robot_pose         (PoseStamped) — from localization_node
       /global_path        (Path)        — from global_planner
       /scan               (LaserScan)   — emergency stop
  Pub: /cmd_vel            (Twist)
       /weight_zones       (PoseArray)   — case1 rerouting
       /replan_request     (PoseStamped) — case1 rerouting
       /social_nav_markers (MarkerArray) — RViz visualisation
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Pose, PoseArray, Point
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# ── Planning constants ────────────────────────────────────────────────────────
STATIC_THR    = 0.05   # m/s  — human considered stationary (case 1)
COLL_RAD      = 0.50   # m    — combined robot (0.15) + human (0.25) + margin
SOCIAL_RAD    = 1.50   # m    — social zone outer ring
PERSONAL_RAD  = 0.80   # m    — personal zone inner ring
SAME_DIR_THR  = 0.70   # cos  — heading similarity for "same direction" (~45°)
APPROACH_THR  = 0.15   # m/s  — head-on approach rate → classify as case 2
CROSS_BUF     = 2.50   # s    — extra safety buffer after human clears crossing
MIN_SPD       = 0.02   # m/s  — minimum forward speed
DEF_MAX_SPD   = 0.30   # m/s  — default robot max speed for test cases
DEF_ANG_SPD   = 0.30   # rad/s
GIVE_WAY_DIST = 1.50   # m    — trigger give-way for faster human from behind
OVERTAKE_WARN = 2.00   # m    — no-overtake zone when following ahead human
WP_TOL        = 0.30   # m    — waypoint reached tolerance
EMERG_DIST    = 0.35   # m    — LiDAR emergency stop
EMERG_ARC     = 0.52   # rad  — ±30° forward arc


def _col(r, g, b, a=1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


class SocialNavPlanner(Node):
    """Case-based social navigation planner for pure-planning tests."""

    def __init__(self):
        super().__init__("social_nav_planner")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("max_speed",    DEF_MAX_SPD)
        self.declare_parameter("angular_speed", DEF_ANG_SPD)
        self.max_speed = self.get_parameter("max_speed").value
        self.ang_speed = self.get_parameter("angular_speed").value

        # ── State ──────────────────────────────────────────────────────
        self.path: list[tuple[float, float]] = []
        self.path_idx    = 0
        self.robot_x     = 0.0
        self.robot_y     = 0.0
        self.robot_yaw   = 0.0
        self.pose_recv   = False
        self.humans: list[tuple[float, float]] = []
        self.hvels:  list[tuple[float, float]] = []
        self.scan_data   = None

        # Displayed state
        self.active_case   = "NONE"
        self.active_action = "Waiting for map + path…"
        self.target_speed  = self.max_speed
        self.give_lat      = 0.0   # lateral nudge for case4b

        # Case1 replan throttle
        self.last_replan_t = 0.0
        self.replan_cd     = 8.0   # seconds between case1 replans

        # ── Publishers ─────────────────────────────────────────────────
        self.cmd_pub    = self.create_publisher(Twist,       "/cmd_vel",             10)
        self.wt_pub     = self.create_publisher(PoseArray,   "/weight_zones",        10)
        self.rp_pub     = self.create_publisher(PoseStamped, "/replan_request",      10)
        self.mk_pub     = self.create_publisher(MarkerArray, "/social_nav_markers",  10)

        # ── Subscribers ────────────────────────────────────────────────
        self.create_subscription(PoseArray,   "/detected_humans",  self._humans_cb, 10)
        self.create_subscription(PoseArray,   "/human_velocities", self._hvel_cb,   10)
        self.create_subscription(PoseStamped, "/robot_pose",        self._pose_cb,  10)
        self.create_subscription(Path,        "/global_path",       self._path_cb,  10)
        self.create_subscription(LaserScan,   "/scan",              self._scan_cb,  10)

        # ── 10 Hz control loop ─────────────────────────────────────────
        self.create_timer(0.1, self._loop)
        self.get_logger().info(
            "Social nav planner started — pure planning test mode (10 Hz)")

    # ── Callbacks ──────────────────────────────────────────────────────

    def _humans_cb(self, msg: PoseArray):
        self.humans = [(p.position.x, p.position.y) for p in msg.poses]

    def _hvel_cb(self, msg: PoseArray):
        self.hvels  = [(p.position.x, p.position.y) for p in msg.poses]

    def _pose_cb(self, msg: PoseStamped):
        self.robot_x   = msg.pose.position.x
        self.robot_y   = msg.pose.position.y
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)
        self.pose_recv = True

    def _path_cb(self, msg: Path):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(self.path) > 1:
            d = math.hypot(self.path[0][0] - self.robot_x,
                           self.path[0][1] - self.robot_y)
            if d < WP_TOL:
                self.path = self.path[1:]
        self.path_idx = 0

    def _scan_cb(self, msg: LaserScan):
        self.scan_data = msg

    # ── Main control loop ───────────────────────────────────────────────

    def _loop(self):
        cmd = Twist()

        if not self.pose_recv:
            self.cmd_pub.publish(cmd)
            return

        # Emergency LiDAR stop
        if self._emergency():
            self.active_case   = "EMERGENCY"
            self.active_action = "LiDAR obstacle — STOPPED"
            self.cmd_pub.publish(cmd)
            self._pub_markers([])
            return

        # No path yet
        if not self.path:
            self.active_action = "No path — waiting for global_planner…"
            self.cmd_pub.publish(cmd)
            self._pub_markers([])
            return

        # Classify each human → pick action
        cases = self._classify_all()
        self.target_speed, self.give_lat, self.active_case, self.active_action = \
            self._decide(cases)

        # Follow path with computed target speed
        cmd = self._follow_path(self.target_speed)

        # Case4b lateral nudge: a slight angular push so the robot swerves
        # gently aside to let the faster human overtake
        if abs(self.give_lat) > 0.01:
            sign = 1.0 if self.give_lat > 0 else -1.0
            cmd.angular.z = max(-self.ang_speed,
                                min(self.ang_speed,
                                    cmd.angular.z + sign * 0.35))

        self.cmd_pub.publish(cmd)
        self._pub_markers(cases)

        self.get_logger().info(
            f"[{self.active_case}]  {self.active_action}"
            f"  spd={self.target_speed:.2f}m/s",
            throttle_duration_sec=2.0)

    # ── Case classification ─────────────────────────────────────────────

    def _classify_all(self) -> list[dict]:
        results = []
        for i, (hx, hy) in enumerate(self.humans):
            hvx, hvy = (self.hvels[i] if i < len(self.hvels) else (0.0, 0.0))
            results.append(self._classify_one(hx, hy, hvx, hvy))
        return results

    def _classify_one(self, hx, hy, hvx, hvy) -> dict:
        rel_x  = hx - self.robot_x
        rel_y  = hy - self.robot_y
        dist   = math.hypot(rel_x, rel_y)
        h_spd  = math.hypot(hvx, hvy)

        base = {"hx": hx, "hy": hy, "hvx": hvx, "hvy": hvy,
                "dist": dist, "h_spd": h_spd}

        if dist < 0.3:
            return {**base, "case": "EMERGENCY"}

        # ── Case 1: stationary ────────────────────────────────────────
        if h_spd < STATIC_THR:
            return {**base, "case": "case1", "hvx": 0.0, "hvy": 0.0, "h_spd": 0.0}

        to_hum  = (rel_x / dist, rel_y / dist)
        h_dir   = (hvx / h_spd, hvy / h_spd)
        r_dir   = (math.cos(self.robot_yaw), math.sin(self.robot_yaw))
        same_dir = r_dir[0] * h_dir[0] + r_dir[1] * h_dir[1]  # +1 = same, -1 = opp
        ahead_d  = rel_x * r_dir[0] + rel_y * r_dir[1]         # >0 = human in front
        approach = -(hvx * to_hum[0] + hvy * to_hum[1])        # >0 = approaching

        # ── Case 4: same direction ────────────────────────────────────
        if same_dir > SAME_DIR_THR:
            sub = "front" if ahead_d > 0 else "behind"
            # Relative speed of human along robot heading (>0 = human faster)
            rel_spd = (hvx * r_dir[0] + hvy * r_dir[1]) - self.max_speed
            return {**base, "case": "case4", "sub": sub,
                    "ahead": ahead_d > 0, "rel_spd": rel_spd}

        # ── Case 2: head-on — human approaching AND moving opposite to robot heading ──
        # along_rh < 0  means human velocity has a component opposing the robot's heading.
        # This prevents a perpendicular crossing human from being misclassified as head-on.
        along_rh = hvx * r_dir[0] + hvy * r_dir[1]  # >0 same dir, <0 opposing
        if approach > APPROACH_THR and along_rh < -APPROACH_THR:
            return {**base, "case": "case2", "approach_rate": approach}

        # ── Case 3: crossing (default for moving, non-aligned human) ──
        cone   = self._collision_cone(hx, hy, hvx, hvy)
        timing = self._crossing_timing(hx, hy, hvx, hvy)
        return {**base, "case": "case3", "cone": cone, "timing": timing}

    # ── Action decision ─────────────────────────────────────────────────

    def _decide(self, cases: list[dict]):
        """Pick the highest-priority action. Returns (speed, lat, case_str, action_str)."""
        if not cases:
            return self.max_speed, 0.0, "NONE", "No human detected — full speed"

        # ── Real-time personal zone guard (highest priority) ──────────────
        # Regardless of case logic, never let any human get closer than PERSONAL_RAD.
        for cd in cases:
            if cd["dist"] < PERSONAL_RAD:
                return MIN_SPD, 0.0, cd["case"], \
                    f"PERSONAL ZONE ({cd['dist']:.2f}m) — STOP"

        # ── Social zone proximity scaling (safety net) ──────────────────
        # When a human is inside SOCIAL_RAD, cap the maximum speed the
        # case-specific handlers can return.  This acts as a safety net
        # even if crossing/timing calculations are off.
        closest_dist = min((cd["dist"] for cd in cases), default=999.0)
        if closest_dist < SOCIAL_RAD:
            prox_scale = max(0.30, closest_dist / SOCIAL_RAD)
            self._proximity_max = self.max_speed * prox_scale
        else:
            self._proximity_max = self.max_speed
        # Priority order (lower number = higher priority)
        PRIO = {"EMERGENCY": 0, "case1": 1, "case2": 2,
                "case3_conflict": 3, "case4b": 3, "case4a": 4,
                "case3_safe": 5, "NONE": 9}

        best = (self.max_speed, 0.0, "NONE", "No threat")
        best_p = PRIO["NONE"]

        for cd in cases:
            c = cd["case"]

            if c == "EMERGENCY":
                return 0.0, 0.0, "EMERGENCY", "Too close — STOPPED"

            elif c == "case1":
                spd, lat, act = self._act_case1(cd)
                p = PRIO["case1"]

            elif c == "case2":
                spd, lat, act = self._act_case2(cd)
                p = PRIO["case2"]

            elif c == "case3":
                spd, lat, act = self._act_case3(cd)
                conflict = "SLOW" in act or "PASS" in act
                p = PRIO["case3_conflict" if conflict else "case3_safe"]
                c = "case3"

            elif c == "case4":
                spd, lat, act = self._act_case4(cd)
                sub = cd.get("sub", "front")
                p = PRIO["case4b" if sub == "behind" else "case4a"]
                c = f"case4_{sub}"

            else:
                continue

            if p < best_p:
                best = (spd, lat, c, act)
                best_p = p

        # Apply proximity safety cap
        spd, lat, c, act = best
        if hasattr(self, '_proximity_max'):
            spd = min(spd, self._proximity_max)
        return (spd, lat, c, act)

    # ── Per-case action handlers ────────────────────────────────────────

    def _act_case1(self, cd) -> tuple[float, float, str]:
        """Static blocker → publish weight zone + request reroute."""
        self._pub_weight_zone(cd["hx"], cd["hy"], radius=1.2, weight=20.0)
        now = time.time()
        if now - self.last_replan_t > self.replan_cd:
            self.last_replan_t = now
            self._send_replan()
        return 0.0, 0.0, "1: Static blocker — rerouting"

    def _act_case2(self, cd) -> tuple[float, float, str]:
        """Head-on approaching → slow / stop."""
        dist = cd["dist"]
        rate = cd["approach_rate"]
        # Combined approach speed = robot_speed + human approach_rate
        combined = rate + self.max_speed
        ttc = dist / max(combined, 0.1)

        if dist < PERSONAL_RAD:
            return 0.0, 0.0, f"2: Head-on STOP (d={dist:.1f}m)"

        if dist < SOCIAL_RAD:
            factor = (dist - PERSONAL_RAD) / (SOCIAL_RAD - PERSONAL_RAD)
            spd = max(MIN_SPD, self.max_speed * factor * 0.5)
            return spd, 0.0, f"2: Head-on SLOW {spd:.2f}m/s"

        spd = self.max_speed * 0.6
        return spd, 0.0, f"2: Head-on approaching (d={dist:.1f}m)"

    def _act_case3(self, cd) -> tuple[float, float, str]:
        """Crossing → compute collision cone; slow to pass BEHIND,
        while ALWAYS staying outside the human's personal zone."""
        cone   = cd.get("cone", {})
        timing = cd.get("timing", {})

        # If no path crossing exists, fall back to raw cone check
        if not timing.get("exists", False):
            if cone.get("collision", False):
                t_exit = cone.get("t_exit")
                if t_exit and t_exit > 0:
                    t_enter = cone.get("t_enter") or 0.0
                    d_entry = self.max_speed * t_enter
                    v_safe  = max(MIN_SPD, d_entry / (t_exit + CROSS_BUF))
                    return v_safe, 0.0, \
                        f"Crossing (cone) — SLOW {v_safe:.2f}m/s"
            return self.max_speed, 0.0, "Crossing — clear"

        d_cross     = timing["dist_to_cross"]    # metres along robot path
        t_h_clears  = timing["t_human_clears"]   # seconds

        if d_cross < 0.0:
            return self.max_speed, 0.0, "Crossing behind — safe"
        if d_cross > 9.0:
            return self.max_speed, 0.0, f"Crossing far ({d_cross:.1f}m)"

        # Always compute the safe speed: arrive AFTER human’s PERSONAL
        # zone has fully cleared the crossing point.
        # t_h_clears already uses PERSONAL_RAD (not COLL_RAD).
        v_safe = d_cross / max(t_h_clears + CROSS_BUF, 0.01)
        v_safe = max(MIN_SPD, min(self.max_speed, v_safe))

        if v_safe >= self.max_speed * 0.98:
            # Defense-in-depth: even when timing says "safe", verify
            # with a social-radius collision cone.  The standard cone
            # uses COLL_RAD (0.5 m) which is too tight; using
            # SOCIAL_RAD (1.5 m) catches crossings even with moderate
            # localization drift (tested robust up to ~2.7 m offset).
            px = cd["hx"] - self.robot_x
            py = cd["hy"] - self.robot_y
            hvx, hvy = cd.get("hvx", 0.0), cd.get("hvy", 0.0)
            rv_x = v_safe * math.cos(self.robot_yaw) - hvx
            rv_y = v_safe * math.sin(self.robot_yaw) - hvy
            a_q  = rv_x ** 2 + rv_y ** 2
            b_q  = -2.0 * (px * rv_x + py * rv_y)
            c_q  = px * px + py * py - SOCIAL_RAD ** 2
            disc = b_q ** 2 - 4.0 * a_q * c_q
            if disc >= 0 and a_q > 1e-8:
                sq = math.sqrt(disc)
                t1 = (-b_q - sq) / (2.0 * a_q)
                t2 = (-b_q + sq) / (2.0 * a_q)
                if t1 > 0.0 or t2 > 0.0:
                    # Robot will enter human's social zone — slow down
                    v_cone = self.max_speed * 0.50
                    return v_cone, 0.0, \
                        f"Crossing — social-cone SLOW {v_cone:.2f}m/s"
            return self.max_speed, 0.0, \
                f"Crossing — safe (d={d_cross:.1f}m)"

        return v_safe, 0.0, \
            f"Crossing — SLOW {v_safe:.2f}m/s (clears t={t_h_clears:.1f}s)"

    def _act_case4(self, cd) -> tuple[float, float, str]:
        """Same direction → no-overtake (front) or give-way (behind)."""
        is_front = cd.get("ahead", True)
        dist     = cd["dist"]
        h_spd    = cd["h_spd"]
        rel_spd  = cd.get("rel_spd", 0.0)   # >0 = human faster than robot

        if is_front:
            # Human is ahead — do NOT overtake
            if dist < OVERTAKE_WARN:
                target = min(self.max_speed, h_spd * 0.95)
                target = max(MIN_SPD, target)
                return target, 0.0, \
                    f"4a: Following {target:.2f}m/s (d={dist:.1f}m)"
            return self.max_speed, 0.0, \
                f"4a: Ahead far (d={dist:.1f}m)"
        else:
            # Human behind — check if it's faster and approaching
            if rel_spd > 0.05 and dist < GIVE_WAY_DIST:
                slow = self.max_speed * 0.40
                return slow, 0.25, f"4b: GIVE WAY (Δv={rel_spd:.2f})"
            if rel_spd > 0.05 and dist < GIVE_WAY_DIST * 2.0:
                return self.max_speed * 0.70, 0.0, \
                    f"4b: Preparing give-way (d={dist:.1f}m)"
            return self.max_speed, 0.0, f"4b: Behind (d={dist:.1f}m)"

    # ── Collision cone computation ──────────────────────────────────────

    def _collision_cone(self, hx, hy, hvx, hvy) -> dict:
        """Velocity-obstacle collision cone.

        Computes whether the current robot velocity Vr is inside the
        velocity obstacle VO(Vh) for the detected human.

        The relative velocity Vrel = Vr - Vh traces out the cone:
          cone apex at origin, opening toward (H - R), half-angle α = arcsin(ρ/d).

        Returns:
          collision  — True if relative velocity direction is inside cone
          half_angle — α (radians)
          bearing    — direction from robot to human (radians)
          distance   — metres
          t_enter    — seconds until collision zone entered (if on course)
          t_exit     — seconds until collision zone exited
        """
        px = hx - self.robot_x
        py = hy - self.robot_y
        d  = math.hypot(px, py)

        if d < COLL_RAD:
            return {"collision": True, "half_angle": math.pi / 2.0,
                    "bearing": math.atan2(py, px), "distance": d,
                    "t_enter": 0.0, "t_exit": 0.5}

        alpha   = math.asin(min(1.0, COLL_RAD / d))
        bearing = math.atan2(py, px)

        # Relative velocity (robot w.r.t. human) — Vr − Vh
        rv_x = self.max_speed * math.cos(self.robot_yaw) - hvx
        rv_y = self.max_speed * math.sin(self.robot_yaw) - hvy
        rv_d = math.hypot(rv_x, rv_y)

        collision = False
        angle_diff = 0.0
        if rv_d > 0.01:
            rel_dir    = math.atan2(rv_y, rv_x)
            angle_diff = rel_dir - bearing
            while angle_diff >  math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi
            collision  = abs(angle_diff) < alpha

        # Time-to-collision quadratic:
        #   |p(t)|² = |p₀ − t·Vrel|² = ρ²
        #   a·t² − 2(p·Vrel)t + (|p|²−ρ²) = 0    (note: p decreases by Vrel*t)
        a = rv_x ** 2 + rv_y ** 2
        b = -2.0 * (px * rv_x + py * rv_y)
        c = d ** 2 - COLL_RAD ** 2
        disc = b ** 2 - 4.0 * a * c
        t_enter, t_exit = None, None
        if disc >= 0 and a > 1e-8:
            sq = math.sqrt(disc)
            t1 = (-b - sq) / (2.0 * a)
            t2 = (-b + sq) / (2.0 * a)
            if t1 > 0:
                t_enter, t_exit = t1, t2
            elif t2 > 0:
                t_enter, t_exit = 0.0, t2

        return {"collision": collision, "half_angle": alpha, "bearing": bearing,
                "distance": d, "t_enter": t_enter, "t_exit": t_exit,
                "angle_diff": angle_diff}

    def _crossing_timing(self, hx, hy, hvx, hvy) -> dict:
        """Find where the human's trajectory crosses the robot's forward path.

        Uses direction toward the next A* waypoint (not raw robot_yaw) so the
        crossing estimate stays accurate when the path curves.

        Keeps tracking the crossing even after the human has parametrically
        passed it, as long as the human body (radius = COLL_RAD) still
        physically overlaps the crossing point.
        """
        # ── Direction: toward next waypoint (falls back to robot_yaw) ──────
        if self.path and self.path_idx < len(self.path):
            tx, ty = self.path[self.path_idx]
            wpd_x  = tx - self.robot_x
            wpd_y  = ty - self.robot_y
            wd = math.hypot(wpd_x, wpd_y)
            rd = (wpd_x / wd, wpd_y / wd) if wd > 0.2 else \
                 (math.cos(self.robot_yaw), math.sin(self.robot_yaw))
        else:
            rd = (math.cos(self.robot_yaw), math.sin(self.robot_yaw))

        dx = hx - self.robot_x
        dy = hy - self.robot_y

        # det = | rd[0]  -hvx |
        #       | rd[1]  -hvy |
        det = rd[0] * (-hvy) - (-hvx) * rd[1]

        if abs(det) < 0.001:
            return {"exists": False}   # paths parallel — no crossing

        t = (dx * (-hvy) - (-hvx) * dy) / det   # metres along robot path
        s = (rd[0] * dy  -  rd[1] * dx) / det   # seconds for human to reach crossing

        h_spd = math.hypot(hvx, hvy)
        if h_spd < 0.01:
            return {"exists": False}

        if t < 0.05:
            return {"exists": False}   # crossing is behind robot

        cross_x = self.robot_x + t * rd[0]
        cross_y = self.robot_y + t * rd[1]

        if s >= 0.0:
            # Human hasn't reached crossing yet — normal case.
            # Wait until human is PERSONAL_RAD past the crossing (social circle clearance).
            t_clears = s + PERSONAL_RAD / h_spd
        else:
            # Human has passed the crossing parametrically.
            # Check if the human body still physically overlaps the crossing
            # within PERSONAL_RAD (social circle, not just physical body).
            dist_h_to_cross = math.hypot(hx - cross_x, hy - cross_y)
            if dist_h_to_cross >= PERSONAL_RAD:
                # Human has fully cleared social zone — no conflict
                return {"exists": False, "human_past": True}
            # Still inside personal zone: time to travel remaining distance
            remaining = PERSONAL_RAD - dist_h_to_cross
            t_clears  = remaining / h_spd

        return {
            "exists":          True,
            "cross_x":         cross_x,
            "cross_y":         cross_y,
            "dist_to_cross":   t,           # metres — robot needs to travel this far
            "t_human_arrives": max(0.0, s), # seconds
            "t_human_clears":  t_clears,    # seconds — when human has cleared
        }

    # ── Path following ──────────────────────────────────────────────────

    def _follow_path(self, target_spd: float) -> Twist:
        cmd = Twist()
        if self.path_idx >= len(self.path):
            return cmd

        # If the social planner says STOP, honour it immediately
        if target_spd < MIN_SPD:
            return cmd

        tx, ty = self.path[self.path_idx]
        dx, dy = tx - self.robot_x, ty - self.robot_y
        dist   = math.hypot(dx, dy)

        if dist < WP_TOL:
            self.path_idx += 1
            if self.path_idx >= len(self.path):
                self.get_logger().info("Goal reached!", throttle_duration_sec=5.0)
                return cmd
            tx, ty = self.path[self.path_idx]
            dx, dy = tx - self.robot_x, ty - self.robot_y
            dist   = math.hypot(dx, dy)

        ang = math.atan2(dy, dx) - self.robot_yaw
        while ang >  math.pi: ang -= 2 * math.pi
        while ang < -math.pi: ang += 2 * math.pi

        if abs(ang) > 0.5:          # large heading error → rotate first
            cmd.angular.z = self.ang_speed if ang > 0 else -self.ang_speed
            cmd.linear.x  = min(target_spd, MIN_SPD)
        elif abs(ang) > 0.12:       # moderate heading error → creep + steer
            cmd.angular.z = ang * 0.8
            cmd.linear.x  = max(MIN_SPD, target_spd * 0.4)
        else:                       # on course → full (social) speed
            cmd.linear.x  = min(target_spd, max(MIN_SPD,
                                 target_spd * min(1.0, dist / 0.6)))
            cmd.angular.z = ang * 0.8
        return cmd

    # ── Emergency check ─────────────────────────────────────────────────

    def _emergency(self) -> bool:
        if self.scan_data is None:
            return False
        count = 0
        for i, r in enumerate(self.scan_data.ranges):
            if self.scan_data.range_min < r < EMERG_DIST:
                a = self.scan_data.angle_min + i * self.scan_data.angle_increment
                if abs(a) < EMERG_ARC:
                    count += 1
                    if count >= 3:
                        return True
        return False

    # ── Case1 helpers ───────────────────────────────────────────────────

    def _pub_weight_zone(self, wx, wy, radius=1.2, weight=20.0):
        z = PoseArray()
        z.header.frame_id = "map"
        z.header.stamp    = self.get_clock().now().to_msg()
        p = Pose()
        p.position.x = wx
        p.position.y = wy
        p.position.z = radius
        p.orientation.w = weight
        z.poses.append(p)
        self.wt_pub.publish(z)

    def _send_replan(self):
        req = PoseStamped()
        req.header.frame_id = "map"
        req.header.stamp    = self.get_clock().now().to_msg()
        req.pose.position.x = self.robot_x
        req.pose.position.y = self.robot_y
        req.pose.orientation.w = 1.0
        self.rp_pub.publish(req)
        self.get_logger().info(
            f"Case1: replan from ({self.robot_x:.1f}, {self.robot_y:.1f})")

    # ══════════════════════════════════════════════════════════════════════
    #  RViz marker helpers
    # ══════════════════════════════════════════════════════════════════════

    def _pub_markers(self, cases: list[dict]):
        ma   = MarkerArray()
        _mid = [0]
        stamp = self.get_clock().now().to_msg()

        # Clear previous frame
        del_m = Marker(); del_m.action = Marker.DELETEALL
        ma.markers.append(del_m)

        # ── Per-human markers ──────────────────────────────────────────
        for cd in cases:
            hx, hy   = cd["hx"], cd["hy"]
            hvx, hvy = cd.get("hvx", 0.0), cd.get("hvy", 0.0)
            cone     = cd.get("cone", {})
            timing   = cd.get("timing", {})

            # Social rings
            ma.markers.append(self._ring(
                hx, hy, SOCIAL_RAD,   stamp, _mid, _col(0.3, 0.6, 1.0, 0.8), "social_ring"))
            ma.markers.append(self._ring(
                hx, hy, PERSONAL_RAD, stamp, _mid, _col(1.0, 0.5, 0.0, 0.9), "personal_ring"))

            # Human body sphere
            ma.markers.append(self._sphere(
                hx, hy, 1.0, 0.55, stamp, _mid, _col(1.0, 0.4, 0.0), "human_sphere"))

            # Human velocity arrow
            h_spd = cd.get("h_spd", 0.0)
            if h_spd > 0.01:
                scale = 3.5   # amplify for visibility
                ma.markers.append(self._arrow(
                    hx, hy, hx + hvx * scale, hy + hvy * scale,
                    stamp, _mid, _col(1.0, 0.5, 0.0), "human_vel"))
                # Predicted human trajectory — smooth (0.5 s steps, 5 s)
                pts = [(hx + hvx * dt * 0.5, hy + hvy * dt * 0.5)
                       for dt in range(11)]
                ma.markers.append(self._line(
                    pts, stamp, _mid, _col(1.0, 0.65, 0.0, 0.7), "human_traj"))

            # Collision cone (shown for all moving humans)
            if cone and "half_angle" in cone:
                bearing = cone["bearing"]
                alpha   = cone["half_angle"]
                dist_h  = cone["distance"]
                collide = cone.get("collision", False)
                ccol    = (_col(1.0, 0.0, 0.0, 0.40) if collide
                           else _col(1.0, 1.0, 0.0, 0.22))
                ma.markers.append(self._cone(
                    self.robot_x, self.robot_y,
                    bearing, alpha, dist_h * 1.25,
                    stamp, _mid, ccol, "cone"))

            # Crossing × marker (case3)
            if timing.get("exists", False):
                cx, cy = timing["cross_x"], timing["cross_y"]
                ma.markers.append(self._cross(cx, cy, stamp, _mid, "crossing"))

            # Human label (compact)
            ma.markers.append(self._text(
                hx, hy + 0.6, 1.8,
                f"H  vel={h_spd:.2f}m/s",
                stamp, _mid, 0.22, _col(1.0, 0.7, 0.2), "human_label"))

        # ── Robot velocity arrow (green) ────────────────────────────────
        rv_x = self.target_speed * math.cos(self.robot_yaw)
        rv_y = self.target_speed * math.sin(self.robot_yaw)
        ma.markers.append(self._arrow(
            self.robot_x, self.robot_y,
            self.robot_x + rv_x * 4.0,
            self.robot_y + rv_y * 4.0,
            stamp, _mid, _col(0.1, 1.0, 0.2), "robot_vel"))

        # Robot predicted trajectory — smooth (0.5 s steps, 3 s)
        rpts = [(self.robot_x + rv_x * dt * 0.5,
                 self.robot_y + rv_y * dt * 0.5) for dt in range(7)]
        ma.markers.append(self._line(
            rpts, stamp, _mid, _col(0.1, 1.0, 0.2, 0.5), "robot_traj"))

        # ── Decision banner — single compact line ───────────────────────
        banner_col = self._case_color()
        spd_str = f"{self.target_speed:.2f}"
        dist_str = f"{cases[0]['dist']:.1f}" if cases else "—"
        ma.markers.append(self._text(
            self.robot_x, self.robot_y - 0.4, 1.6,
            f"{self.active_case}  spd {spd_str}  d={dist_str}m",
            stamp, _mid, 0.25, banner_col, "decision_text"))

        self.mk_pub.publish(ma)

    # ── Marker construction helpers ─────────────────────────────────────

    def _sphere(self, x, y, z, r, stamp, mid, color, ns) -> Marker:
        m = Marker()
        m.header.frame_id = "map"; m.header.stamp = stamp
        m.ns = ns; m.id = mid[0]; mid[0] += 1
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = r
        m.color = color
        return m

    def _arrow(self, x0, y0, x1, y1, stamp, mid, color, ns) -> Marker:
        m = Marker()
        m.header.frame_id = "map"; m.header.stamp = stamp
        m.ns = ns; m.id = mid[0]; mid[0] += 1
        m.type = Marker.ARROW; m.action = Marker.ADD
        m.points = [Point(x=x0, y=y0, z=0.15), Point(x=x1, y=y1, z=0.15)]
        m.scale.x = 0.07; m.scale.y = 0.20; m.scale.z = 0.0
        m.color = color
        return m

    def _text(self, x, y, z, text, stamp, mid, scale, color, ns) -> Marker:
        m = Marker()
        m.header.frame_id = "map"; m.header.stamp = stamp
        m.ns = ns; m.id = mid[0]; mid[0] += 1
        m.type = Marker.TEXT_VIEW_FACING; m.action = Marker.ADD
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.z = scale
        m.text = text
        m.color = color
        m.lifetime.sec = 0; m.lifetime.nanosec = 300_000_000  # 300 ms auto-expire
        return m

    def _cone(self, rx, ry, bearing, half_angle, length,
              stamp, mid, color, ns) -> Marker:
        """Fan of triangles representing the collision cone."""
        m = Marker()
        m.header.frame_id = "map"; m.header.stamp = stamp
        m.ns = ns; m.id = mid[0]; mid[0] += 1
        m.type = Marker.TRIANGLE_LIST; m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color = color
        N = 12
        apex = Point(x=rx, y=ry, z=0.05)
        prev = None
        for i in range(N + 1):
            a  = bearing - half_angle + (2 * half_angle / N) * i
            pt = Point(x=rx + length * math.cos(a),
                       y=ry + length * math.sin(a), z=0.05)
            if prev is not None:
                m.points.extend([apex, prev, pt])
                fade = _col(color.r, color.g, color.b,
                            color.a * (1.0 - 0.4 * i / N))
                m.colors.extend([fade, fade, fade])
            prev = pt
        return m

    def _ring(self, cx, cy, r, stamp, mid, color, ns) -> Marker:
        m = Marker()
        m.header.frame_id = "map"; m.header.stamp = stamp
        m.ns = ns; m.id = mid[0]; mid[0] += 1
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.05; m.color = color
        N = 48
        for i in range(N + 1):
            a = 2 * math.pi * i / N
            m.points.append(Point(x=cx + r * math.cos(a),
                                  y=cy + r * math.sin(a), z=0.05))
        return m

    def _cross(self, cx, cy, stamp, mid, ns) -> Marker:
        m = Marker()
        m.header.frame_id = "map"; m.header.stamp = stamp
        m.ns = ns; m.id = mid[0]; mid[0] += 1
        m.type = Marker.LINE_LIST; m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.09; m.color = _col(1.0, 0.0, 0.0, 0.95)
        r = 0.35
        m.points = [
            Point(x=cx - r, y=cy - r, z=0.12), Point(x=cx + r, y=cy + r, z=0.12),
            Point(x=cx + r, y=cy - r, z=0.12), Point(x=cx - r, y=cy + r, z=0.12),
        ]
        return m

    def _line(self, pts, stamp, mid, color, ns) -> Marker:
        m = Marker()
        m.header.frame_id = "map"; m.header.stamp = stamp
        m.ns = ns; m.id = mid[0]; mid[0] += 1
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.05; m.color = color
        m.points = [Point(x=p[0], y=p[1], z=0.10) for p in pts]
        return m

    def _case_color(self) -> ColorRGBA:
        c = self.active_case.lower()
        act = self.active_action.lower()
        if "emergency" in c:          return _col(1.0, 0.0, 0.0)
        if "case1" in c:              return _col(1.0, 0.4, 0.0)
        if "case2" in c:              return _col(1.0, 0.6, 0.0)
        if "case3" in c:
            if "slow" in act or "conflict" in act:
                return _col(1.0, 0.3, 0.0)
            return _col(0.9, 0.9, 0.0)
        if "4a" in c:                 return _col(0.2, 0.8, 1.0)
        if "4b" in c:                 return _col(0.4, 0.6, 1.0)
        return _col(0.5, 1.0, 0.5)


def main():
    rclpy.init()
    node = SocialNavPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

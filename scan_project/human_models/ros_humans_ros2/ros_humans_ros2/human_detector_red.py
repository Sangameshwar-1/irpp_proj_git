#!/usr/bin/env python3
"""
Human Detector — Red Shape Detection (Simplified)
==================================================
Detects RED-coloured shapes from the rover's 4 camera feeds.

This is a simplified version that only looks for red colour blobs instead
of the full white-body + red-head pipeline.  It estimates each detection's
position, direction, and speed using pinhole projection + tracking.

Detection pipeline (per camera frame)
--------------------------------------
1. Convert to HSV colour space.
2. Threshold for RED (two ranges: H ≈ 0–10 and H ≈ 170–180, high S, high V).
3. Morphological open/close to clean noise.
4. Find contours, filter by area.
5. Estimate range from contour size (pinhole model).
6. Estimate bearing from centroid column.
7. Convert (bearing, range) → world-frame (x, y).
8. Track across frames to compute velocity.

Topics
------
  Subscribes:
    /camera/front/image  (sensor_msgs/Image)
    /camera/right/image
    /camera/back/image
    /camera/left/image
    /robot_pose          (geometry_msgs/PoseStamped)

  Publishes:
    /detected_humans          (geometry_msgs/PoseArray)   – positions
    /human_velocities         (geometry_msgs/PoseArray)   – velocities
    /detected_humans_markers  (visualization_msgs/MarkerArray)
"""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point
from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import Marker, MarkerArray

try:
    from cv_bridge import CvBridge
    import cv2
    HAS_CV = True
except ImportError:
    HAS_CV = False


# ─── Tracked target ──────────────────────────────────────────────────────────

class TrackedTarget:
    """Lightweight tracker for a single red detection with EMA smoothing."""
    _next_id = 0

    # Position smoothing factor: lower = smoother but more lag
    POS_ALPHA = 0.35
    # Velocity smoothing factor
    VEL_ALPHA = 0.3
    # Max plausible human speed (m/s) — reject larger jumps as noise
    MAX_HUMAN_SPEED = 1.5

    def __init__(self, x, y, stamp):
        self.id = TrackedTarget._next_id
        TrackedTarget._next_id += 1
        self.x = x
        self.y = y
        self.vx = 0.0
        self.vy = 0.0
        self.last_seen = stamp
        self.hits = 1
        # Keep raw detection for velocity calc (before smoothing)
        self._raw_x = x
        self._raw_y = y

    def update(self, x, y, stamp):
        dt = stamp - self.last_seen
        if dt > 0.01:
            # Velocity from raw (un-smoothed) positions
            nvx = (x - self._raw_x) / dt
            nvy = (y - self._raw_y) / dt
            spd = math.hypot(nvx, nvy)
            if spd < self.MAX_HUMAN_SPEED:
                self.vx = self.VEL_ALPHA * nvx + (1 - self.VEL_ALPHA) * self.vx
                self.vy = self.VEL_ALPHA * nvy + (1 - self.VEL_ALPHA) * self.vy
            # else: ignore — likely noise, keep old velocity

        self._raw_x = x
        self._raw_y = y

        # EMA-smooth position to reduce jitter / drift
        if self.hits > 1:
            self.x = self.POS_ALPHA * x + (1 - self.POS_ALPHA) * self.x
            self.y = self.POS_ALPHA * y + (1 - self.POS_ALPHA) * self.y
        else:
            self.x = x
            self.y = y

        self.last_seen = stamp
        self.hits += 1

    def predict(self, dt):
        return (self.x + self.vx * dt, self.y + self.vy * dt)

    def dist(self, x, y):
        return math.hypot(self.x - x, self.y - y)


# ─── Detection node ──────────────────────────────────────────────────────────

class HumanDetectorRed(Node):
    """Detect red shapes from camera images."""

    CAMERA_BEARINGS = {
        "front": 0.0,
        "right": -math.pi / 2.0,
        "back":  math.pi,
        "left":  math.pi / 2.0,
    }

    def __init__(self):
        super().__init__("human_detector_red")

        if not HAS_CV:
            self.get_logger().error(
                "OpenCV / cv_bridge not available — detector disabled")
            return

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("detection_rate", 5.0)
        self.declare_parameter("min_contour_area", 100)
        self.declare_parameter("max_detect_range", 8.0)
        self.declare_parameter("track_timeout", 3.0)
        # Red HSV thresholds (red wraps around H=0/180)
        self.declare_parameter("red_h_low1", 0)
        self.declare_parameter("red_h_high1", 10)
        self.declare_parameter("red_h_low2", 170)
        self.declare_parameter("red_h_high2", 180)
        self.declare_parameter("red_s_min", 50)
        self.declare_parameter("red_v_min", 80)
        # Pinhole camera model
        self.declare_parameter("cam_focal_px", 320.0)
        self.declare_parameter("reference_size", 0.5)    # metres — body WIDTH (diameter)
        self.declare_parameter("reference_height", 1.7)  # metres — body HEIGHT (for tall blobs)
        self.declare_parameter("cam_hfov_deg", 90.0)
        self.declare_parameter("show_debug_window", True)
        # LiDAR fusion: prefer LiDAR range when available
        self.declare_parameter("use_lidar_fusion", True)
        self.declare_parameter("lidar_bearing_tolerance", 0.15)  # rad — match window
        self.declare_parameter("publish_timeout", 0.6)  # only publish targets seen within this many seconds

        self.det_rate = self.get_parameter("detection_rate").value
        self.min_area = self.get_parameter("min_contour_area").value
        self.max_range = self.get_parameter("max_detect_range").value
        self.timeout = self.get_parameter("track_timeout").value
        self.rh1 = self.get_parameter("red_h_low1").value
        self.rh2 = self.get_parameter("red_h_high1").value
        self.rh3 = self.get_parameter("red_h_low2").value
        self.rh4 = self.get_parameter("red_h_high2").value
        self.rs_min = self.get_parameter("red_s_min").value
        self.rv_min = self.get_parameter("red_v_min").value
        self.focal = self.get_parameter("cam_focal_px").value
        self.ref_width = self.get_parameter("reference_size").value
        self.ref_height = self.get_parameter("reference_height").value
        self.hfov = math.radians(self.get_parameter("cam_hfov_deg").value)
        self.show_debug = self.get_parameter("show_debug_window").value
        self.use_lidar = self.get_parameter("use_lidar_fusion").value
        self.lidar_tol = self.get_parameter("lidar_bearing_tolerance").value
        self.publish_timeout = self.get_parameter("publish_timeout").value
        # Focal length will be recomputed from actual image width on first frame
        self.focal_computed = False

        # ── State ─────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.images = {"front": None, "right": None,
                       "back": None, "left": None}
        # Per-camera flag: True when a NEW image arrived since last _tick()
        self.image_new = {"front": False, "right": False,
                          "back": False, "left": False}
        # Per-camera stamped robot pose (captures pose AT image arrival)
        self.image_poses = {
            "front": (0.0, 0.0, 0.0),
            "right": (0.0, 0.0, 0.0),
            "back":  (0.0, 0.0, 0.0),
            "left":  (0.0, 0.0, 0.0),
        }
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.tracked: list[TrackedTarget] = []
        self.scan_data = None  # latest LiDAR scan
        # Reject detections within this fraction of image edge (camera overlap)
        self.edge_margin = 0.08  # 8% of image width on each side

        # ── Publishers ────────────────────────────────────────────────
        self.humans_pub = self.create_publisher(
            PoseArray, "/detected_humans", 10)
        self.vel_pub = self.create_publisher(
            PoseArray, "/human_velocities", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/detected_humans_markers", 10)

        # ── Subscribers ───────────────────────────────────────────────
        for cam in ("front", "right", "back", "left"):
            self.create_subscription(
                Image, f"/camera/{cam}/image",
                lambda msg, c=cam: self._img_cb(msg, c), 10)
        self.create_subscription(
            PoseStamped, "/robot_pose", self._pose_cb, 10)
        self.create_subscription(
            LaserScan, "/scan", self._scan_cb, 10)

        # ── Timer ─────────────────────────────────────────────────────
        self.create_timer(1.0 / self.det_rate, self._tick)
        self.get_logger().info(
            f"Red-shape detector started ({self.det_rate} Hz, "
            f"range ≤ {self.max_range} m)")

    # ── Callbacks ─────────────────────────────────────────────────────

    def _img_cb(self, msg, cam):
        try:
            self.images[cam] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_new[cam] = True  # mark as fresh for _tick()
            # Snapshot robot pose at the moment this image arrived
            # so we project detections from the correct position
            self.image_poses[cam] = (self.robot_x, self.robot_y, self.robot_yaw)
        except Exception as e:
            self.get_logger().error(f"Camera {cam}: {e}",
                                   throttle_duration_sec=5.0)

    def _pose_cb(self, msg):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def _scan_cb(self, msg):
        self.scan_data = msg

    # ── LiDAR range lookup ────────────────────────────────────────────

    def _lidar_range_at_bearing(self, world_bearing):
        """Get LiDAR range at a world-frame bearing.

        The LiDAR scan is in the robot body frame, so we subtract robot_yaw
        to get the scan-frame angle, then find the closest valid range.
        Returns the range in metres, or None if no valid reading.
        """
        if self.scan_data is None:
            return None

        scan = self.scan_data
        # Convert world bearing to robot-local bearing
        local_bearing = world_bearing - self.robot_yaw
        # Normalise to [-pi, pi]
        while local_bearing > math.pi:
            local_bearing -= 2 * math.pi
        while local_bearing < -math.pi:
            local_bearing += 2 * math.pi

        # Find the scan index closest to this bearing
        if local_bearing < scan.angle_min or local_bearing > scan.angle_max:
            return None

        idx_center = int((local_bearing - scan.angle_min) / scan.angle_increment)
        n_rays = len(scan.ranges)
        # Check a window of rays around the bearing
        window = max(1, int(self.lidar_tol / scan.angle_increment))
        best_range = None
        for di in range(-window, window + 1):
            idx = idx_center + di
            if 0 <= idx < n_rays:
                r = scan.ranges[idx]
                if scan.range_min < r < scan.range_max:
                    if best_range is None or r < best_range:
                        best_range = r
        return best_range

    # ── Detection tick ────────────────────────────────────────────────

    def _tick(self):
        if not HAS_CV:
            return
        now = time.time()
        raw = []

        for cam in self.images:
            # Only process cameras that received a NEW image since last tick
            # This prevents re-detecting the same stale frame over and over,
            # which would keep ghost humans alive indefinitely.
            if self.image_new[cam] and self.images[cam] is not None:
                pose = self.image_poses[cam]
                raw.extend(self._detect(self.images[cam], cam, pose))
                self.image_new[cam] = False  # consumed

        # ── Update tracker ──
        matched = set()
        for wx, wy in raw:
            best, best_d = None, 3.5  # wider association radius
            for t in self.tracked:
                if id(t) in matched:
                    continue  # don't match multiple detections to same track
                d = t.dist(wx, wy)
                if d < best_d:
                    best_d = d
                    best = t
            if best is not None:
                best.update(wx, wy, now)
                matched.add(id(best))
            else:
                self.tracked.append(TrackedTarget(wx, wy, now))

        # Prune stale
        self.tracked = [t for t in self.tracked
                        if now - t.last_seen < self.timeout]

        # Merge duplicates (two tracks within 1.5m are likely the same human)
        self._merge_duplicates(1.5)

        # ── Publish ──
        pos_msg = PoseArray()
        pos_msg.header.frame_id = "map"
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg = PoseArray()
        vel_msg.header = pos_msg.header

        for t in self.tracked:
            if t.hits < 2:
                continue
            # Only publish targets with recent evidence (within publish_timeout).
            # Stale targets stay in self.tracked for re-association but are NOT
            # sent to consumers — this eliminates ghost markers in RViz.
            if now - t.last_seen > self.publish_timeout:
                continue
            p = Pose()
            p.position.x = t.x
            p.position.y = t.y
            p.orientation.w = float(t.id)
            pos_msg.poses.append(p)

            v = Pose()
            v.position.x = t.vx
            v.position.y = t.vy
            v.orientation.w = float(t.id)
            vel_msg.poses.append(v)

        self.humans_pub.publish(pos_msg)
        self.vel_pub.publish(vel_msg)
        self._publish_markers()

        if pos_msg.poses and self.get_clock().now().nanoseconds % 2_000_000_000 < 200_000_000:
            self.get_logger().info(
                f"Detected {len(pos_msg.poses)} red target(s)")

        if self.show_debug:
            self._debug_window()

    # ── Core detection ────────────────────────────────────────────────

    def _detect(self, img, cam_name, pose):
        """Detect red blobs in a single camera image.
        Returns list of (world_x, world_y).

        Uses the robot pose captured at the time the image arrived (not the
        current pose) to eliminate drift when the robot is moving.

        Range estimation priority:
          1. LiDAR range at the detection bearing (most accurate)
          2. Pinhole model using blob WIDTH and body diameter reference
             (height-based estimation is unreliable because the 1.7 m body
              is vertically clipped by the 480 px frame at close range)
        """
        h, w = img.shape[:2]
        dets = []
        pose_x, pose_y, pose_yaw = pose

        # Compute focal length from actual image width (once)
        if not self.focal_computed:
            self.focal = (w / 2.0) / math.tan(self.hfov / 2.0)
            self.focal_computed = True
            self.get_logger().info(
                f"Focal length computed from image: {self.focal:.1f} px "
                f"(image {w}x{h}, HFOV {math.degrees(self.hfov):.0f}°)")

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Two red masks (red wraps around H=0/180)
        m1 = cv2.inRange(hsv,
                         np.array([self.rh1, self.rs_min, self.rv_min]),
                         np.array([self.rh2, 255, 255]))
        m2 = cv2.inRange(hsv,
                         np.array([self.rh3, self.rs_min, self.rv_min]),
                         np.array([self.rh4, 255, 255]))
        mask = cv2.bitwise_or(m1, m2)

        # Morphological cleanup
        kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kern, iterations=2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue

            x_r, y_r, bw, bh = cv2.boundingRect(cnt)
            if bw < 5 and bh < 5:
                continue

            # ── Reject detections near image edges (camera overlap zone) ──
            cx = x_r + bw / 2.0
            margin_px = w * self.edge_margin
            if cx < margin_px or cx > w - margin_px:
                continue  # skip — likely also seen by the adjacent camera

            # ── Bearing from centroid ──
            norm_x = (cx / w) - 0.5
            bearing_cam = norm_x * self.hfov
            cam_bear = self.CAMERA_BEARINGS[cam_name]
            # Use the pose AT capture time (not current pose)
            world_bear = pose_yaw + cam_bear + bearing_cam

            # ── Range estimation ──
            est_range = None

            # Method 1: LiDAR fusion (most accurate)
            if self.use_lidar:
                lidar_r = self._lidar_range_at_bearing(world_bear)
                if lidar_r is not None and 0.3 < lidar_r < self.max_range:
                    est_range = lidar_r

            # Method 2: Pinhole model — ALWAYS use blob WIDTH
            # The body height (1.7 m) gets clipped by the 480 px frame at
            # ranges < ~2 m (camera is at 0.3 m height, body is 0–1.7 m),
            # making bh unreliable.  Blob width maps to the consistent
            # body diameter (0.5 m) and is always fully visible.
            if est_range is None:
                if bw >= 5:
                    est_range = (self.ref_width * self.focal) / bw

            if est_range is None or est_range > self.max_range or est_range < 0.3:
                continue

            # ── World-frame position (using pose AT capture time) ──
            wx = pose_x + est_range * math.cos(world_bear)
            wy = pose_y + est_range * math.sin(world_bear)
            dets.append((wx, wy))

        return dets

    def _merge_duplicates(self, min_dist):
        """Merge tracked targets that are within min_dist of each other.

        Keeps the one with more hits (more confident estimate).
        """
        if len(self.tracked) < 2:
            return
        merged = []
        removed = set()
        for i, t1 in enumerate(self.tracked):
            if i in removed:
                continue
            for j in range(i + 1, len(self.tracked)):
                if j in removed:
                    continue
                t2 = self.tracked[j]
                if t1.dist(t2.x, t2.y) < min_dist:
                    # Keep the one with more observations
                    if t2.hits > t1.hits:
                        t1.x, t1.y = t2.x, t2.y
                        t1.vx = t2.vx
                        t1.vy = t2.vy
                        t1.hits = max(t1.hits, t2.hits)
                    removed.add(j)
            merged.append(t1)
        self.tracked = merged

    # ── Visualisation ─────────────────────────────────────────────────

    def _publish_markers(self):
        markers = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        stamp = self.get_clock().now().to_msg()
        now = time.time()
        mid = 0
        for t in self.tracked:
            if t.hits < 2:
                continue
            # Skip stale targets — no marker for ghosts
            if now - t.last_seen > self.publish_timeout:
                continue
            # Sphere at detection position
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "red_target"
            m.id = mid; mid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = t.x
            m.pose.position.y = t.y
            m.pose.position.z = 1.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.4
            m.color.r = 1.0; m.color.a = 0.8
            m.lifetime.sec = 1
            markers.markers.append(m)

            # Velocity arrow
            spd = math.hypot(t.vx, t.vy)
            if spd > 0.05:
                a = Marker()
                a.header.frame_id = "map"
                a.header.stamp = stamp
                a.ns = "red_velocity"
                a.id = mid; mid += 1
                a.type = Marker.ARROW
                a.action = Marker.ADD
                s = Point(); s.x = t.x; s.y = t.y; s.z = 0.5
                e = Point()
                px, py = t.predict(2.0)
                e.x = px; e.y = py; e.z = 0.5
                a.points = [s, e]
                a.scale.x = 0.08; a.scale.y = 0.15
                a.color.r = 1.0; a.color.g = 0.5; a.color.a = 0.8
                a.lifetime.sec = 1
                markers.markers.append(a)

            # ID label
            lbl = Marker()
            lbl.header.frame_id = "map"
            lbl.header.stamp = stamp
            lbl.ns = "red_id"
            lbl.id = mid; mid += 1
            lbl.type = Marker.TEXT_VIEW_FACING
            lbl.action = Marker.ADD
            lbl.pose.position.x = t.x
            lbl.pose.position.y = t.y
            lbl.pose.position.z = 1.8
            lbl.scale.z = 0.3
            lbl.color.r = 1.0; lbl.color.g = 1.0; lbl.color.a = 1.0
            lbl.text = f"R{t.id} v={spd:.1f}"
            lbl.lifetime.sec = 1
            markers.markers.append(lbl)

        self.marker_pub.publish(markers)

    def _debug_window(self):
        """Show 2×2 camera grid with red detections highlighted."""
        try:
            panels = []
            th, tw = 240, 320
            for cam in ("front", "right", "back", "left"):
                img = self.images[cam]
                if img is None:
                    p = np.zeros((th, tw, 3), dtype=np.uint8)
                    cv2.putText(p, f"{cam}: no img", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                else:
                    p = cv2.resize(img, (tw, th))
                    # Overlay red mask
                    hsv = cv2.cvtColor(p, cv2.COLOR_BGR2HSV)
                    m1 = cv2.inRange(hsv,
                                     np.array([self.rh1, self.rs_min, self.rv_min]),
                                     np.array([self.rh2, 255, 255]))
                    m2 = cv2.inRange(hsv,
                                     np.array([self.rh3, self.rs_min, self.rv_min]),
                                     np.array([self.rh4, 255, 255]))
                    mask = cv2.bitwise_or(m1, m2)
                    cnts, _ = cv2.findContours(
                        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for c in cnts:
                        if cv2.contourArea(c) > 30:
                            x, y, w, h = cv2.boundingRect(c)
                            cv2.rectangle(p, (x, y), (x+w, y+h),
                                          (0, 0, 255), 2)
                cv2.putText(p, cam.upper(), (5, th - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                panels.append(p)

            top = np.hstack([panels[0], panels[1]])
            bot = np.hstack([panels[2], panels[3]])
            grid = np.vstack([top, bot])
            now_t = time.time()
            n_active = sum(1 for t in self.tracked
                           if t.hits >= 2 and now_t - t.last_seen <= self.publish_timeout)
            n_total = sum(1 for t in self.tracked if t.hits >= 2)
            cv2.putText(grid, f"Active: {n_active}  (tracked: {n_total})",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 255), 2)
            cv2.imshow("Red Detection - 4 Cameras", grid)
            cv2.waitKey(1)
        except Exception:
            self.show_debug = False


def main():
    rclpy.init()
    node = HumanDetectorRed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

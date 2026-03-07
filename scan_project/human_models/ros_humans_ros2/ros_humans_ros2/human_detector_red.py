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

class KalmanTrackedTarget:
    """Kalman-filter tracker for a single red detection.

    State vector: [x, y, vx, vy] in world frame.
    Implements the full predict + update cycle described in the tracking spec.
    Exposes the same external interface as the old EMA tracker so the rest
    of the node code is unchanged.
    """
    _next_id = 0
    MAX_HUMAN_SPEED = 1.5   # m/s — reject faster jumps as sensor noise

    def __init__(self, x, y, stamp):
        self.id = KalmanTrackedTarget._next_id
        KalmanTrackedTarget._next_id += 1
        self.hits = 1
        self.last_seen = stamp

        # State vector [x, y, vx, vy]
        self.X = np.array([x, y, 0.0, 0.0], dtype=float)
        # Initial covariance — high uncertainty on velocity
        self.P = np.diag([0.5, 0.5, 2.0, 2.0])
        # Measurement matrix H: we observe x and y only
        self.H = np.array([[1., 0., 0., 0.],
                           [0., 1., 0., 0.]])
        # Measurement noise covariance R (LiDAR cluster accuracy ~0.1 m)
        self.R = np.diag([0.1, 0.1])
        # Process noise rate (motion uncertainty accumulated per second)
        self._Q_rate = np.diag([0.01, 0.01, 0.05, 0.05])

    # ── Read-only properties matching old tracker interface ──────────────────
    @property
    def x(self):
        return float(self.X[0])

    @property
    def y(self):
        return float(self.X[1])

    @property
    def vx(self):
        return float(self.X[2])

    @property
    def vy(self):
        return float(self.X[3])

    def dist(self, x, y):
        return math.hypot(self.X[0] - x, self.X[1] - y)

    def predict(self, dt):
        """Non-mutating position forecast — used for velocity arrows only."""
        return (self.X[0] + self.X[2] * dt,
                self.X[1] + self.X[3] * dt)

    # ── Internal Kalman steps ─────────────────────────────────────────────────
    def _kf_predict(self, dt):
        """Kalman prediction step: X = F*X,  P = F*P*F' + Q  (mutates state)."""
        F = np.array([[1., 0., dt, 0.],
                      [0., 1., 0., dt],
                      [0., 0., 1.,  0.],
                      [0., 0., 0.,  1.]])
        self.X = F @ self.X
        self.P = F @ self.P @ F.T + self._Q_rate * dt

    def update(self, x, y, stamp):
        """Full Kalman predict + update with new measurement (x, y)."""
        dt = max(stamp - self.last_seen, 0.001)

        # Reject implausible position jumps once the track is established
        if self.hits > 3:
            speed = math.hypot(x - self.X[0], y - self.X[1]) / dt
            if speed > self.MAX_HUMAN_SPEED:
                # Likely noise — propagate prediction only, skip measurement
                self._kf_predict(dt)
                self.last_seen = stamp
                return

        # 1. Prediction step (time update)
        self._kf_predict(dt)

        # 2. Update step (measurement update)
        z = np.array([x, y])
        innov = z - self.H @ self.X                       # innovation
        S = self.H @ self.P @ self.H.T + self.R           # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)          # Kalman gain
        self.X = self.X + K @ innov                        # updated state
        self.P = (np.eye(4) - K @ self.H) @ self.P        # updated covariance

        self.last_seen = stamp
        self.hits += 1


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
        self.declare_parameter("min_contour_area", 40)
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
        self.declare_parameter("reference_size", 0.5)    # metres — head WIDTH (diameter)
        self.declare_parameter("reference_height", 0.5)  # metres — head sphere diameter (visible red part)
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
        self.tracked: list[KalmanTrackedTarget] = []
        self.scan_data = None  # latest LiDAR scan
        # Reject detections within this fraction of image edge (camera overlap)
        self.edge_margin = 0.15  # 15% of image width on each side

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

    # ── LiDAR cluster-based position estimation ────────────────────────

    def _lidar_cluster_at_bearing(self, world_bearing, robot_x, robot_y, robot_yaw,
                                   window_deg=10.0):
        """Estimate human position from LiDAR using camera-guided clustering.

        Pipeline (per tracking spec steps 5–9):
          1. Convert world bearing to robot-local angle.
          2. Collect scan points within ±window_deg of that angle.
          3. Convert polar (r, α) to robot-frame Cartesian (lx, ly).
          4. Distance-based clustering: gap > 0.3 m starts a new cluster.
          5. Filter clusters by width: 0.1–0.8 m (human head/body).
          6. Select the cluster whose centroid angle is closest to the
             camera bearing.
          7. Convert cluster centroid from robot frame to world frame.

        `window_deg` is widened automatically for close-range/near-edge blobs
        where the camera bearing is less reliable.

        Returns (world_x, world_y) or None if no valid cluster found.
        """
        if self.scan_data is None:
            return None

        scan = self.scan_data
        # World bearing → robot-local bearing
        local_bearing = world_bearing - robot_yaw
        while local_bearing > math.pi:
            local_bearing -= 2 * math.pi
        while local_bearing < -math.pi:
            local_bearing += 2 * math.pi

        if local_bearing < scan.angle_min or local_bearing > scan.angle_max:
            return None

        # Angular window around camera detection (adaptive, caller-specified)
        window_rad = math.radians(window_deg)
        idx_min = max(0, int(
            (local_bearing - window_rad - scan.angle_min) / scan.angle_increment))
        idx_max = min(len(scan.ranges) - 1, int(
            (local_bearing + window_rad - scan.angle_min) / scan.angle_increment))

        # Step 6: convert valid scan rays to robot-frame Cartesian points
        pts = []  # list of (lx, ly) in robot frame
        for i in range(idx_min, idx_max + 1):
            r = scan.ranges[i]
            if not (scan.range_min < r < min(scan.range_max, self.max_range)):
                continue
            a = scan.angle_min + i * scan.angle_increment
            pts.append((r * math.cos(a), r * math.sin(a)))

        if not pts:
            return None

        # Step 7: distance-based clustering (gap threshold 0.3 m)
        clusters = []
        current = [pts[0]]
        for i in range(1, len(pts)):
            dx = pts[i][0] - pts[i - 1][0]
            dy = pts[i][1] - pts[i - 1][1]
            if math.hypot(dx, dy) < 0.3:    # same cluster
                current.append(pts[i])
            else:
                clusters.append(current)
                current = [pts[i]]
        clusters.append(current)

        # Step 8: filter by cluster width (human head/body: 0.1–0.8 m)
        # and select the cluster whose centroid direction best matches
        # the camera-derived bearing
        best_cluster = None
        best_angle_diff = float('inf')
        for clust in clusters:
            xs = [p[0] for p in clust]
            ys = [p[1] for p in clust]
            width = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            # Multi-point clusters must pass the width filter;
            # single-point clusters (head at long range) are always kept
            if len(clust) > 1 and not (0.1 <= width <= 0.8):
                continue
            # Step 9: cluster centroid
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)
            angle_diff = abs(math.atan2(cy, cx) - local_bearing)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            if angle_diff < best_angle_diff:
                best_angle_diff = angle_diff
                best_cluster = (cx, cy)

        if best_cluster is None:
            return None

        # ── Center correction ──────────────────────────────────────────
        # The LiDAR hits the NEAR FACE of the human body/head, not the
        # geometric centre.  Push the cluster centroid by one body radius
        # (≈0.22 m) in the camera bearing direction to recover the true
        # centre.  Without this, the Kalman filter sees the "near face"
        # drifting as the robot changes angle, generating spurious velocity.
        _HUMAN_RADIUS = 0.22   # m — conservative average of body (0.25) and head (0.25)
        lx = best_cluster[0] + _HUMAN_RADIUS * math.cos(local_bearing)
        ly = best_cluster[1] + _HUMAN_RADIUS * math.sin(local_bearing)

        # Robot frame → world frame (2-D rigid body transform)
        wx = robot_x + lx * math.cos(robot_yaw) - ly * math.sin(robot_yaw)
        wy = robot_y + lx * math.sin(robot_yaw) + ly * math.cos(robot_yaw)
        return wx, wy

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

        # ── Cluster raw detections ──
        # When multiple cameras see the same human, they produce slightly
        # different world positions.  Cluster nearby raw detections into
        # a single averaged point BEFORE feeding to the tracker.
        raw = self._cluster_raw(raw, radius=1.2)

        # ── Update tracker ──
        matched = set()
        for wx, wy in raw:
            best, best_d = None, 2.0  # tighter association radius
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
                self.tracked.append(KalmanTrackedTarget(wx, wy, now))

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
            if t.hits < 3:
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

        if pos_msg.poses:
            self.get_logger().info(
                f"Detected {len(pos_msg.poses)} red target(s)",
                throttle_duration_sec=5.0)

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
          2. Pinhole model using blob WIDTH and head diameter reference
             (only the head sphere is red; body is beige/neutral)
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

            # ── Reject if bounding box is clipped at the horizontal edges ──
            # Centroid-only check misses blobs whose EDGE is cut off.
            # A clipped bbox means the blob width is artificially small →
            # pinhole overestimates range; the bearing is also wrong.
            cx = x_r + bw / 2.0
            margin_px = w * self.edge_margin          # 15% margin for centroid
            bbox_margin = w * 0.05                    # 5% inner check for bbox edge
            if cx < margin_px or cx > w - margin_px:
                continue  # centroid in overlap zone
            if x_r < bbox_margin or (x_r + bw) > (w - bbox_margin):
                continue  # bbox physically touches the edge → clipped blob

            # ── Reject vertically clipped blobs (close-range partial head) ──
            # When the robot is very close the head fills the frame vertically
            # and gets clipped top/bottom → bw is wrong → bad pinhole range.
            vert_margin = 8  # pixels
            if y_r < vert_margin or (y_r + bh) > (h - vert_margin):
                continue

            # ── Aspect ratio filter (head sphere ≈ circular in image) ──
            # Extreme ratios indicate a partially-visible or clipped blob.
            aspect = bw / max(bh, 1)
            if not (0.35 <= aspect <= 3.0):
                continue

            # ── Bearing from centroid ──
            norm_x = (cx / w) - 0.5
            bearing_cam = norm_x * self.hfov
            cam_bear = self.CAMERA_BEARINGS[cam_name]
            # Use the pose AT capture time (not current pose)
            world_bear = pose_yaw + cam_bear + bearing_cam

            # ── Adaptive LiDAR window ──────────────────────────────────
            # blob_frac = fraction of frame width the blob occupies.
            # Large blob → human is close → bearing is less reliable →
            # widen the LiDAR search window so we don’t miss the cluster.
            blob_frac = bw / w
            edge_prox = min(cx, w - cx) / w   # 0 = at edge, 0.5 = centre
            if blob_frac > 0.10 or edge_prox < 0.30:
                lidar_window = 22.0   # wide: close range or near edge
            else:
                lidar_window = 10.0   # normal

            # ── Position estimation ─────────────────────────────────────
            world_pos = None

            # Method 1: LiDAR cluster (steps 5–9 of tracking spec)
            # Extracts scan points in ±lidar_window°, distance-clusters them,
            # filters by human body width, returns cluster centroid in
            # world frame.  Most accurate when LiDAR hits the human.
            if self.use_lidar:
                world_pos = self._lidar_cluster_at_bearing(
                    world_bear, pose_x, pose_y, pose_yaw,
                    window_deg=lidar_window)

            # Method 2: Pinhole model fallback
            # Only used when LiDAR cluster was not found AND the blob is
            # NOT large (large blob = close range = pinhole is least reliable
            # because the visible fraction of the head is unpredictable).
            if world_pos is None and bw >= 5 and blob_frac < 0.12:
                est_range = (self.ref_width * self.focal) / bw
                if 0.3 < est_range < self.max_range:
                    world_pos = (pose_x + est_range * math.cos(world_bear),
                                 pose_y + est_range * math.sin(world_bear))

            if world_pos is None:
                continue

            dets.append(world_pos)

        return dets

    @staticmethod
    def _cluster_raw(points, radius=1.2):
        """Cluster nearby raw detections and return centroid of each cluster.

        When multiple cameras see the same human, they produce slightly
        different world-frame positions.  This merges detections within
        `radius` metres into a single averaged point so the tracker only
        sees ONE detection per real human per tick.

        Uses simple greedy clustering (fast for small N).
        """
        if len(points) <= 1:
            return points

        used = [False] * len(points)
        clusters = []
        for i in range(len(points)):
            if used[i]:
                continue
            cx, cy = points[i]
            n = 1
            used[i] = True
            for j in range(i + 1, len(points)):
                if used[j]:
                    continue
                dx = points[j][0] - cx
                dy = points[j][1] - cy
                if math.hypot(dx, dy) < radius:
                    # Running centroid update
                    cx = (cx * n + points[j][0]) / (n + 1)
                    cy = (cy * n + points[j][1]) / (n + 1)
                    n += 1
                    used[j] = True
            clusters.append((cx, cy))
        return clusters

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
            if t.hits < 3:
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
                           if t.hits >= 3 and now_t - t.last_seen <= self.publish_timeout)
            n_total = sum(1 for t in self.tracked if t.hits >= 3)
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

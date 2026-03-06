#!/usr/bin/env python3
"""
human_detector_cv.py
====================
CV-based human detection node for the A* navigation system.

Detects humans modelled as WHITE cylinders (body) + RED spheres (head)
from the rover's 4 camera feeds.  Publishes detected human positions to
/detected_humans as a PoseArray, which the A* planner uses for dynamic
obstacle avoidance and replanning.

Detection pipeline (per camera frame)
--------------------------------------
1.  Convert to HSV colour space.
2.  Threshold for WHITE regions (high V, low S) → binary mask of candidate
    human-body pixels.
3.  Threshold for RED regions (H near 0 or 180, high S, high V) → binary
    mask of candidate human-head pixels.
4.  Morphological open/close to reduce noise on both masks.
5.  Find white contours (body); filter by aspect ratio, area, solidity.
6.  For each body candidate, look for a red contour (head) directly above
    it.  A confirmed human detection requires both body + head.
7.  Estimate bearing angle from the contour's centroid column.
8.  Estimate range from the contour height (pinhole projection model).
9.  Convert (bearing, range) into a world-frame (x, y) position using the
    rover's known pose, then publish.

A separate OpenCV debug window displays all four cameras with coloured
bounding boxes around body (green) and head (red) detections.

Topics
------
  Subscribes:
    /camera/front/image  (sensor_msgs/Image)
    /camera/right/image
    /camera/back/image
    /camera/left/image
    /odom              (nav_msgs/Odometry)

  Publishes:
    /detected_humans        (geometry_msgs/PoseArray)   – current positions
    /detected_humans_markers (visualization_msgs/MarkerArray) – RViz markers
    /human_velocities       (geometry_msgs/PoseArray)   – velocities as dx,dy in pose.position
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

try:
    from cv_bridge import CvBridge
    import cv2
    HAS_CV = True
except ImportError:
    HAS_CV = False


# ─── Tracked Human ───────────────────────────────────────────────────────────

class TrackedHuman:
    """Lightweight tracker for a single detected human."""
    _next_id = 0

    def __init__(self, x, y, stamp):
        self.id = TrackedHuman._next_id
        TrackedHuman._next_id += 1
        self.x = x
        self.y = y
        self.vx = 0.0          # velocity (m/s)
        self.vy = 0.0
        self.last_seen = stamp
        self.hits = 1          # consecutive detections
        self.age = 0           # frames since creation

    def update(self, x, y, stamp):
        dt = stamp - self.last_seen
        if dt > 0.01:
            alpha = 0.4        # exponential smoothing
            new_vx = (x - self.x) / dt
            new_vy = (y - self.y) / dt
            # Clamp unreasonable velocities (teleports / noise)
            spd = math.hypot(new_vx, new_vy)
            if spd < 3.0:      # humans walk < 2 m/s; allow some margin
                self.vx = alpha * new_vx + (1 - alpha) * self.vx
                self.vy = alpha * new_vy + (1 - alpha) * self.vy
        self.x = x
        self.y = y
        self.last_seen = stamp
        self.hits += 1

    def predict(self, dt):
        """Predict position dt seconds into the future."""
        return (self.x + self.vx * dt, self.y + self.vy * dt)

    def distance_to(self, x, y):
        return math.hypot(self.x - x, self.y - y)


# ─── Detection Node ──────────────────────────────────────────────────────────

class HumanDetectorCV(Node):
    """Detect white-cylinder humans from camera images."""

    # Camera mounting angles relative to the rover's heading (radians).
    # Front=0, Right=-π/2, Back=π, Left=π/2
    CAMERA_BEARINGS = {
        "front": 0.0,
        "right": -math.pi / 2.0,
        "back":  math.pi,
        "left":  math.pi / 2.0,
    }

    def __init__(self):
        super().__init__("human_detector_cv")

        if not HAS_CV:
            self.get_logger().error(
                "OpenCV / cv_bridge not installed – human detector disabled.")
            return

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("detection_rate", 5.0)      # Hz
        self.declare_parameter("min_contour_area", 300)     # px² (lowered for distant humans)
        self.declare_parameter("max_detect_range", 8.0)     # metres
        self.declare_parameter("track_timeout", 3.0)        # seconds
        self.declare_parameter("white_v_min", 180)          # HSV V lower bound (handles shadows)
        self.declare_parameter("white_s_max", 60)           # HSV S upper bound
        # Red head HSV thresholds (red wraps around H=0/180)
        self.declare_parameter("red_h_low1", 0)             # first red range lower H
        self.declare_parameter("red_h_high1", 10)            # first red range upper H
        self.declare_parameter("red_h_low2", 170)            # second red range lower H
        self.declare_parameter("red_h_high2", 180)           # second red range upper H
        self.declare_parameter("red_s_min", 50)              # min saturation for red (Gazebo lighting)
        self.declare_parameter("red_v_min", 80)              # min value for red
        # pinhole camera projection constants (tuned for Gazebo 640×480 cam, 90° HFoV)
        self.declare_parameter("cam_focal_px", 320.0)       # focal length in pixels
        self.declare_parameter("human_real_height", 1.7)    # metres
        self.declare_parameter("cam_hfov_deg", 90.0)        # horizontal FoV
        # Debug window
        self.declare_parameter("show_debug_window", True)   # OpenCV imshow window

        # Spawn pose (must match astar_path_planner)
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", -8.0)
        self.declare_parameter("spawn_yaw", 1.5708)

        self.detection_rate = self.get_parameter("detection_rate").value
        self.min_contour_area = self.get_parameter("min_contour_area").value
        self.max_range = self.get_parameter("max_detect_range").value
        self.track_timeout = self.get_parameter("track_timeout").value
        self.white_v_min = self.get_parameter("white_v_min").value
        self.white_s_max = self.get_parameter("white_s_max").value
        self.red_h_low1  = self.get_parameter("red_h_low1").value
        self.red_h_high1 = self.get_parameter("red_h_high1").value
        self.red_h_low2  = self.get_parameter("red_h_low2").value
        self.red_h_high2 = self.get_parameter("red_h_high2").value
        self.red_s_min   = self.get_parameter("red_s_min").value
        self.red_v_min   = self.get_parameter("red_v_min").value
        self.focal_px = self.get_parameter("cam_focal_px").value
        self.human_h = self.get_parameter("human_real_height").value
        self.cam_hfov = math.radians(self.get_parameter("cam_hfov_deg").value)
        self.show_debug = self.get_parameter("show_debug_window").value

        self.spawn_x = self.get_parameter("spawn_x").value
        self.spawn_y = self.get_parameter("spawn_y").value
        self.spawn_yaw = self.get_parameter("spawn_yaw").value
        self.spawn_cos = math.cos(self.spawn_yaw)
        self.spawn_sin = math.sin(self.spawn_yaw)

        # ── State ─────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.camera_images = {"front": None, "right": None,
                              "back": None, "left": None}
        # Debug drawing: stores (cam_name → list of detection dicts)
        self.debug_detections = {"front": [], "right": [],
                                 "back": [], "left": []}
        self.robot_x = self.spawn_x
        self.robot_y = self.spawn_y
        self.robot_yaw = self.spawn_yaw
        self.tracked_humans: list[TrackedHuman] = []

        # ── Publishers ────────────────────────────────────────────────────
        self.humans_pub = self.create_publisher(PoseArray, "/detected_humans", 10)
        self.vel_pub = self.create_publisher(PoseArray, "/human_velocities", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/detected_humans_markers", 10)

        # ── Subscribers ───────────────────────────────────────────────────
        for cam in ("front", "right", "back", "left"):
            self.create_subscription(
                Image, f"/camera/{cam}/image",
                lambda msg, c=cam: self._cam_cb(msg, c), 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)

        # ── Detection timer ───────────────────────────────────────────────
        self.create_timer(1.0 / self.detection_rate, self._detect_tick)

        self.get_logger().info(
            f"Human detector CV started (rate={self.detection_rate} Hz, "
            f"range={self.max_range} m)")

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _cam_cb(self, msg, cam_name):
        try:
            self.camera_images[cam_name] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Camera {cam_name}: {e}",
                                   throttle_duration_sec=5.0)

    def _odom_cb(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        self.robot_x = self.spawn_x + ox * self.spawn_cos - oy * self.spawn_sin
        self.robot_y = self.spawn_y + ox * self.spawn_sin + oy * self.spawn_cos
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy) + self.spawn_yaw

    # ── Core detection ────────────────────────────────────────────────────

    def _detect_tick(self):
        """Run detection on all cameras, update tracker, publish."""
        if not HAS_CV:
            return

        now = time.time()
        raw_detections = []   # list of (world_x, world_y)

        for cam_name, img in self.camera_images.items():
            if img is None:
                continue
            dets = self._detect_humans_in_image(img, cam_name)
            raw_detections.extend(dets)

        # ── Update tracker ────────────────────────────────────────────
        matched = set()
        for (wx, wy) in raw_detections:
            best_track = None
            best_dist = 2.0    # max association distance (metres)
            for trk in self.tracked_humans:
                d = trk.distance_to(wx, wy)
                if d < best_dist:
                    best_dist = d
                    best_track = trk
            if best_track is not None:
                best_track.update(wx, wy, now)
                matched.add(id(best_track))
            else:
                self.tracked_humans.append(TrackedHuman(wx, wy, now))

        # Prune stale tracks
        self.tracked_humans = [
            t for t in self.tracked_humans
            if (now - t.last_seen) < self.track_timeout
        ]

        # ── Publish current positions ─────────────────────────────────
        pose_arr = PoseArray()
        pose_arr.header.frame_id = "map"
        pose_arr.header.stamp = self.get_clock().now().to_msg()

        vel_arr = PoseArray()
        vel_arr.header = pose_arr.header

        for trk in self.tracked_humans:
            if trk.hits < 2:
                continue   # need at least 2 detections before publishing

            p = Pose()
            p.position.x = trk.x
            p.position.y = trk.y
            p.position.z = 0.0
            # Encode track ID in orientation.w (hacky but simple)
            p.orientation.w = float(trk.id)
            pose_arr.poses.append(p)

            # Velocity message: dx,dy encoded in position
            vp = Pose()
            vp.position.x = trk.vx
            vp.position.y = trk.vy
            vp.position.z = 0.0
            vp.orientation.w = float(trk.id)
            vel_arr.poses.append(vp)

        self.humans_pub.publish(pose_arr)
        self.vel_pub.publish(vel_arr)
        self._publish_markers(pose_arr)

        if pose_arr.poses:
            self.get_logger().info(
                f"Detected {len(pose_arr.poses)} human(s)",
                throttle_duration_sec=2.0)

        # Show OpenCV debug window
        if self.show_debug:
            self._show_debug_window()

    # ── Debug window ──────────────────────────────────────────────────────

    def _show_debug_window(self):
        """Display a 2×2 grid of the 4 camera views with detection overlays."""
        try:
            panels = []
            cam_order = ["front", "right", "back", "left"]
            target_h, target_w = 240, 320   # per-panel size

            for cam in cam_order:
                img = self.camera_images[cam]
                if img is None:
                    panel = np.zeros((target_h, target_w, 3), dtype=np.uint8)
                    cv2.putText(panel, f"{cam}: no image", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                else:
                    panel = cv2.resize(img, (target_w, target_h))
                    scale_x = target_w / img.shape[1]
                    scale_y = target_h / img.shape[0]

                    # Draw detection boxes
                    for det in self.debug_detections.get(cam, []):
                        bx, by, bw, bh = det["box"]
                        bx = int(bx * scale_x)
                        by = int(by * scale_y)
                        bw = int(bw * scale_x)
                        bh = int(bh * scale_y)
                        if det["type"] == "body":
                            cv2.rectangle(panel, (bx, by),
                                          (bx + bw, by + bh), (0, 255, 0), 2)
                            rng = det.get("range", 0)
                            cv2.putText(panel, f"{rng:.1f}m",
                                        (bx, by - 5),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                                        (0, 255, 0), 1)
                        elif det["type"] == "head":
                            cv2.rectangle(panel, (bx, by),
                                          (bx + bw, by + bh), (0, 0, 255), 2)
                            cv2.putText(panel, "HEAD",
                                        (bx, by - 5),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                        (0, 0, 255), 1)

                # Camera label
                cv2.putText(panel, cam.upper(), (5, target_h - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                panels.append(panel)

            # 2×2 grid
            top = np.hstack([panels[0], panels[1]])
            bot = np.hstack([panels[2], panels[3]])
            grid = np.vstack([top, bot])

            # Tracking summary
            n_tracked = sum(1 for t in self.tracked_humans if t.hits >= 2)
            cv2.putText(grid, f"Tracked: {n_tracked} humans",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 255), 2)

            cv2.imshow("Human Detection - 4 Cameras", grid)
            cv2.waitKey(1)
        except Exception as e:
            # Headless environment or display error — disable quietly
            self.show_debug = False
            self.get_logger().warn(
                f"Debug window disabled: {e}", throttle_duration_sec=10.0)

    def _detect_humans_in_image(self, img, cam_name):
        """
        Detect humans (white cylinder body + red sphere head) in a single
        camera image.  Returns list of (world_x, world_y) positions.
        """
        h, w = img.shape[:2]
        detections = []
        debug_boxes = []   # for the debug window

        # ── 1) HSV conversion ──────────────────────────────────────────
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # ── 2) WHITE mask (body) ───────────────────────────────────────
        white_mask = cv2.inRange(
            hsv,
            np.array([0, 0, self.white_v_min]),
            np.array([180, self.white_s_max, 255])
        )

        # ── 3) RED mask (head) — red wraps around H=0/180 ─────────────
        red_mask1 = cv2.inRange(
            hsv,
            np.array([self.red_h_low1, self.red_s_min, self.red_v_min]),
            np.array([self.red_h_high1, 255, 255])
        )
        red_mask2 = cv2.inRange(
            hsv,
            np.array([self.red_h_low2, self.red_s_min, self.red_v_min]),
            np.array([self.red_h_high2, 255, 255])
        )
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # ── 4) Morphological cleanup ──────────────────────────────────
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        kernel_sm = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel_sm, iterations=1)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel_sm, iterations=2)

        # ── 5) Find white contours (body candidates) ──────────────────
        white_contours, _ = cv2.findContours(
            white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # ── 6) Find red contours (head candidates) ────────────────────
        red_contours, _ = cv2.findContours(
            red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_boxes = []
        for rc in red_contours:
            area = cv2.contourArea(rc)
            if area < 30:   # very small blobs → noise (lowered for distant heads)
                continue
            rx, ry, rw, rh = cv2.boundingRect(rc)
            red_boxes.append((rx, ry, rw, rh))

        for cnt in white_contours:
            area = cv2.contourArea(cnt)
            if area < self.min_contour_area:
                continue

            x_r, y_r, bw, bh = cv2.boundingRect(cnt)

            # Filter: tall blobs (cylinder aspect ratio ≥ 1.0)
            if bh < bw * 1.0:
                continue

            # Filter: solidity
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            if hull_area > 0:
                solidity = area / hull_area
                if solidity < 0.5:
                    continue

            # ── 7) Require a red head blob ABOVE the white body ────────
            body_cx = x_r + bw / 2.0
            body_top = y_r          # top-edge y of the body bounding box
            head_found = False
            for (rx, ry, rw, rh) in red_boxes:
                red_cx = rx + rw / 2.0
                red_bot = ry + rh    # bottom-edge y of red blob
                # Red blob centre must be horizontally close to body centre
                if abs(red_cx - body_cx) > bw * 2.0:
                    continue
                # Red blob must be above or overlapping the top of the body
                # (allow up to body-height*0.3 overlap and head-height gap)
                if red_bot > body_top + bh * 0.5:
                    continue
                if ry < body_top - bh * 0.8:
                    continue   # too far above
                head_found = True
                # Draw red head box in debug
                debug_boxes.append({"type": "head", "box": (rx, ry, rw, rh)})
                break

            if not head_found:
                continue   # no red head → not a confirmed human

            # ── 8) Estimate range from apparent height (pinhole model) ─
            if bh < 10:
                continue
            estimated_range = (self.human_h * self.focal_px) / bh
            if estimated_range > self.max_range or estimated_range < 0.3:
                continue

            # ── 9) Estimate bearing from centroid column ───────────────
            norm_x = (body_cx / w) - 0.5
            bearing_in_cam = norm_x * self.cam_hfov

            # ── 10) Convert to world frame ─────────────────────────────
            cam_bearing = self.CAMERA_BEARINGS[cam_name]
            world_bearing = self.robot_yaw + cam_bearing + bearing_in_cam
            world_x = self.robot_x + estimated_range * math.cos(world_bearing)
            world_y = self.robot_y + estimated_range * math.sin(world_bearing)

            detections.append((world_x, world_y))
            debug_boxes.append({
                "type": "body", "box": (x_r, y_r, bw, bh),
                "range": estimated_range
            })

        # Store debug info for this camera
        self.debug_detections[cam_name] = debug_boxes
        return detections

    # ── Visualization ─────────────────────────────────────────────────────

    def _publish_markers(self, pose_arr):
        """Publish RViz markers showing detected humans."""
        markers = MarkerArray()

        # Delete old markers
        delete = Marker()
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        stamp = self.get_clock().now().to_msg()
        mid = 0

        for i, trk in enumerate(self.tracked_humans):
            if trk.hits < 2:
                continue

            # Body cylinder marker (white)
            body = Marker()
            body.header.frame_id = "map"
            body.header.stamp = stamp
            body.ns = "human_body"
            body.id = mid; mid += 1
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x = trk.x
            body.pose.position.y = trk.y
            body.pose.position.z = 0.85
            body.pose.orientation.w = 1.0
            body.scale.x = 0.5   # diameter
            body.scale.y = 0.5
            body.scale.z = 1.7   # height
            body.color.r = 1.0
            body.color.g = 1.0
            body.color.b = 1.0
            body.color.a = 0.7
            body.lifetime.sec = 1
            markers.markers.append(body)

            # Head sphere marker (red)
            head = Marker()
            head.header.frame_id = "map"
            head.header.stamp = stamp
            head.ns = "human_head"
            head.id = mid; mid += 1
            head.type = Marker.SPHERE
            head.action = Marker.ADD
            head.pose.position.x = trk.x
            head.pose.position.y = trk.y
            head.pose.position.z = 1.85
            head.pose.orientation.w = 1.0
            head.scale.x = 0.3
            head.scale.y = 0.3
            head.scale.z = 0.3
            head.color.r = 1.0
            head.color.g = 0.0
            head.color.b = 0.0
            head.color.a = 0.9
            head.lifetime.sec = 1
            markers.markers.append(head)

            # Velocity arrow
            spd = math.hypot(trk.vx, trk.vy)
            if spd > 0.05:
                arrow = Marker()
                arrow.header.frame_id = "map"
                arrow.header.stamp = stamp
                arrow.ns = "human_velocity"
                arrow.id = mid; mid += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD

                start = Point()
                start.x = trk.x; start.y = trk.y; start.z = 0.5
                # Show predicted position 2 seconds ahead
                end = Point()
                pred = trk.predict(2.0)
                end.x = pred[0]; end.y = pred[1]; end.z = 0.5
                arrow.points = [start, end]
                arrow.scale.x = 0.08   # shaft diameter
                arrow.scale.y = 0.15   # head diameter
                arrow.scale.z = 0.0
                arrow.color.r = 1.0
                arrow.color.g = 0.5
                arrow.color.b = 0.0
                arrow.color.a = 0.8
                arrow.lifetime.sec = 1
                markers.markers.append(arrow)

            # ID label
            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = stamp
            label.ns = "human_id"
            label.id = mid; mid += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = trk.x
            label.pose.position.y = trk.y
            label.pose.position.z = 2.3
            label.scale.z = 0.3
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 0.0
            label.color.a = 1.0
            label.text = f"H{trk.id} v={spd:.1f}m/s"
            label.lifetime.sec = 1
            markers.markers.append(label)

        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = HumanDetectorCV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

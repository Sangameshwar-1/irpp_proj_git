#!/usr/bin/env python3
"""
Human Case Controller
=====================
Ground-truth human injector for social navigation planning tests.

Replaces both human_detector_red AND move_humans for the planning test
cases.  This node:
  1. Animates one human model in Gazebo via the set_pose service.
  2. Publishes PERFECT /detected_humans and /human_velocities so the
     social_nav_planner has accurate, noise-free ground-truth data.

One node instance per test session.  The `case` ROS parameter selects
the scenario:

  case1  — static human at ( 0.0,  0.0)  vel ( 0.00,  0.00)  [blocking]
  case2  — human at ( 3.5,  0.0)         vel (-0.20,  0.00)  [head-on]
  case3  — human at ( 0.0,  2.0)         vel ( 0.00, -0.15)  [crossing]
  case4a — human at (-1.0,  0.0)         vel ( 0.20,  0.00)  [ahead, same dir]
  case4b — human at (-5.0,  0.0)         vel ( 0.40,  0.00)  [behind, faster]
  case5  — human at (-7.0,  0.0)         vel ( 0.60,  0.00)  [behind, much faster]
  case6a — human at (-1.0,  3.5)         vel ( 0.20, -0.30)  [diagonal upper-left]
  case6b — human at (-1.0, -3.5)         vel ( 0.20,  0.30)  [diagonal lower-left]
  case6c — human at ( 3.0,  2.5)         vel (-0.30, -0.15)  [diagonal upper-right]
  case6d — human at ( 3.0, -2.5)         vel (-0.30,  0.15)  [diagonal lower-right]

Topics
------
  Publishes:
    /detected_humans    (PoseArray)  — ground truth position (id in orientation.w)
    /human_velocities   (PoseArray)  — ground truth velocity (vx/vy in position.x/y)
"""

import math
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity


def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


# ── Case catalogue ─────────────────────────────────────────────────────────────
CASES = {
    # pos (x, y) — starting position injected into Gazebo on first tick
    # vel (vx, vy) — constant velocity applied each tick
    # yaw — visual orientation of the human model
    "case1": {
        "desc": "Static human blocking direct path",
        "pos": ( 0.0,  0.0),
        "vel": ( 0.0,  0.0),
        "yaw":  0.0,
    },
    "case2": {
        "desc": "Human approaching head-on",
        "pos": ( 3.5,  0.0),
        "vel": (-0.2,  0.0),
        "yaw":  math.pi,
    },
    "case3": {
        "desc": "Human crossing path — collision on current course (collision cone)",
        "pos": ( 0.0,  4.0),
        "vel": ( 0.0, -0.30),
        "yaw": -math.pi / 2.0,
    },
    "case4a": {
        "desc": "Human ahead, same direction — no overtaking",
        "pos": (-1.0,  0.0),
        "vel": ( 0.2,  0.0),
        "yaw":  0.0,
    },
    "case4b": {
        "desc": "Human behind, faster — give way",
        "pos": (-5.0,  0.0),
        "vel": ( 0.4,  0.0),
        "yaw":  0.0,
    },
    "case5": {
        "desc": "Human from behind, much faster — replan to move aside",
        "pos": (-7.0,  0.0),
        "vel": ( 0.60,  0.0),
        "yaw":  0.0,
    },
    "case6a": {
        "desc": "Diagonal crossing from upper-left (moving down-right)",
        "pos": (-1.0,  3.5),
        "vel": ( 0.20, -0.30),
        "yaw":  math.atan2(-0.30, 0.20),
    },
    "case6b": {
        "desc": "Diagonal crossing from lower-left (moving up-right)",
        "pos": (-1.0, -3.5),
        "vel": ( 0.20,  0.30),
        "yaw":  math.atan2(0.30, 0.20),
    },
    "case6c": {
        "desc": "Diagonal approach from upper-right (moving down-left toward robot)",
        "pos": ( 3.0,  2.5),
        "vel": (-0.30, -0.15),
        "yaw":  math.atan2(-0.15, -0.30),
    },
    "case6d": {
        "desc": "Diagonal approach from lower-right (moving up-left toward robot)",
        "pos": ( 3.0, -2.5),
        "vel": (-0.30,  0.15),
        "yaw":  math.atan2(0.15, -0.30),
    },
}


class HumanCaseController(Node):
    """Animates one human in Gazebo and publishes ground-truth poses."""

    def __init__(self):
        super().__init__("human_case_controller")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("case",       "case1")
        self.declare_parameter("world_name", "test_case_arena")
        self.declare_parameter("model_name", "human_1")
        self.declare_parameter("rate",       20.0)

        case_name   = self.get_parameter("case").value
        world_name  = self.get_parameter("world_name").value
        self.model  = self.get_parameter("model_name").value
        rate        = self.get_parameter("rate").value
        self.world_name = world_name        # keep for gz CLI calls

        cfg = CASES.get(case_name, CASES["case1"])
        self.hx,  self.hy  = cfg["pos"]
        self.vx,  self.vy  = cfg["vel"]
        self.yaw            = cfg["yaw"]
        self.description    = cfg["desc"]

        self.get_logger().info(
            f"Human controller: [{case_name}] {self.description}")
        self.get_logger().info(
            f"  Start ({self.hx:.1f}, {self.hy:.1f})  "
            f"Vel ({self.vx:.3f}, {self.vy:.3f}) m/s")

        # ── Publishers ───────────────────────────────────────────────
        self.humans_pub = self.create_publisher(
            PoseArray, "/detected_humans",  10)
        self.vel_pub    = self.create_publisher(
            PoseArray, "/human_velocities", 10)

        # ── Gazebo set_pose service ──────────────────────────────────
        svc = f"/world/{world_name}/set_pose"
        self.client = self.create_client(SetEntityPose, svc)
        self.get_logger().info(f"Waiting for {svc} …")
        self.client.wait_for_service(timeout_sec=15.0)
        self.get_logger().info("set_pose service ready — human controller active")

        # ── Initial teleport via gz CLI (reliable, bypasses bridge) ──
        self._gz_teleport(self.hx, self.hy, self.yaw)

        # ── Timing state ─────────────────────────────────────────────
        self.last_sec: float | None = None
        self._teleport_failures = 0

        self.create_timer(1.0 / max(rate, 1.0), self._tick)

    # ── gz CLI teleport (reliable) ───────────────────────────────

    def _gz_teleport(self, x: float, y: float, yaw: float):
        """Teleport human via `gz service` CLI — bypasses ros_gz_bridge."""
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        req_msg = (
            f'name: "{self.model}" '
            f'position: {{ x: {x}, y: {y}, z: 0.0 }} '
            f'orientation: {{ x: {qx}, y: {qy}, z: {qz}, w: {qw} }}'
        )
        cmd = [
            'gz', 'service',
            '-s', f'/world/{self.world_name}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', req_msg,
        ]
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                self.get_logger().info(
                    f"✅ Teleported {self.model} → ({x:.1f}, {y:.1f}) via gz CLI")
            else:
                self.get_logger().warn(
                    f"⚠ gz teleport rc={result.returncode}: "
                    f"{result.stderr.strip()}")
        except Exception as e:
            self.get_logger().warn(f"⚠ gz teleport error: {e}")

    # ── Tick ─────────────────────────────────────────────────────────

    def _tick(self):
        now_stamp = self.get_clock().now()
        ns0, ns1  = now_stamp.seconds_nanoseconds()
        now_sec   = float(ns0) + float(ns1) * 1e-9

        # Integrate velocity → update position
        if self.last_sec is not None:
            dt = now_sec - self.last_sec
            self.hx += self.vx * dt
            self.hy += self.vy * dt
        self.last_sec = now_sec

        # Teleport human model in Gazebo
        self._set_pose_gz(self.hx, self.hy, self.yaw)

        # Publish ground truth to planning node
        stamp = now_stamp.to_msg()
        self._publish_gt(stamp)

    def _set_pose_gz(self, x: float, y: float, yaw: float):
        """Send set_pose request to Gazebo (async with result check).
        Falls back to gz CLI every 100 ticks if bridge keeps failing."""
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = 0.0      # model root at floor (z=0); body link has +0.5 offset internally
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        entity = Entity()
        entity.name = self.model
        entity.type = Entity.MODEL

        req = SetEntityPose.Request()
        req.entity = entity
        req.pose   = pose
        future = self.client.call_async(req)
        future.add_done_callback(self._set_pose_done)

        # Periodic gz CLI fallback (every 5 s at 20 Hz = every 100 ticks)
        if self._teleport_failures > 0 and self._teleport_failures % 100 == 0:
            self._gz_teleport(x, y, yaw)

    def _set_pose_done(self, future):
        """Callback to check if set_pose succeeded."""
        try:
            result = future.result()
            if result is None or not result.success:
                self._teleport_failures += 1
                if self._teleport_failures <= 3:
                    self.get_logger().warn(
                        f"set_pose bridge returned failure "
                        f"(count={self._teleport_failures})")
        except Exception:
            self._teleport_failures += 1

    def _publish_gt(self, stamp):
        """Publish human position and velocity to planning node."""
        # Position message (same format as human_detector_red output)
        pos_msg = PoseArray()
        pos_msg.header.frame_id = "map"
        pos_msg.header.stamp    = stamp
        p = Pose()
        p.position.x = self.hx
        p.position.y = self.hy
        p.position.z = 0.0
        p.orientation.w = 1.0   # tracker id = 0
        pos_msg.poses.append(p)
        self.humans_pub.publish(pos_msg)

        # Velocity message (vx/vy stored in position.x/y — same as local_planner expects)
        vel_msg = PoseArray()
        vel_msg.header = pos_msg.header
        v = Pose()
        v.position.x = self.vx
        v.position.y = self.vy
        v.position.z = 0.0
        v.orientation.w = 1.0
        vel_msg.poses.append(v)
        self.vel_pub.publish(vel_msg)


def main():
    rclpy.init()
    node = HumanCaseController()
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

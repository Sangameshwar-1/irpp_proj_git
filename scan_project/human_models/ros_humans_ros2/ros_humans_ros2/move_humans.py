import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity


def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


# Define valid movable region bounds (room is 25x25, walls at ±12.5)
ROOM_MIN = -10.0
ROOM_MAX = 10.0


class HumanMover(Node):
    def __init__(self):
        super().__init__("move_humans")
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("world", "large_messy_room")

        # ── Social-nav case humans (all on robot path: spawn 0,-8 → goal 5,5) ──
        # Case 1 — near-static blocker: sits on the path, speed < STATIC_THR
        # Case 2 — head-on:             approaches robot from north along path
        # Case 3 — crossing:            east-west across robot's north-bound path
        # Case 4a — ahead same dir:     slower human just ahead of robot
        # Case 5  — fast from behind:   faster human overtaking from behind
        default_humans = [
            # Case 1: near-stationary blocker at (1,-2) on the robot path
            {
                "name": "human_moving_1",
                "speed": 0.03,      # < STATIC_THR=0.05 → classified case1
                "waypoints": [
                    [1.0, -2.0, 0.0],
                    [1.2, -2.0, math.pi],
                    [1.2, -2.0, math.pi],
                    [1.0, -2.0, 0.0],
                ],
            },
            # Case 2: head-on — walks south along the robot's expected path
            # Start at y=6 (further north) so the human is still heading SOUTH
            # when the robot arrives at mid-room (~y=-2, t≈22s).  With y=4 the
            # human would reach its south waypoint at y=-5 in only 11/0.35=31s
            # giving plenty of margin; starting at y=6 adds ~6s more runway.
            {
                "name": "human_moving_2",
                "speed": 0.35,
                "waypoints": [
                    [1.5,  6.0, -math.pi / 2.0],   # north end, heading south
                    [1.5, -5.0,  math.pi / 2.0],   # south end, heading north
                    [1.5, -5.0,  math.pi / 2.0],
                    [1.5,  6.0, -math.pi / 2.0],
                ],
            },
            # Case 3: crossing — east-west across robot's northbound path at y≈1
            {
                "name": "human_moving_3",
                "speed": 0.45,
                "waypoints": [
                    [-7.0, 1.0,  0.0],          # west, heading east
                    [ 7.0, 1.0,  math.pi],      # east, heading west
                    [ 7.0, 1.0,  math.pi],
                    [-7.0, 1.0,  0.0],
                ],
            },
            # Case 4a: same direction ahead — slower, robot will catch up
            {
                "name": "human_moving_4",
                "speed": 0.22,      # slower than rover (0.30) → case4a when ahead
                "waypoints": [
                    [ 0.5, -5.0,  math.pi / 4.0],
                    [ 4.5,  4.0, -3.0 * math.pi / 4.0],
                    [ 4.5,  4.0, -3.0 * math.pi / 4.0],
                    [ 0.5, -5.0,  math.pi / 4.0],
                ],
            },
            # Case 5: fast from behind — same NE direction, faster than rover
            # Start at y=-12 (4m behind robot spawn y=-8).  Relative overtake
            # speed ≈ 0.28 m/s → ~14s to close, giving a clear observation
            # window before the human passes. (y=-9 was only 1m behind = 4s)
            {
                "name": "human_moving_5",
                "speed": 0.58,      # faster than rover (0.30) → case5 when behind
                "waypoints": [
                    [ 0.5, -12.0,  math.pi / 2.0],
                    [ 0.5,   9.0, -math.pi / 2.0],
                    [ 0.5,   9.0, -math.pi / 2.0],
                    [ 0.5, -12.0,  math.pi / 2.0],
                ],
            },
        ]
        self.declare_parameter("humans_json", json.dumps(default_humans))
        # publish_detections=True  → GT mode: publishes /detected_humans + /human_velocities
        # publish_detections=False → camera mode: only teleports humans; perception via camera
        self.declare_parameter("publish_detections", True)
        self.rate_hz = self.get_parameter("rate").get_parameter_value().double_value
        self.world = self.get_parameter("world").get_parameter_value().string_value
        humans_json = self.get_parameter("humans_json").get_parameter_value().string_value
        self.humans = json.loads(humans_json)
        self._publish_detections = (
            self.get_parameter("publish_detections").get_parameter_value().bool_value
        )

        service_name = f"/world/{self.world}/set_pose"
        self.client = self.create_client(SetEntityPose, service_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {service_name}...")

        # Ground truth publisher (for live_visualization_node — always active)
        self.gt_pub = self.create_publisher(PoseArray, '/human_ground_truth', 10)
        # Social nav planner inputs (only published in GT mode)
        self.det_pub = self.create_publisher(PoseArray, '/detected_humans', 10)
        self.vel_pub = self.create_publisher(PoseArray, '/human_velocities', 10)

        mode_str = "GT injection" if self._publish_detections else "camera mode (teleport only)"
        self.get_logger().info(f"move_humans perception mode: {mode_str}")

        self.state = {}
        self._init_segments()
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self._tick)

    def _init_segments(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        for human in self.humans:
            wps = human["waypoints"]
            self.state[human["name"]] = {
                "idx": 0,
                "start": float(now),
                "duration": self._segment_duration(wps[0], wps[1], human["speed"]),
            }

    @staticmethod
    def _segment_duration(a, b, speed):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dist = math.hypot(dx, dy)
        return max(dist / max(speed, 0.01), 0.1)

    def _update_human(self, human, now):
        name = human["name"]
        wps = human["waypoints"]
        st = self.state[name]
        idx = st["idx"]
        nxt = (idx + 1) % len(wps)
        t = (now - st["start"]) / st["duration"]

        if t >= 1.0:
            st["idx"] = nxt
            st["start"] = now
            st["duration"] = self._segment_duration(wps[nxt], wps[(nxt + 1) % len(wps)], human["speed"])
            idx = st["idx"]
            nxt = (idx + 1) % len(wps)
            t = 0.0

        ax, ay, ayaw = wps[idx]
        bx, by, byaw = wps[nxt]
        x = ax + (bx - ax) * t
        y = ay + (by - ay) * t
        yaw = ayaw + (byaw - ayaw) * t

        # Instantaneous velocity from current segment
        seg_dx = bx - ax
        seg_dy = by - ay
        seg_dist = math.hypot(seg_dx, seg_dy)
        spd = human["speed"]
        if seg_dist > 0.01:
            vx = (seg_dx / seg_dist) * spd
            vy = (seg_dy / seg_dist) * spd
        else:
            vx, vy = 0.0, 0.0

        qx, qy, qz, qw = _quat_from_yaw(yaw)
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = 0.0
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        entity = Entity()
        entity.name = name
        entity.type = Entity.MODEL

        req = SetEntityPose.Request()
        req.entity = entity
        req.pose = pose
        self.client.call_async(req)
        return (x, y, vx, vy)

    def _tick(self):
        now = float(self.get_clock().now().seconds_nanoseconds()[0])
        stamp = self.get_clock().now().to_msg()

        gt_msg  = PoseArray()
        det_msg = PoseArray()
        vel_msg = PoseArray()
        gt_msg.header.frame_id  = 'map'
        det_msg.header.frame_id = 'map'
        vel_msg.header.frame_id = 'map'
        gt_msg.header.stamp  = stamp
        det_msg.header.stamp = stamp
        vel_msg.header.stamp = stamp

        for human in self.humans:
            result = self._update_human(human, now)
            if result is None:
                continue
            x, y, vx, vy = result

            # /human_ground_truth (for live_visualization_node)
            pg = Pose()
            pg.position.x = x
            pg.position.y = y
            gt_msg.poses.append(pg)

            # /detected_humans — position for social_nav_planner
            pd = Pose()
            pd.position.x = x
            pd.position.y = y
            det_msg.poses.append(pd)

            # /human_velocities — velocity for social_nav_planner
            pv = Pose()
            pv.position.x = vx
            pv.position.y = vy
            vel_msg.poses.append(pv)

        self.gt_pub.publish(gt_msg)
        if self._publish_detections:
            self.det_pub.publish(det_msg)
            self.vel_pub.publish(vel_msg)


def main():
    rclpy.init()
    node = HumanMover()
    node.get_logger().info("Human mover started (ROS2)")
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

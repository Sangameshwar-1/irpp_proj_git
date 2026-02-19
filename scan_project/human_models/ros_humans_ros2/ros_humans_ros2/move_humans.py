import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
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
        
        # All waypoints MUST be within valid movable region [-10, 10]
        default_humans = [
            # 1. Horizontal patrol across room center (avoids obstacles)
            {
                "name": "human_moving_1",
                "speed": 0.4,
                "waypoints": [
                    [-7.0, 2.0, 0.0],           # West side
                    [7.0, 2.0, math.pi],        # East side
                    [7.0, 2.0, math.pi],        # Turn
                    [-7.0, 2.0, 0.0],           # Back west
                ],
            },
            # 2. Vertical patrol on east side
            {
                "name": "human_moving_2",
                "speed": 0.35,
                "waypoints": [
                    [5.0, -7.0, math.pi / 2.0],   # South
                    [5.0, 7.0, -math.pi / 2.0],   # North
                    [5.0, 7.0, -math.pi / 2.0],   # Turn
                    [5.0, -7.0, math.pi / 2.0],   # Back south
                ],
            },
            # 3. Random walker around center (avoiding obstacles)
            {
                "name": "human_moving_3",
                "speed": 0.3,
                "waypoints": [
                    [0.0, 4.0, -math.pi / 4.0],
                    [4.0, 1.0, -math.pi / 2.0],
                    [2.0, -3.0, math.pi],
                    [-3.0, -1.0, math.pi / 2.0],
                    [-3.0, 3.0, 0.0],
                    [0.0, 4.0, -math.pi / 4.0],
                ],
            },
            # 4. Diagonal walker (staying in valid region)
            {
                "name": "human_moving_4",
                "speed": 0.35,
                "waypoints": [
                    [-5.0, -5.0, math.pi / 4.0],
                    [5.0, 5.0, -3 * math.pi / 4.0],
                    [5.0, 5.0, -3 * math.pi / 4.0],
                    [-5.0, -5.0, math.pi / 4.0],
                ],
            },
            # 5. Perimeter walker (inside walls)
            {
                "name": "human_moving_5",
                "speed": 0.4,
                "waypoints": [
                    [7.0, 0.0, math.pi / 2.0],
                    [4.0, 7.0, math.pi],
                    [-4.0, 6.0, -math.pi],
                    [-7.0, 0.0, -math.pi / 2.0],
                    [-4.0, -6.0, 0.0],
                    [4.0, -7.0, math.pi / 2.0],
                    [7.0, 0.0, math.pi / 2.0],
                ],
            },
        ]
        self.declare_parameter("humans_json", json.dumps(default_humans))
        self.rate_hz = self.get_parameter("rate").get_parameter_value().double_value
        self.world = self.get_parameter("world").get_parameter_value().string_value
        humans_json = self.get_parameter("humans_json").get_parameter_value().string_value
        self.humans = json.loads(humans_json)

        service_name = f"/world/{self.world}/set_pose"
        self.client = self.create_client(SetEntityPose, service_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {service_name}...")

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

    def _tick(self):
        now = float(self.get_clock().now().seconds_nanoseconds()[0])
        for human in self.humans:
            self._update_human(human, now)


def main():
    rclpy.init()
    node = HumanMover()
    node.get_logger().info("Human mover started (ROS2)")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

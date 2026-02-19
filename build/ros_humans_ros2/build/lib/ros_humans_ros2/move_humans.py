import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity


def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class HumanMover(Node):
    def __init__(self):
        super().__init__("move_humans")
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("world", "complex_grid_map")
        default_humans = [
            # 1. Wide/Artery Navigation: Long paths, higher speed
            {
                "name": "human_moving_1",
                "speed": 0.6,
                "waypoints": [
                    [-45.0, 20.0, math.pi / 2.0],  # West Artery North
                    [-45.0, -20.0, -math.pi / 2.0], # West Artery South
                    [-20.0, -20.0, 0.0], # Crossing to Mid-West
                    [-20.0, -15.0, math.pi / 2.0], # To Lower Cross Road
                    [-45.0, -15.0, math.pi], # Back to Artery
                ],
            },
            # 2. Central Navigation: Moderate speed, grid pattern
            {
                "name": "human_moving_2",
                "speed": 0.5,
                "waypoints": [
                    [10.0, 15.0, math.pi / 2.0], # Upper Cross Road / Center Road overlap
                    [10.0, -15.0, -math.pi / 2.0], # Lower Cross Road / Center Road
                    [-15.0, -15.0, math.pi], # Back to Mid-West
                    [-15.0, 15.0, math.pi / 2.0], # To Upper Cross Road
                    [10.0, 15.0, 0.0], # Back to start
                ],
            },
            # 3. Tight Space Navigation: Dense Grid (Warren), slower/changing
            {
                "name": "human_moving_3",
                "speed": 0.35, # Slower for tight spaces
                "waypoints": [
                    [36.0, 0.0, 0.0], # Entering Warren area (between blocks)
                    [44.0, 0.0, 0.0],
                    [44.0, 3.0, math.pi / 2.0], # Between N blocks
                    [36.0, 3.0, math.pi],
                    [36.0, -3.0, -math.pi / 2.0], # Between S blocks
                    [44.0, -3.0, 0.0],
                    [44.0, 0.0, math.pi / 2.0], # Loop
                ],
            },
            # 4. Perimeter/Alley Navigation: Narrow paths (East Alley)
            {
                "name": "human_moving_4",
                "speed": 0.4,
                "waypoints": [
                    [48.0, 25.0, -math.pi / 2.0], # East Alley North
                    [48.0, -25.0, -math.pi / 2.0], # East Alley South
                    [39.0, -20.0, math.pi], # Cut through Block H4/Warren gap
                    [39.0, 20.0, math.pi / 2.0], # Back up past Warren
                    [48.0, 25.0, 0.0],
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

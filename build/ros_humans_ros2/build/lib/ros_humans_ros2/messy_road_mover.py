import json
import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity


def _quat_from_yaw(yaw):
    """Convert yaw angle to quaternion."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MessyRoadMover(Node):
    """
    Node to move humans and vehicles in a messy road simulation.
    Supports different speeds, heights, and movement patterns.
    """

    def __init__(self):
        super().__init__("messy_road_mover")
        
        # Parameters
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("world", "messy_road")
        self.declare_parameter("enable_random_behavior", True)
        
        # Default humans with different heights and speeds
        default_humans = [
            # 1. Tall Fast Walker (1.9m) - Walking on sidewalk
            {
                "name": "human_tall_fast",
                "type": "human",
                "speed": 1.8,  # Fast walking speed (m/s)
                "height": 1.9,
                "behavior": "linear",
                "waypoints": [
                    [-40.0, 5.5, 0.0],
                    [40.0, 5.5, 0.0],
                    [40.0, 5.5, 3.14159],
                    [-40.0, 5.5, 3.14159],
                ],
            },
            # 2. Average Medium Walker (1.7m) - Crossing street
            {
                "name": "human_avg_medium",
                "type": "human",
                "speed": 1.2,
                "height": 1.7,
                "behavior": "linear",
                "waypoints": [
                    [-30.0, -5.5, 0.0],
                    [-30.0, 5.5, 1.5708],
                    [-10.0, 5.5, 0.0],
                    [-10.0, -5.5, -1.5708],
                    [-30.0, -5.5, 3.14159],
                ],
            },
            # 3. Short Slow Walker (1.5m) - On sidewalk
            {
                "name": "human_short_slow",
                "type": "human",
                "speed": 0.6,
                "height": 1.5,
                "behavior": "linear",
                "waypoints": [
                    [10.0, 5.5, 0.0],
                    [35.0, 5.5, 0.0],
                    [35.0, 5.5, 3.14159],
                    [10.0, 5.5, 3.14159],
                ],
            },
            # 4. Child Runner (1.2m) - Erratic movement
            {
                "name": "human_child_runner",
                "type": "human",
                "speed": 2.5,  # Running child
                "height": 1.2,
                "behavior": "erratic",
                "erratic_factor": 0.5,
                "waypoints": [
                    [20.0, -5.5, 0.0],
                    [25.0, -4.5, 0.5],
                    [22.0, -5.0, -0.5],
                    [28.0, -5.5, 0.3],
                    [20.0, -6.0, 3.14159],
                    [18.0, -5.0, -2.8],
                ],
            },
            # 5. Elderly Slow Walker (1.6m) - Very slow
            {
                "name": "human_elderly_slow",
                "type": "human",
                "speed": 0.4,
                "height": 1.6,
                "behavior": "linear",
                "waypoints": [
                    [-15.0, 5.5, 0.0],
                    [-5.0, 5.5, 0.0],
                    [-5.0, 5.5, 3.14159],
                    [-15.0, 5.5, 3.14159],
                ],
            },
            # 6. Jogger (1.75m) - Fast jogging
            {
                "name": "human_jogger",
                "type": "human",
                "speed": 3.0,  # Jogging speed
                "height": 1.75,
                "behavior": "linear",
                "waypoints": [
                    [35.0, 5.5, 3.14159],
                    [-35.0, 5.5, 3.14159],
                    [-35.0, 5.5, 0.0],
                    [35.0, 5.5, 0.0],
                ],
            },
            # 7. Distracted Phone User (1.68m) - Slow, wandering
            {
                "name": "human_phone_user",
                "type": "human",
                "speed": 0.5,
                "height": 1.68,
                "behavior": "wandering",
                "wander_radius": 0.3,
                "waypoints": [
                    [-5.0, -5.5, 0.0],
                    [5.0, -5.5, 0.0],
                    [5.0, -5.5, 3.14159],
                    [-5.0, -5.5, 3.14159],
                ],
            },
            # 8. Group Walker 1 (1.72m) - Walking in pair
            {
                "name": "human_group_1",
                "type": "human",
                "speed": 0.9,
                "height": 1.72,
                "behavior": "linear",
                "waypoints": [
                    [45.0, -5.5, 3.14159],
                    [-45.0, -5.5, 3.14159],
                    [-45.0, -5.5, 0.0],
                    [45.0, -5.5, 0.0],
                ],
            },
            # 9. Group Walker 2 (1.65m) - Walking with Group 1
            {
                "name": "human_group_2",
                "type": "human",
                "speed": 0.9,
                "height": 1.65,
                "behavior": "linear",
                "waypoints": [
                    [46.0, -5.5, 3.14159],
                    [-44.0, -5.5, 3.14159],
                    [-44.0, -5.5, 0.0],
                    [46.0, -5.5, 0.0],
                ],
            },
        ]

        # Default vehicles with different speeds
        default_vehicles = [
            # 1. Fast Car (Blue) - Highway speed
            {
                "name": "vehicle_car_fast",
                "type": "vehicle",
                "speed": 15.0,  # ~54 km/h
                "waypoints": [
                    [-50.0, -2.0, 0.0],
                    [55.0, -2.0, 0.0],
                    [55.0, -2.0, 3.14159],
                    [-50.0, -2.0, 3.14159],
                ],
            },
            # 2. Medium Speed Car (Red) - City speed
            {
                "name": "vehicle_car_medium",
                "type": "vehicle",
                "speed": 8.0,  # ~29 km/h
                "waypoints": [
                    [50.0, 2.0, 3.14159],
                    [-50.0, 2.0, 3.14159],
                    [-50.0, 2.0, 0.0],
                    [50.0, 2.0, 0.0],
                ],
            },
            # 3. Slow SUV (White) - Careful driver
            {
                "name": "vehicle_suv_slow",
                "type": "vehicle",
                "speed": 5.0,  # ~18 km/h
                "waypoints": [
                    [-20.0, -2.5, 0.0],
                    [45.0, -2.5, 0.0],
                    [45.0, -2.5, 3.14159],
                    [-20.0, -2.5, 3.14159],
                ],
            },
            # 4. Very Slow Truck - Heavy vehicle
            {
                "name": "vehicle_truck_slow",
                "type": "vehicle",
                "speed": 3.0,  # ~11 km/h
                "waypoints": [
                    [30.0, 2.5, 3.14159],
                    [-55.0, 2.5, 3.14159],
                    [-55.0, 2.5, 0.0],
                    [30.0, 2.5, 0.0],
                ],
            },
            # 5. Fast Motorcycle - Lane splitting
            {
                "name": "vehicle_motorcycle",
                "type": "vehicle",
                "speed": 12.0,  # ~43 km/h
                "waypoints": [
                    [-45.0, 1.5, 0.0],
                    [50.0, 0.5, 0.1],
                    [50.0, -1.5, 3.14159],
                    [-45.0, -0.5, 3.0],
                ],
            },
            # 6. Very Slow Bus - Public transport
            {
                "name": "vehicle_bus",
                "type": "vehicle",
                "speed": 4.0,  # ~14 km/h with stops
                "waypoints": [
                    [-55.0, 2.5, 0.0],
                    [-20.0, 2.5, 0.0],  # Bus stop 1
                    [10.0, 2.5, 0.0],   # Bus stop 2
                    [45.0, 2.5, 0.0],
                    [45.0, 2.5, 3.14159],
                    [10.0, 2.5, 3.14159],
                    [-20.0, 2.5, 3.14159],
                    [-55.0, 2.5, 3.14159],
                ],
            },
            # 7. Bicycle - On secondary road
            {
                "name": "vehicle_bicycle",
                "type": "vehicle",
                "speed": 4.5,  # ~16 km/h
                "waypoints": [
                    [0.0, 10.0, -1.5708],
                    [0.0, -35.0, -1.5708],
                    [0.0, -35.0, 1.5708],
                    [0.0, 10.0, 1.5708],
                ],
            },
        ]

        self.declare_parameter("humans_json", json.dumps(default_humans))
        self.declare_parameter("vehicles_json", json.dumps(default_vehicles))

        # Get parameters
        self.rate_hz = self.get_parameter("rate").get_parameter_value().double_value
        self.world = self.get_parameter("world").get_parameter_value().string_value
        self.enable_random = self.get_parameter("enable_random_behavior").get_parameter_value().bool_value
        
        humans_json = self.get_parameter("humans_json").get_parameter_value().string_value
        vehicles_json = self.get_parameter("vehicles_json").get_parameter_value().string_value
        
        self.humans = json.loads(humans_json)
        self.vehicles = json.loads(vehicles_json)
        
        # Combine all entities
        self.entities = self.humans + self.vehicles

        # Setup service client
        service_name = f"/world/{self.world}/set_pose"
        self.client = self.create_client(SetEntityPose, service_name)
        
        self.get_logger().info(f"Waiting for service: {service_name}")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service {service_name} not available, waiting...")

        # Initialize state for all entities
        self.state = {}
        self._init_segments()
        
        # Create timer
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self._tick)
        
        self.get_logger().info(f"Messy Road Mover started with {len(self.humans)} humans and {len(self.vehicles)} vehicles")

    def _init_segments(self):
        """Initialize movement state for all entities."""
        now = self.get_clock().now().seconds_nanoseconds()[0]
        
        for entity in self.entities:
            wps = entity["waypoints"]
            self.state[entity["name"]] = {
                "idx": 0,
                "start": float(now),
                "duration": self._segment_duration(wps[0], wps[1], entity["speed"]),
                "random_offset_x": 0.0,
                "random_offset_y": 0.0,
                "last_random_update": now,
            }

    @staticmethod
    def _segment_duration(a, b, speed):
        """Calculate duration to travel between two waypoints."""
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dist = math.hypot(dx, dy)
        return max(dist / max(speed, 0.01), 0.1)

    def _get_random_offset(self, entity, state, now):
        """Generate random offset for wandering/erratic behavior."""
        behavior = entity.get("behavior", "linear")
        
        if behavior == "erratic":
            # Update random offset periodically
            if now - state["last_random_update"] > 0.5:
                factor = entity.get("erratic_factor", 0.3)
                state["random_offset_x"] = random.uniform(-factor, factor)
                state["random_offset_y"] = random.uniform(-factor, factor)
                state["last_random_update"] = now
                
        elif behavior == "wandering":
            # Smooth wandering
            if now - state["last_random_update"] > 1.0:
                radius = entity.get("wander_radius", 0.2)
                state["random_offset_x"] = random.uniform(-radius, radius)
                state["random_offset_y"] = random.uniform(-radius, radius)
                state["last_random_update"] = now
        else:
            state["random_offset_x"] = 0.0
            state["random_offset_y"] = 0.0
            
        return state["random_offset_x"], state["random_offset_y"]

    def _update_entity(self, entity, now):
        """Update position of a single entity."""
        name = entity["name"]
        wps = entity["waypoints"]
        st = self.state[name]
        idx = st["idx"]
        nxt = (idx + 1) % len(wps)
        t = (now - st["start"]) / st["duration"]

        # Check if we've reached the next waypoint
        if t >= 1.0:
            st["idx"] = nxt
            st["start"] = now
            next_idx = (nxt + 1) % len(wps)
            st["duration"] = self._segment_duration(wps[nxt], wps[next_idx], entity["speed"])
            idx = st["idx"]
            nxt = (idx + 1) % len(wps)
            t = 0.0

        # Interpolate position
        ax, ay, ayaw = wps[idx]
        bx, by, byaw = wps[nxt]
        
        # Base position
        x = ax + (bx - ax) * t
        y = ay + (by - ay) * t
        yaw = ayaw + (byaw - ayaw) * t
        
        # Apply random offset for certain behaviors
        if self.enable_random and entity.get("type") == "human":
            offset_x, offset_y = self._get_random_offset(entity, st, now)
            x += offset_x
            y += offset_y

        # Create pose message
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = 0.0
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        # Create entity message
        gz_entity = Entity()
        gz_entity.name = name
        gz_entity.type = Entity.MODEL

        # Send request
        req = SetEntityPose.Request()
        req.entity = gz_entity
        req.pose = pose
        self.client.call_async(req)

    def _tick(self):
        """Timer callback to update all entities."""
        now = float(self.get_clock().now().seconds_nanoseconds()[0])
        for entity in self.entities:
            self._update_entity(entity, now)


def main():
    rclpy.init()
    node = MessyRoadMover()
    node.get_logger().info("Messy Road Mover node started (ROS2)")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""
ROS2 Node to publish poses of all entities (humans and vehicles) in the messy road simulation.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Header
import tf2_ros


class MessyRoadPosePublisher(Node):
    """
    Publisher node for tracking all dynamic entities (humans and vehicles)
    in the messy road simulation.
    """

    def __init__(self):
        super().__init__("messy_road_pose_publisher")
        
        # Parameters
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("rate", 10.0)
        
        # Human names with their heights for reference
        self.declare_parameter("human_names", [
            "human_tall_fast",      # 1.9m - Fast
            "human_avg_medium",     # 1.7m - Medium
            "human_short_slow",     # 1.5m - Slow
            "human_child_runner",   # 1.2m - Erratic/Fast
            "human_elderly_slow",   # 1.6m - Very slow
            "human_jogger",         # 1.75m - Fast jogger
            "human_phone_user",     # 1.68m - Slow, wandering
            "human_group_1",        # 1.72m - Group walking
            "human_group_2",        # 1.65m - Group walking
        ])
        
        # Vehicle names
        self.declare_parameter("vehicle_names", [
            "vehicle_car_fast",     # Fast blue car
            "vehicle_car_medium",   # Medium red car
            "vehicle_suv_slow",     # Slow white SUV
            "vehicle_truck_slow",   # Very slow delivery truck
            "vehicle_motorcycle",   # Fast motorcycle
            "vehicle_bus",          # Slow bus
            "vehicle_bicycle",      # Bicycle
        ])

        # Get parameters
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.human_names = list(self.get_parameter("human_names").value)
        self.vehicle_names = list(self.get_parameter("vehicle_names").value)
        self.rate_hz = self.get_parameter("rate").get_parameter_value().double_value

        # Publishers
        self.human_pub = self.create_publisher(PoseArray, "/human_poses", 10)
        self.vehicle_pub = self.create_publisher(PoseArray, "/vehicle_poses", 10)
        self.all_entities_pub = self.create_publisher(PoseArray, "/all_entity_poses", 10)
        
        # Individual pose publishers for specific tracking
        self.individual_pubs = {}
        for name in self.human_names + self.vehicle_names:
            self.individual_pubs[name] = self.create_publisher(
                PoseStamped, f"/{name}/pose", 10
            )

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timer
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self._tick)
        
        self.get_logger().info(
            f"Messy Road Pose Publisher started - tracking {len(self.human_names)} humans "
            f"and {len(self.vehicle_names)} vehicles"
        )

    def _get_pose_from_tf(self, entity_name):
        """Get pose of an entity from TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                entity_name,
                Time(),
            )
            
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException):
            return None

    def _tick(self):
        """Timer callback to publish all poses."""
        current_time = self.get_clock().now().to_msg()
        
        # Create headers
        header = Header()
        header.stamp = current_time
        header.frame_id = self.frame_id
        
        # Human poses
        human_poses = PoseArray()
        human_poses.header = header
        
        for name in self.human_names:
            pose = self._get_pose_from_tf(name)
            if pose:
                human_poses.poses.append(pose)
                
                # Publish individual pose
                pose_stamped = PoseStamped()
                pose_stamped.header = header
                pose_stamped.pose = pose
                self.individual_pubs[name].publish(pose_stamped)
        
        self.human_pub.publish(human_poses)
        
        # Vehicle poses
        vehicle_poses = PoseArray()
        vehicle_poses.header = header
        
        for name in self.vehicle_names:
            pose = self._get_pose_from_tf(name)
            if pose:
                vehicle_poses.poses.append(pose)
                
                # Publish individual pose
                pose_stamped = PoseStamped()
                pose_stamped.header = header
                pose_stamped.pose = pose
                self.individual_pubs[name].publish(pose_stamped)
        
        self.vehicle_pub.publish(vehicle_poses)
        
        # All entities combined
        all_poses = PoseArray()
        all_poses.header = header
        all_poses.poses = human_poses.poses + vehicle_poses.poses
        self.all_entities_pub.publish(all_poses)


def main():
    rclpy.init()
    node = MessyRoadPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

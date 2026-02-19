import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseArray, Pose
import tf2_ros


class HumanPosePublisher(Node):
    def __init__(self):
        super().__init__("human_pose_publisher")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("human_names", [
            "human_moving_1",
            "human_moving_2",
            "human_moving_3",
            "human_moving_4",
        ])
        self.declare_parameter("rate", 10.0)

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.human_names = list(self.get_parameter("human_names").value)
        self.rate_hz = self.get_parameter("rate").get_parameter_value().double_value

        self.pub = self.create_publisher(PoseArray, "/human_pose", 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self._tick)

    def _tick(self):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self.frame_id

        for name in self.human_names:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame_id,
                    name,
                    Time(),
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            pose_array.poses.append(pose)

        self.pub.publish(pose_array)


def main():
    rclpy.init()
    node = HumanPosePublisher()
    node.get_logger().info("Human pose publisher started (ROS2)")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

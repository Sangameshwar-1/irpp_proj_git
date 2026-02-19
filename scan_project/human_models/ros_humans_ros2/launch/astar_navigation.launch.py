from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file for A* path planning demonstration
    - Gazebo simulation with TurtleBot rover
    - NO moving humans (disabled temporarily)
    - A* path planner for goal-based navigation
    - Point cloud mapper for obstacle detection
    """
    world = LaunchConfiguration("world")

    # Get package share directory for world files
    pkg_share = get_package_share_directory("ros_humans_ros2")
    default_world = os.path.join(pkg_share, "worlds", "large_messy_room.world")

    gazebo_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    )

    return LaunchDescription([
        # Use simulation time for all nodes
        SetParameter(name='use_sim_time', value=True),
        
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="Gazebo world file",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={"gz_args": ["-r ", world]}.items(),
        ),
        # Clock bridge (Gazebo -> ROS2) - CRITICAL for time sync
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="clock_bridge",
            output="screen",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="pose_tf_bridge",
            output="screen",
            arguments=[
                "/world/large_messy_room/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
            ],
        ),
        # TurtleBot rover cmd_vel bridge (ROS2 -> Gazebo)
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="turtlebot_cmd_vel_bridge",
            output="screen",
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/model/turtlebot3_rover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
            ],
        ),
        # TurtleBot rover odometry bridge (Gazebo -> ROS2)
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="turtlebot_odom_bridge",
            output="screen",
            arguments=[
                "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"
            ],
        ),
        # TurtleBot LiDAR scan bridge (Gazebo -> ROS2)
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="turtlebot_scan_bridge",
            output="screen",
            arguments=[
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
            ],
        ),
        # Camera bridges (Gazebo -> ROS2)
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="camera_front_bridge",
            output="screen",
            arguments=[
                "/camera/front/image@sensor_msgs/msg/Image[gz.msgs.Image"
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="camera_right_bridge",
            output="screen",
            arguments=[
                "/camera/right/image@sensor_msgs/msg/Image[gz.msgs.Image"
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="camera_back_bridge",
            output="screen",
            arguments=[
                "/camera/back/image@sensor_msgs/msg/Image[gz.msgs.Image"
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="camera_left_bridge",
            output="screen",
            arguments=[
                "/camera/left/image@sensor_msgs/msg/Image[gz.msgs.Image"
            ],
        ),
        # A* Path Planner (replaces rover_explorer for goal-based navigation)
        Node(
            package="ros_humans_ros2",
            executable="astar_path_planner",
            name="astar_path_planner",
            output="screen",
            parameters=[{
                "linear_speed": 0.3,
                "angular_speed": 0.5,
                "goal_tolerance": 0.3,
                "grid_resolution": 0.2,
                "grid_size": 100,
                "obstacle_inflation": 0.4,
                "auto_start": True,
                "default_goal_x": 5.0,
                "default_goal_y": 5.0,
                "use_sim_time": True
            }],
        ),
        # Point cloud mapper (for visualization and mapping)
        Node(
            package="ros_humans_ros2",
            executable="pointcloud_mapper",
            name="pointcloud_mapper",
            output="screen",
            parameters=[{
                "map_frame": "map",
                "robot_frame": "base_footprint",
                "max_points": 100000,
                "use_sim_time": True
            }],
        ),
        # 360 camera view combiner
        Node(
            package="ros_humans_ros2",
            executable="camera_view_360",
            name="camera_view_360",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        
        # NOTE: Moving humans are DISABLED in this launch file
        # To re-enable, uncomment the following nodes:
        # Node(
        #     package="ros_gz_bridge",
        #     executable="parameter_bridge",
        #     name="set_pose_bridge",
        #     output="screen",
        #     arguments=[
        #         "/world/large_messy_room/set_pose@ros_gz_interfaces/srv/SetEntityPose"
        #     ],
        # ),
        # Node(
        #     package="ros_humans_ros2",
        #     executable="move_humans",
        #     name="move_humans",
        #     output="screen",
        #     parameters=[{"rate": 10.0, "world": "large_messy_room", "use_sim_time": True}],
        # ),
        # Node(
        #     package="ros_humans_ros2",
        #     executable="publish_human_pose",
        #     name="human_pose_publisher",
        #     output="screen",
        #     parameters=[{"frame_id": "world", "use_sim_time": True}],
        # ),
    ])

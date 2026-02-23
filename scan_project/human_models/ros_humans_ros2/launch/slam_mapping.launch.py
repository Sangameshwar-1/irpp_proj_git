"""
SLAM Mapping Launch File - Clean reliable version
Gazebo Harmonic + slam_toolbox + Frontier Exploration
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("ros_humans_ros2")
    default_world = os.path.join(pkg_share, "worlds", "large_messy_room.world")
    gazebo_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )

    world        = LaunchConfiguration("world")
    auto_explore = LaunchConfiguration("auto_explore")
    explore_time = LaunchConfiguration("exploration_time")

    sim_time = {"use_sim_time": True}

    return LaunchDescription([
        # ── Arguments ──────────────────────────────────────────────────────
        DeclareLaunchArgument("world",            default_value=default_world),
        DeclareLaunchArgument("auto_explore",     default_value="true"),
        DeclareLaunchArgument("exploration_time", default_value="300.0"),

        LogInfo(msg="=== SLAM Mapping: starting Gazebo ==="),

        # ── Gazebo ─────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={"gz_args": ["-r ", world]}.items(),
        ),

        # ── ROS <-> Gazebo Bridges ─────────────────────────────────────────
        # Clock  (Gazebo -> ROS) – needed for use_sim_time
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="clock_bridge", output="screen",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            parameters=[sim_time],
        ),
        # cmd_vel  (ROS -> Gazebo) – matches <topic>/cmd_vel</topic> in world
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="cmd_vel_bridge", output="screen",
            arguments=["/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],
            parameters=[sim_time],
        ),
        # odom  (Gazebo -> ROS) – DiffDrive publishes gz.msgs.Odometry (confirmed)
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="odom_bridge", output="screen",
            arguments=[
                "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"
            ],
            parameters=[sim_time],
        ),
        # scan  (Gazebo -> ROS)
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="scan_bridge", output="screen",
            arguments=["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
            parameters=[sim_time],
        ),

        # ── Static TF: robot body frames ───────────────────────────────────
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="base_footprint_to_base_link",
            arguments=["--x", "0", "--y", "0", "--z", "0.08",
                       "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", "base_footprint",
                       "--child-frame-id", "base_link"],
            parameters=[sim_time],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="base_link_to_lidar_link",
            arguments=["--x", "0", "--y", "0", "--z", "0.1",
                       "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", "base_link",
                       "--child-frame-id", "lidar_link"],
            parameters=[sim_time],
        ),

        # ── odom_tf_publisher ──────────────────────────────────────────────
        # Subscribes /odom -> publishes odom->base_footprint TF.
        # Filters stale Gazebo bridge buffer messages automatically.
        Node(
            package="ros_humans_ros2", executable="odom_tf_publisher",
            name="odom_tf_publisher", output="screen",
            parameters=[sim_time],
        ),

        # ── slam_toolbox (starts after 6s to let Gazebo/bridges be ready) ──
        TimerAction(period=6.0, actions=[
            LogInfo(msg="=== Starting slam_toolbox ==="),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox", output="screen",
                parameters=[{
                    **sim_time,
                    "odom_frame":  "odom",
                    "map_frame":   "map",
                    "base_frame":  "base_footprint",
                    "scan_topic":  "/scan",
                    "mode":        "mapping",
                    # TF
                    "transform_timeout":        0.5,
                    "tf_buffer_duration":       30.0,
                    "transform_publish_period": 0.05,
                    # Map
                    "resolution":          0.05,
                    "max_laser_range":     12.0,
                    "map_update_interval": 2.0,
                    # Accept scans with minimal movement
                    "minimum_time_interval":    0.5,
                    "minimum_travel_distance":  0.1,
                    "minimum_travel_heading":   0.1,
                    # Scan matching
                    "use_scan_matching":  True,
                    "use_scan_barycenter": True,
                    "scan_buffer_size":   10,
                    "scan_buffer_maximum_scan_distance": 10.0,
                    "link_match_minimum_response_fine":  0.1,
                    "link_scan_maximum_distance":        1.5,
                    # Solver
                    "solver_plugin":       "solver_plugins::CeresSolver",
                    "ceres_linear_solver": "SPARSE_NORMAL_CHOLESKY",
                    "ceres_preconditioner": "SCHUR_JACOBI",
                    "ceres_trust_strategy": "LEVENBERG_MARQUARDT",
                    "ceres_dogleg_type":    "TRADITIONAL_DOGLEG",
                    "ceres_loss_function":  "None",
                    "stack_size_to_use":    40000000,
                    # Loop closure
                    "do_loop_closing":              True,
                    "loop_search_maximum_distance": 3.0,
                    "loop_match_minimum_chain_size": 10,
                }],
            ),
        ]),

        # ── Frontier Explorer (starts after 15s, after SLAM is up) ─────────
        # Use the existing frontier_explorer implementation for exploration
        TimerAction(period=15.0, actions=[
            LogInfo(msg="=== Starting frontier_explorer ==="),
            Node(
                package="ros_humans_ros2", executable="frontier_explorer",
                name="frontier_explorer", output="screen",
                parameters=[{
                    **sim_time,
                    "linear_speed":        0.20,
                    "angular_speed":       0.45,
                    "obstacle_threshold":  0.30,
                    "exploration_timeout": explore_time,
                    "goal_tolerance":      0.5,
                    "min_frontier_size":   5,
                }],
                condition=IfCondition(auto_explore),
            ),
        ]),
    ])

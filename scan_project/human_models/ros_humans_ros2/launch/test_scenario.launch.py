"""
Test Scenario Launch File
==========================
A parameterized launch file for mini test environments.

Each shell script sets the scenario-specific arguments:
  world_file, world_name, map_yaml, spawn_x/y/yaw, goal_x/y, room bounds.

Usage:
    ros2 launch ros_humans_ros2 test_scenario.launch.py \
        world_name:=test_corridor_blocked \
        world_file:=/path/to/test_corridor_blocked.world \
        map_yaml:=$HOME/ros2_maps/test_corridor_blocked.yaml \
        spawn_x:=-4.0  spawn_y:=0.0  spawn_yaw:=0.0 \
        goal_x:=4.0    goal_y:=0.0 \
        room_min_x:=-5.5 room_max_x:=5.5 \
        room_min_y:=-2.0 room_max_y:=2.0
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def _launch_setup(context, *args, **kwargs):
    """Resolve all LaunchConfiguration values at launch time so we can
    build dynamic strings (topic names, file paths) in normal Python."""

    world_name = context.launch_configurations["world_name"]
    world_file = context.launch_configurations["world_file"]
    map_yaml = context.launch_configurations["map_yaml"]
    spawn_x = float(context.launch_configurations["spawn_x"])
    spawn_y = float(context.launch_configurations["spawn_y"])
    spawn_yaw = float(context.launch_configurations["spawn_yaw"])
    goal_x = float(context.launch_configurations["goal_x"])
    goal_y = float(context.launch_configurations["goal_y"])
    room_min_x = float(context.launch_configurations["room_min_x"])
    room_max_x = float(context.launch_configurations["room_max_x"])
    room_min_y = float(context.launch_configurations["room_min_y"])
    room_max_y = float(context.launch_configurations["room_max_y"])
    map_size = float(context.launch_configurations["map_size"])

    gazebo_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch", "gz_sim.launch.py")

    set_pose_topic = f"/world/{world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose"

    return [
        # ═══════════════════════════════════════════════════════════
        #  SIMULATION
        # ═══════════════════════════════════════════════════════════

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={"gz_args": f"-r {world_file}"}.items()),

        # Clock bridge
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="clock_bridge", output="screen",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]),

        # cmd_vel bridge
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="cmd_vel_bridge", output="screen",
             arguments=[
                 "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                 "/model/turtlebot3_rover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"]),

        # Odometry bridge
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="odom_bridge", output="screen",
             arguments=["/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"]),

        # LiDAR bridge
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="scan_bridge", output="screen",
             arguments=["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"]),

        # Camera bridges (4 directions)
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="cam_front", output="screen",
             arguments=["/camera/front/image@sensor_msgs/msg/Image[gz.msgs.Image"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="cam_right", output="screen",
             arguments=["/camera/right/image@sensor_msgs/msg/Image[gz.msgs.Image"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="cam_back", output="screen",
             arguments=["/camera/back/image@sensor_msgs/msg/Image[gz.msgs.Image"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="cam_left", output="screen",
             arguments=["/camera/left/image@sensor_msgs/msg/Image[gz.msgs.Image"]),

        # ═══════════════════════════════════════════════════════════
        #  STATIC MAP
        # ═══════════════════════════════════════════════════════════

        Node(package="ros_humans_ros2", executable="map_publisher",
             name="map_publisher", output="screen",
             parameters=[{"yaml_file": map_yaml,
                           "publish_rate": 0.5,
                           "frame_id": "map",
                           "use_sim_time": True}]),

        # ═══════════════════════════════════════════════════════════
        #  TF + POINT CLOUDS
        # ═══════════════════════════════════════════════════════════

        Node(package="ros_humans_ros2", executable="pointcloud_mapper",
             name="pointcloud_mapper", output="screen",
             parameters=[{"map_frame": "map",
                           "robot_frame": "base_footprint",
                           "max_points": 10000,
                           "map_resolution": 0.1,
                           "map_size": map_size,
                           "spawn_x": spawn_x,
                           "spawn_y": spawn_y,
                           "spawn_yaw": spawn_yaw,
                           "publish_accumulated_cloud": True,
                           "use_sim_time": True}]),

        # 360 camera combiner
        Node(package="ros_humans_ros2", executable="camera_view_360",
             name="camera_view_360", output="screen",
             parameters=[{"use_sim_time": True}]),

        # ═══════════════════════════════════════════════════════════
        #  SET POSE BRIDGE (needed for move_humans GT publisher)
        # ═══════════════════════════════════════════════════════════

        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="set_pose_bridge", output="screen",
             arguments=[set_pose_topic]),

        # move_humans — publishes /human_ground_truth
        # Static humans: empty humans_json means no waypoint movement
        Node(package="ros_humans_ros2", executable="move_humans",
             name="move_humans", output="screen",
             parameters=[{"rate": 10.0,
                           "world": world_name,
                           "humans_json": "[]",
                           "use_sim_time": True}]),

        # ═══════════════════════════════════════════════════════════
        #  NAVIGATION STACK
        # ═══════════════════════════════════════════════════════════

        # Localization
        Node(package="ros_humans_ros2", executable="localization_node",
             name="localization_node", output="screen",
             parameters=[{"spawn_x": spawn_x,
                           "spawn_y": spawn_y,
                           "spawn_yaw": spawn_yaw,
                           "max_velocity": 0.6,
                           "max_yaw_rate": 2.5,
                           "use_sim_time": True}]),

        # Human detector (RED shapes)
        Node(package="ros_humans_ros2", executable="human_detector_red",
             name="human_detector_red", output="screen",
             parameters=[{"detection_rate": 5.0,
                           "min_contour_area": 40,
                           "max_detect_range": 8.0,
                           "track_timeout": 3.0,
                           "red_h_low1": 0,
                           "red_h_high1": 10,
                           "red_h_low2": 170,
                           "red_h_high2": 180,
                           "red_s_min": 50,
                           "red_v_min": 80,
                           "cam_focal_px": 320.0,
                           "reference_size": 0.5,
                           "cam_hfov_deg": 90.0,
                           "show_debug_window": True,
                           "use_sim_time": True}]),

        # Global planner (A* on weighted grid)
        Node(package="ros_humans_ros2", executable="global_planner",
             name="global_planner", output="screen",
             parameters=[{"planner_resolution": 0.2,
                           "inflation_radius": 0.35,
                           "goal_tolerance_m": 0.3,
                           "auto_start": True,
                           "default_goal_x": goal_x,
                           "default_goal_y": goal_y,
                           "room_min_x": room_min_x,
                           "room_max_x": room_max_x,
                           "room_min_y": room_min_y,
                           "room_max_y": room_max_y,
                           "use_sim_time": True}]),

        # Local planner (path following + WAIT / REROUTE)
        Node(package="ros_humans_ros2", executable="local_planner",
             name="local_planner", output="screen",
             parameters=[{"linear_speed": 0.3,
                           "angular_speed": 0.5,
                           "waypoint_tolerance": 0.3,
                           "human_threat_dist": 3.0,
                           "human_zone_radius": 1.5,
                           "weight_multiplier": 10.0,
                           "max_wait_time": 8.0,
                           "crossing_clear_time": 5.0,
                           "predict_horizon": 3.0,
                           "emergency_stop_dist": 0.4,
                           "use_sim_time": True}]),

        # Live visualization (GT + perception maps + social circles)
        Node(package="ros_humans_ros2", executable="live_visualization_node",
             name="live_visualization_node", output="screen",
             parameters=[{"resolution": 0.2,
                           "map_size": map_size,
                           "publish_rate": 2.0,
                           "human_radius_cells": 3,
                           "spawn_x": spawn_x,
                           "spawn_y": spawn_y,
                           "spawn_yaw": spawn_yaw,
                           "use_sim_time": True}]),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("ros_humans_ros2")
    default_world = os.path.join(pkg_share, "worlds", "test_corridor_blocked.world")
    default_map = os.path.join(
        os.path.expanduser("~"), "ros2_maps", "test_corridor_blocked.yaml")

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),

        # ── Declare all arguments with defaults (corridor_blocked) ──
        DeclareLaunchArgument("world_name", default_value="test_corridor_blocked",
                              description="World name (matches <world name=...>)"),
        DeclareLaunchArgument("world_file", default_value=default_world,
                              description="Full path to .world file"),
        DeclareLaunchArgument("map_yaml", default_value=default_map,
                              description="Path to the map YAML file"),
        DeclareLaunchArgument("spawn_x", default_value="-4.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("goal_x", default_value="4.0"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
        DeclareLaunchArgument("room_min_x", default_value="-5.5"),
        DeclareLaunchArgument("room_max_x", default_value="5.5"),
        DeclareLaunchArgument("room_min_y", default_value="-2.0"),
        DeclareLaunchArgument("room_max_y", default_value="2.0"),
        DeclareLaunchArgument("map_size", default_value="14.0",
                              description="Map size in metres for pointcloud/viz"),

        # Resolve all configs and build nodes via OpaqueFunction
        OpaqueFunction(function=_launch_setup),
    ])

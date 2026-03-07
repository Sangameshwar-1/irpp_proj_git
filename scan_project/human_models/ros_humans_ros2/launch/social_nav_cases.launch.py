"""
Social Navigation Cases Launch File
=====================================
Minimal launch for planning-only tests.  Perception is BYPASSED entirely:
  • No cameras, no human_detector_red
  • human_case_controller publishes ground-truth /detected_humans + /human_velocities
  • social_nav_planner subscribes and drives the robot

Nodes launched
--------------
  Simulation:     Gazebo (test_case_arena.world), bridges (clock/cmd_vel/odom/scan/set_pose)
  Planning:       global_planner (A*), social_nav_planner (case-based decisions)
  Ground truth:   human_case_controller (animates human + publishes GT poses/vels)
  Localization:   localization_node, map_publisher, pointcloud_mapper

Launch arguments
----------------
  case        — case1 | case2 | case3 | case4a | case4b   (default: case1)
  goal_x/y    — navigation goal  (default: 4.5, 0.0)
  spawn_x/y   — robot spawn position  (default: -4.0, 0.0)
  spawn_yaw   — robot spawn yaw  (default: 0.0)
  world_file  — absolute path to .world file
  map_yaml    — absolute path to map YAML
  room_min/max_x/y — occupancy-grid bounds for A*

Usage example (via run_test_case.sh):
  ros2 launch ros_humans_ros2 social_nav_cases.launch.py case:=case3
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
    world_name  = context.launch_configurations["world_name"]
    world_file  = context.launch_configurations["world_file"]
    map_yaml    = context.launch_configurations["map_yaml"]
    case_name   = context.launch_configurations["case"]
    spawn_x     = float(context.launch_configurations["spawn_x"])
    spawn_y     = float(context.launch_configurations["spawn_y"])
    spawn_yaw   = float(context.launch_configurations["spawn_yaw"])
    goal_x      = float(context.launch_configurations["goal_x"])
    goal_y      = float(context.launch_configurations["goal_y"])
    room_min_x  = float(context.launch_configurations["room_min_x"])
    room_max_x  = float(context.launch_configurations["room_max_x"])
    room_min_y  = float(context.launch_configurations["room_min_y"])
    room_max_y  = float(context.launch_configurations["room_max_y"])
    map_size    = float(context.launch_configurations["map_size"])

    gazebo_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch", "gz_sim.launch.py")

    set_pose_topic = (
        f"/world/{world_name}/set_pose"
        f"@ros_gz_interfaces/srv/SetEntityPose")

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

        # cmd_vel bridge (ROS2 → Gazebo)
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="cmd_vel_bridge", output="screen",
             arguments=[
                 "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                 "/model/turtlebot3_rover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"]),

        # Odometry bridge (Gazebo → ROS2)
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="odom_bridge", output="screen",
             arguments=["/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"]),

        # LiDAR bridge (emergency stop only — perception not used)
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="scan_bridge", output="screen",
             arguments=["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"]),

        # set_pose bridge (human_case_controller teleports human model)
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="set_pose_bridge", output="screen",
             arguments=[set_pose_topic]),

        # ═══════════════════════════════════════════════════════════
        #  MAP + LOCALIZATION
        # ═══════════════════════════════════════════════════════════

        Node(package="ros_humans_ros2", executable="map_publisher",
             name="map_publisher", output="screen",
             parameters=[{"yaml_file": map_yaml,
                           "publish_rate": 0.5,
                           "frame_id": "map",
                           "use_sim_time": True}]),

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

        Node(package="ros_humans_ros2", executable="localization_node",
             name="localization_node", output="screen",
             parameters=[{"spawn_x": spawn_x,
                           "spawn_y": spawn_y,
                           "spawn_yaw": spawn_yaw,
                           "max_velocity": 0.6,
                           "max_yaw_rate": 2.5,
                           "search_xy":   0.15,      # tight for small symmetric arena
                           "search_yaw":  0.08,      # ~4.6° max
                           "correction_alpha": 0.3,   # smoother blending
                           "use_sim_time": True}]),

        # ═══════════════════════════════════════════════════════════
        #  GROUND-TRUTH HUMAN CONTROLLER  (replaces perception)
        # ═══════════════════════════════════════════════════════════

        Node(package="ros_humans_ros2", executable="human_case_controller",
             name="human_case_controller", output="screen",
             parameters=[{"case":       case_name,
                           "world_name": world_name,
                           "model_name": "human_1",
                           "rate":       20.0,
                           "use_sim_time": True}]),

        # ═══════════════════════════════════════════════════════════
        #  PLANNING
        # ═══════════════════════════════════════════════════════════

        # Global planner (A* with weight zones)
        Node(package="ros_humans_ros2", executable="global_planner",
             name="global_planner", output="screen",
             parameters=[{"planner_resolution": 0.2,
                           "inflation_radius":   0.35,
                           "goal_tolerance_m":   0.30,
                           "auto_start":         True,
                           "default_goal_x":     goal_x,
                           "default_goal_y":     goal_y,
                           "room_min_x":         room_min_x,
                           "room_max_x":         room_max_x,
                           "room_min_y":         room_min_y,
                           "room_max_y":         room_max_y,
                           # Proactive monitoring: replan if human is near path
                           "proactive_replan_radius": 0.75,
                           "proactive_zone_radius":   0.80,
                           "proactive_weight":       20.0,
                           "proactive_cooldown":     5.0,
                           "use_sim_time": True}]),

        # Social nav planner — replaces local_planner for test cases
        Node(package="ros_humans_ros2", executable="social_nav_planner",
             name="social_nav_planner", output="screen",
             parameters=[{"max_speed":    0.30,
                           "angular_speed": 0.30,
                           "use_sim_time": True}]),

        # ═══════════════════════════════════════════════════════════
        #  VISUALISATION
        # ═══════════════════════════════════════════════════════════

        Node(package="ros_humans_ros2", executable="live_visualization_node",
             name="live_visualization_node", output="screen",
             parameters=[{"resolution": 0.2,
                           "map_size":    map_size,
                           "publish_rate": 2.0,
                           "human_radius_cells": 3,
                           "spawn_x":   spawn_x,
                           "spawn_y":   spawn_y,
                           "spawn_yaw": spawn_yaw,
                           "use_sim_time": True}]),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("ros_humans_ros2")
    default_world = os.path.join(
        pkg_share, "worlds", "test_case_arena.world")
    default_map = os.path.join(
        os.path.expanduser("~"), "ros2_maps", "test_case_arena.yaml")

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),

        # ── Declare all arguments ──────────────────────────────────
        DeclareLaunchArgument(
            "case", default_value="case1",
            description="Planning test case: case1|case2|case3|case4a|case4b"),
        DeclareLaunchArgument(
            "world_name", default_value="test_case_arena",
            description="Gazebo world name (matches <world name=...>)"),
        DeclareLaunchArgument(
            "world_file", default_value=default_world,
            description="Full path to .world file"),
        DeclareLaunchArgument(
            "map_yaml", default_value=default_map,
            description="Full path to map YAML (generate via world_to_map first)"),
        DeclareLaunchArgument("spawn_x",   default_value="-4.0"),
        DeclareLaunchArgument("spawn_y",   default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("goal_x",    default_value="4.5"),
        DeclareLaunchArgument("goal_y",    default_value="0.0"),
        DeclareLaunchArgument("room_min_x", default_value="-6.5"),
        DeclareLaunchArgument("room_max_x", default_value="6.5"),
        DeclareLaunchArgument("room_min_y", default_value="-4.5"),
        DeclareLaunchArgument("room_max_y", default_value="4.5"),
        DeclareLaunchArgument("map_size",   default_value="16.0",
                              description="Map extent for pointcloud/viz nodes"),

        OpaqueFunction(function=_launch_setup),
    ])

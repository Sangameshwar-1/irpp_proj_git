"""
Navigation Launch File — Large World (Social Nav Planner)
=========================================================
Launches the complete navigation stack with social-aware planner:

    ┌──────────────────┐
    │  Gazebo + Bridges │  (simulation + sensor data)
    └────────┬─────────┘
             │  /odom, /scan, /camera/*
             ▼
    ┌──────────────────┐       ┌──────────────────┐
    │  map_publisher   │       │ localization_node │
    │  (/map static)   │       │ (/robot_pose)     │
    └────────┬─────────┘       └────────┬──────────┘
             │                          │
             ▼                          ▼
    ┌──────────────────────────────────────────────┐
    │             global_planner                    │
    │  (A* + proactive replan → /global_path)      │
    │  Receives /weight_zones + /replan_request    │
    └────────────────────┬─────────────────────────┘
                         │
                         ▼
    ┌──────────────────────────────────────────────┐
    │          social_nav_planner                   │
    │  (case-based VO, cone speed, directional      │
    │   routing, exit-replan, parallel turns)       │
    │  → /cmd_vel,  → /weight_zones + /replan_req  │
    └──────────────────────────────────────────────┘
                         ▲
              /detected_humans  /human_velocities
    ┌──────────────────────────────────────────────┐
    │              move_humans                      │
    │  (GT positions + velocities for 5 cases:     │
    │   case1 static, case2 head-on, case3 cross,  │
    │   case4a ahead, case5 fast-behind)            │
    └──────────────────────────────────────────────┘
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os

_DEFAULT_MAP_YAML = os.path.join(
    os.path.expanduser("~"), "ros2_maps", "large_messy_room_map.yaml"
)


def generate_launch_description():
    world = LaunchConfiguration("world")
    default_goal_x = LaunchConfiguration("default_goal_x")
    default_goal_y = LaunchConfiguration("default_goal_y")
    map_yaml = LaunchConfiguration("map_yaml")
    perception_mode = LaunchConfiguration("perception_mode")

    # Conditions: camera vs GT
    is_camera = PythonExpression(["'", perception_mode, "' == 'camera'"])
    is_gt     = PythonExpression(["'", perception_mode, "' != 'camera'"])

    pkg_share = get_package_share_directory("ros_humans_ros2")
    default_world = os.path.join(pkg_share, "worlds", "large_messy_room.world")
    gazebo_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch", "gz_sim.launch.py")

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),

        # ── Launch arguments ──────────────────────────────────────────
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("default_goal_x", default_value="5.0"),
        DeclareLaunchArgument("default_goal_y", default_value="5.0"),
        DeclareLaunchArgument("map_yaml", default_value=_DEFAULT_MAP_YAML),
        DeclareLaunchArgument(
            "perception_mode",
            default_value="gt",
            description="Perception mode: 'gt' = ground-truth injection via move_humans, "
                        "'camera' = HSV+Kalman detection via human_detector_red"),


        # ═══════════════════════════════════════════════════════════════
        #  SIMULATION  (unchanged from previous architecture)
        # ═══════════════════════════════════════════════════════════════

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={"gz_args": ["-r ", world]}.items()),

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

        # ═══════════════════════════════════════════════════════════════
        #  STATIC MAP  (unchanged)
        # ═══════════════════════════════════════════════════════════════

        Node(package="ros_humans_ros2", executable="map_publisher",
             name="map_publisher", output="screen",
             parameters=[{"yaml_file": map_yaml,
                           "publish_rate": 0.5,
                           "frame_id": "map",
                           "use_sim_time": True}]),

        # ═══════════════════════════════════════════════════════════════
        #  TF + POINT CLOUDS  (unchanged — provides TF tree for RViz)
        # ═══════════════════════════════════════════════════════════════

        Node(package="ros_humans_ros2", executable="pointcloud_mapper",
             name="pointcloud_mapper", output="screen",
             parameters=[{"map_frame": "map",
                           "robot_frame": "base_footprint",
                           "max_points": 10000,
                           "map_resolution": 0.1,
                           "map_size": 28.0,
                           "spawn_x": 0.0,
                           "spawn_y": -8.0,
                           "spawn_yaw": 1.5708,
                           "publish_accumulated_cloud": True,
                           "use_sim_time": True}]),

        # 360 camera combiner
        Node(package="ros_humans_ros2", executable="camera_view_360",
             name="camera_view_360", output="screen",
             parameters=[{"use_sim_time": True}]),

        # ═══════════════════════════════════════════════════════════════
        #  HUMANS  (move_humans unchanged, detector → RED only)
        # ═══════════════════════════════════════════════════════════════

        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="set_pose_bridge", output="screen",
             arguments=[
                 "/world/large_messy_room/set_pose"
                 "@ros_gz_interfaces/srv/SetEntityPose"]),

        # ── move_humans: GT mode — teleports humans AND publishes /detected_humans ──
        Node(package="ros_humans_ros2", executable="move_humans",
             name="move_humans", output="screen",
             parameters=[{"rate": 10.0,
                           "world": "large_messy_room",
                           "publish_detections": True,
                           "use_sim_time": True}],
             condition=IfCondition(is_gt)),

        # ── move_humans: camera mode — teleports humans only, no GT publishing ──
        Node(package="ros_humans_ros2", executable="move_humans",
             name="move_humans", output="screen",
             parameters=[{"rate": 10.0,
                           "world": "large_messy_room",
                           "publish_detections": False,
                           "use_sim_time": True}],
             condition=IfCondition(is_camera)),

        # ── human_detector_red: camera mode only ─────────────────────
        # HSV red-blob detection + Kalman tracker (Hungarian assignment)
        # Reads 4 bridged camera feeds → publishes /detected_humans
        Node(package="ros_humans_ros2", executable="human_detector_red",
             name="human_detector_red", output="screen",
             parameters=[{"detection_rate": 5.0,
                           "min_contour_area": 40,
                           "max_detect_range": 12.0,
                           "track_timeout": 1.5,
                           "publish_timeout": 0.6,
                           "red_h_low1": 0,
                           "red_h_high1": 15,
                           "red_h_low2": 165,
                           "red_h_high2": 180,
                           "red_s_min": 50,
                           "red_v_min": 50,
                           "cam_focal_px": 320.0,
                           "reference_size": 0.5,
                           "reference_height": 0.5,
                           "cam_hfov_deg": 90.0,
                           "use_lidar_fusion": True,
                           "lidar_bearing_tolerance": 0.15,
                           "show_debug_window": True,
                           "use_sim_time": True}],
             condition=IfCondition(is_camera)),

        # ═══════════════════════════════════════════════════════════════
        #  NEW ARCHITECTURE NODES
        # ═══════════════════════════════════════════════════════════════

        # ── 1. Localization ───────────────────────────────────────────
        # Fuses odom + spawn offset, publishes /robot_pose in map frame
        Node(package="ros_humans_ros2", executable="localization_node",
             name="localization_node", output="screen",
             parameters=[{"spawn_x": 0.0,
                           "spawn_y": -8.0,
                           "spawn_yaw": 1.5708,
                           "max_velocity": 0.6,
                           "max_yaw_rate": 2.5,
                           "use_sim_time": True}]),

        # ── 2. Human detection ────────────────────────────────────────────
        #      GT mode:     move_humans publishes /detected_humans directly
        #      Camera mode: human_detector_red (HSV + Kalman) reads cameras
        #      Both modes:  move_humans teleports humans in Gazebo

        # ── 3. Global planner (A* on weighted grid) ──────────────────
        Node(package="ros_humans_ros2", executable="global_planner",
             name="global_planner", output="screen",
             parameters=[{"planner_resolution": 0.2,
                           "inflation_radius": 0.35,
                           "goal_tolerance_m": 0.3,
                           "auto_start": True,
                           "default_goal_x": default_goal_x,
                           "default_goal_y": default_goal_y,
                           "room_min_x": -13.0,
                           "room_max_x": 13.0,
                           "room_min_y": -13.0,
                           "room_max_y": 13.0,
                           "proactive_replan_radius": 0.75,
                           "proactive_zone_radius": 0.80,
                           "proactive_weight": 20.0,
                           "proactive_cooldown": 5.0,
                           "use_sim_time": True}]),

        # ── 4. Social-nav planner (case-based VO + directional routing) ──
        Node(package="ros_humans_ros2", executable="social_nav_planner",
             name="social_nav_planner", output="screen",
             parameters=[{"max_speed": 0.30,
                           "angular_speed": 0.30,
                           "use_sim_time": True}]),

        # ── 5. Live visualization (GT + perception maps + social circles) ──
        Node(package="ros_humans_ros2", executable="live_visualization_node",
             name="live_visualization_node", output="screen",
             parameters=[{"resolution": 0.2,
                           "map_size": 28.0,
                           "publish_rate": 2.0,
                           "human_radius_cells": 3,
                           "spawn_x": 0.0,
                           "spawn_y": -8.0,
                           "spawn_yaw": 1.5708,
                           "use_sim_time": True}]),
    ])

#!/usr/bin/env python3
"""
World to Map Launch File
=========================
Automatically converts a .world file to a map and displays it in RViz.
This launch file:
1. Converts the .world file to PGM+YAML map files
2. Publishes the map to /map topic
3. Launches RViz with the map visualization

Usage
-----
  # Use default world (large_messy_room.world)
  ros2 launch ros_humans_ros2 world_to_map.launch.py

  # Use specific world file
  ros2 launch ros_humans_ros2 world_to_map.launch.py world:=messy_road.world

  # Custom resolution and output directory
  ros2 launch ros_humans_ros2 world_to_map.launch.py world:=messy_road.world resolution:=0.03 output_dir:=/tmp/maps
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory("ros_humans_ros2")
    
    # Declare arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="large_messy_room.world",
        description="World file name (in package worlds directory) or absolute path",
    )
    
    resolution_arg = DeclareLaunchArgument(
        "resolution",
        default_value="0.05",
        description="Map resolution in meters per pixel",
    )
    
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value=os.path.join(pkg_share, "maps"),
        description="Output directory for generated maps",
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Whether to launch RViz for visualization",
    )
    
    # Get launch configurations
    world = LaunchConfiguration("world")
    resolution = LaunchConfiguration("resolution")
    output_dir = LaunchConfiguration("output_dir")
    launch_rviz = LaunchConfiguration("launch_rviz")
    
    # Determine world file path
    # If world is just a filename, look in package worlds directory
    # Otherwise, use as absolute path
    default_world_dir = os.path.join(pkg_share, "worlds")
    
    # Python expression to determine world path
    world_path_expr = PythonExpression([
        "'", world, "' if os.path.isabs('", world, "') else os.path.join('", 
        default_world_dir, "', '", world, "')"
    ])
    
    # Python expression to determine output path (without extension)
    output_path_expr = PythonExpression([
        "os.path.join('", output_dir, "', ",
        "os.path.splitext(os.path.basename('", world, "'))[0] + '_map')"
    ])
    
    # Step 1: Convert world file to map
    world_to_map_process = ExecuteProcess(
        cmd=[
            "python3", "-m", "ros_humans_ros2.world_to_map",
            world_path_expr,
            "-o", output_path_expr,
            "-r", resolution,
            "--inflate", "0.1",
            "--padding", "1.0"
        ],
        output="screen",
        shell=False,
    )
    
    # Step 2: Publish the generated map
    # This will be started after world_to_map completes
    yaml_file_expr = PythonExpression([
        output_path_expr, " + '.yaml'"
    ])
    
    map_publisher_node = Node(
        package="ros_humans_ros2",
        executable="map_publisher",
        name="map_publisher",
        output="screen",
        parameters=[{
            "yaml_file": yaml_file_expr,
            "publish_rate": 1.0,
            "frame_id": "map",
        }],
    )
    
    # Step 3: Launch RViz (conditional)
    rviz_config = os.path.join(pkg_share, "..", "..", "rviz_config.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=launch_rviz,
    )
    
    # Event handler to start map publisher after conversion completes
    start_map_publisher_event = RegisterEventHandler(
        OnProcessExit(
            target_action=world_to_map_process,
            on_exit=[
                LogInfo(msg="World file conversion complete. Starting map publisher..."),
                map_publisher_node,
                rviz_node,
            ],
        )
    )
    
    return LaunchDescription([
        world_arg,
        resolution_arg,
        output_dir_arg,
        launch_rviz_arg,
        world_to_map_process,
        start_map_publisher_event,
    ])

"""
astar_with_gazebo.launch.py
============================
Combined launch that starts the Gazebo simulation (via ros_gz_sim / slam mapping
launch) and the lightweight A* planner node (`astar_navigation.launch.py`).

Usage:
  ros2 launch ros_humans_ros2 astar_with_gazebo.launch.py

This file simply includes the existing `slam_mapping.launch.py` (which brings
up the Gazebo world and map_server) and the `astar_navigation.launch.py` node
launch so everything is started under one command.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ros_humans_ros2')

    slam_launch = os.path.join(pkg_share, 'launch', 'slam_mapping.launch.py')
    astar_launch = os.path.join(pkg_share, 'launch', 'astar_navigation.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(astar_launch),
        ),
    ])

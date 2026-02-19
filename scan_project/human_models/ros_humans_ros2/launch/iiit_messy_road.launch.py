#!/usr/bin/env python3
"""
Launch file for IIIT Hyderabad Messy Road simulation.
Launches Gazebo with the merged IIIT campus + messy road world
and starts the mover node for humans and vehicles.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get package share directory for world files
    pkg_share = get_package_share_directory('ros_humans_ros2')
    world_file = os.path.join(pkg_share, 'worlds', 'iiit_messy_road.world')
    
    # Get ros_gz_sim launch file
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
    
    world = LaunchConfiguration('world')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Path to the world file'
        ),
        
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={
                'gz_args': ['-r ', world],
            }.items(),
        ),
        
        # Bridge for pose information (TF)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='pose_tf_bridge',
            arguments=['/world/iiit_messy_road/pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
            output='screen'
        ),
        
        # Bridge for SetEntityPose service
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='set_pose_bridge',
            arguments=['/world/iiit_messy_road/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
            output='screen'
        ),
        
        # Human and vehicle mover node
        Node(
            package='ros_humans_ros2',
            executable='iiit_messy_road_mover',
            name='iiit_messy_road_mover',
            output='screen'
        ),
        
        # Pose publisher node (optional, for debugging)
        Node(
            package='ros_humans_ros2',
            executable='publish_human_pose',
            name='human_pose_publisher',
            output='screen'
        ),
    ])

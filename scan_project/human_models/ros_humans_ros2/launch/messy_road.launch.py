"""
Launch file for the Messy Road simulation with humans and vehicles.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='/home/sangam/Documents/Acad/sem-4/IRPP/PROJ/scan_project/gazebo_worlds/messy_road.world',
        description='Path to Gazebo world file'
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='20.0',
        description='Update rate in Hz'
    )
    
    enable_random_arg = DeclareLaunchArgument(
        'enable_random_behavior',
        default_value='true',
        description='Enable random wandering/erratic behavior for humans'
    )

    world = LaunchConfiguration('world')

    # Use ros_gz_sim launch file
    gazebo_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    )

    # Gazebo simulation via ros_gz_sim
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
    )

    # TF bridge for pose info
    pose_tf_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="pose_tf_bridge",
        output="screen",
        arguments=[
            "/world/messy_road/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
        ],
    )

    # Service bridge for set_pose
    set_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="set_pose_bridge",
        output="screen",
        arguments=[
            "/world/messy_road/set_pose@ros_gz_interfaces/srv/SetEntityPose"
        ],
    )

    # Messy road mover node
    messy_road_mover = Node(
        package='ros_humans_ros2',
        executable='messy_road_mover',
        name='messy_road_mover',
        output='screen',
        parameters=[{
            'world': 'messy_road',
            'rate': LaunchConfiguration('rate'),
            'enable_random_behavior': LaunchConfiguration('enable_random_behavior'),
        }]
    )

    # Human pose publisher node
    human_pose_publisher = Node(
        package='ros_humans_ros2',
        executable='publish_human_pose',
        name='human_pose_publisher',
        output='screen',
        parameters=[{
            'frame_id': 'world',
            'human_names': [
                'human_tall_fast',
                'human_avg_medium',
                'human_short_slow',
                'human_child_runner',
                'human_elderly_slow',
                'human_jogger',
                'human_phone_user',
                'human_group_1',
                'human_group_2',
            ],
            'rate': 10.0,
        }]
    )

    return LaunchDescription([
        world_arg,
        rate_arg,
        enable_random_arg,
        gazebo_sim,
        pose_tf_bridge,
        set_pose_bridge,
        messy_road_mover,
        human_pose_publisher,
    ])

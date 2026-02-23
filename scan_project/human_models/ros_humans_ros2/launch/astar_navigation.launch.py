"""
astar_navigation.launch.py
==========================
Launch file for the A* path planner node.

Assumptions
-----------
* The /map topic is already being published (e.g. by map_server).
* The robot publishes /odom (nav_msgs/Odometry).
* A goal is sent on /move_base_simple/goal (geometry_msgs/PoseStamped).

Outputs
-------
* /astar_path  (nav_msgs/Path)    -- for RViz visualisation
* /cmd_vel     (geometry_msgs/Twist) -- drives the robot

Usage
-----
  ros2 launch ros_humans_ros2 astar_navigation.launch.py
  ros2 launch ros_humans_ros2 astar_navigation.launch.py linear_speed:=0.30
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Tuneable parameters exposed at launch time
    args = [
        DeclareLaunchArgument('waypoint_tolerance',       default_value='0.20',
                              description='Distance (m) to consider a waypoint reached'),
        DeclareLaunchArgument('linear_speed',             default_value='0.25',
                              description='Max forward speed (m/s)'),
        DeclareLaunchArgument('angular_speed',            default_value='1.00',
                              description='Max yaw rate (rad/s)'),
        DeclareLaunchArgument('obstacle_inflation_cells', default_value='3',
                              description='Inflate obstacles by N grid cells'),
        DeclareLaunchArgument('waypoint_stride',          default_value='5',
                              description='Keep every Nth A* waypoint'),
    ]

    astar_node = Node(
        package    = 'ros_humans_ros2',
        executable = 'astar_path_planner',
        name       = 'astar_path_planner',
        output     = 'screen',
        emulate_tty= True,
        parameters = [{
            'waypoint_tolerance':       LaunchConfiguration('waypoint_tolerance'),
            'linear_speed':             LaunchConfiguration('linear_speed'),
            'angular_speed':            LaunchConfiguration('angular_speed'),
            'obstacle_inflation_cells': LaunchConfiguration('obstacle_inflation_cells'),
            'waypoint_stride':          LaunchConfiguration('waypoint_stride'),
        }],
    )

    return LaunchDescription(args + [astar_node])

from setuptools import setup
import os
from glob import glob

package_name = "ros_humans_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/demo.launch.py",
            "launch/messy_road.launch.py",
            "launch/iiit_messy_road.launch.py",
            "launch/astar_navigation.launch.py",
            "launch/navigation.launch.py",
            "launch/world_to_map.launch.py",
            "launch/test_scenario.launch.py",
            "launch/social_nav_cases.launch.py",
        ]),
        ("share/" + package_name + "/worlds", glob("worlds/*.world")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="scan-project",
    maintainer_email="user@example.com",
    description="ROS 2 human pose publisher and mover with messy road simulation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "publish_human_pose = ros_humans_ros2.publish_human_pose:main",
            "move_humans = ros_humans_ros2.move_humans:main",
            "messy_road_mover = ros_humans_ros2.messy_road_mover:main",
            "messy_road_pose_publisher = ros_humans_ros2.messy_road_pose_publisher:main",
            "iiit_messy_road_mover = ros_humans_ros2.iiit_messy_road_mover:main",
            "rover_explorer = ros_humans_ros2.rover_explorer:main",
            "pointcloud_mapper = ros_humans_ros2.pointcloud_mapper:main",
            "camera_view_360 = ros_humans_ros2.camera_view_360:main",
            "astar_path_planner = ros_humans_ros2.astar_path_planner:main",
            "world_to_map = ros_humans_ros2.world_to_map:main",
            "map_publisher = ros_humans_ros2.map_publisher:main",
            "human_detector_cv = ros_humans_ros2.human_detector_cv:main",
            "global_planner = ros_humans_ros2.global_planner:main",
            "local_planner = ros_humans_ros2.local_planner:main",
            "human_detector_red = ros_humans_ros2.human_detector_red:main",
            "localization_node = ros_humans_ros2.localization_node:main",
            "live_visualization_node = ros_humans_ros2.live_visualization_node:main",
            "human_case_controller = ros_humans_ros2.human_case_controller:main",
            "social_nav_planner    = ros_humans_ros2.social_nav_planner:main",
        ],
    },
)

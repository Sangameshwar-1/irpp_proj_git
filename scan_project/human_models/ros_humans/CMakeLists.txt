cmake_minimum_required(VERSION 3.0.2)
project(ros_humans)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  gazebo_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/publish_human_pose.py
  scripts/move_humans.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

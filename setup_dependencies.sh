#!/bin/bash

# Update package list
echo "Updating package list..."
sudo apt-get update

# Install ROS 2 Jazzy (assuming dependencies are already set up for ROS 2)
# If not, full installation instructions would be needed, but usually users have ROS 2 if they are asking for this.
# However, to be safe, we will ensure ros-jazzy-desktop is installed.
echo "Installing ROS 2 Jazzy Desktop..."
sudo apt-get install -y ros-jazzy-desktop

# Install Gazebo Sim (Harmonic is the default for Jazzy)
echo "Installing Gazebo Sim..."
sudo apt-get install -y ros-jazzy-ros-gz

# Install build tools
echo "Installing build tools..."
sudo apt-get install -y python3-colcon-common-extensions

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

echo "Installation complete."

#!/bin/bash

# =============================================================================
# ROS2 A* Path Planning Navigation Simulation
# =============================================================================
# This script launches:
#   1. Gazebo simulation with TurtleBot rover (NO moving humans)
#   2. A* Path Planner for goal-based navigation
#   3. Point Cloud Mapper (builds map from LiDAR scans)
#   4. RViz2 for visualization
# =============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  ROS2 A* Path Planning Navigation     ${NC}"
echo -e "${BLUE}========================================${NC}"

# Source ROS 2 environment
echo -e "${YELLOW}Sourcing ROS 2 Jazzy environment...${NC}"
source /opt/ros/jazzy/setup.bash

# Define workspace and project paths
WORKSPACE_DIR=~/ros2_ws
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
PROJECT_SOURCE_DIR=$PROJECT_DIR/scan_project/human_models/ros_humans_ros2
RVIZ_CONFIG=$PROJECT_DIR/scan_project/rviz_config.rviz

# Check if workspace exists, if not create it
if [ ! -d "$WORKSPACE_DIR/src" ]; then
    echo -e "${YELLOW}Creating workspace directory...${NC}"
    mkdir -p $WORKSPACE_DIR/src
fi

# Copy project to workspace
TARGET_DIR=$WORKSPACE_DIR/src/ros_humans_ros2
echo -e "${YELLOW}Updating project in workspace...${NC}"
rm -rf $TARGET_DIR
cp -r $PROJECT_SOURCE_DIR $WORKSPACE_DIR/src/

# Build the workspace
echo -e "${YELLOW}Building the workspace...${NC}"
cd $WORKSPACE_DIR
colcon build --symlink-install --packages-select ros_humans_ros2

# Source the workspace
echo -e "${GREEN}Sourcing workspace...${NC}"
source install/setup.bash

# Function to cleanup on exit
cleanup() {
    echo -e "\n${YELLOW}Shutting down simulation...${NC}"
    pkill -f "ros2" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ruby" 2>/dev/null || true
    echo -e "${GREEN}Simulation stopped.${NC}"
}

# Set trap to cleanup on script exit
trap cleanup EXIT INT TERM

echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}Starting A* Navigation Simulation:${NC}"
echo -e "  - Gazebo with large_messy_room world"
echo -e "  - TurtleBot3 rover"
echo -e "  - ${CYAN}A* Path Planner (goal-based navigation)${NC}"
echo -e "  - Point Cloud Mapper"
echo -e "  - ${RED}Moving humans DISABLED${NC}"
echo -e "${BLUE}========================================${NC}"

# Launch the A* navigation simulation
echo -e "${YELLOW}Launching A* Navigation simulation...${NC}"
ros2 launch ros_humans_ros2 astar_navigation.launch.py &
GAZEBO_PID=$!

# Wait for Gazebo to start
echo -e "${YELLOW}Waiting for simulation to initialize (15 seconds)...${NC}"
sleep 15

# Check if simulation is running
if ! ps -p $GAZEBO_PID > /dev/null 2>&1; then
    echo -e "${RED}Error: Gazebo failed to start!${NC}"
    exit 1
fi

# Launch RViz2 for visualization
echo -e "${YELLOW}Launching RViz2...${NC}"
if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d $RVIZ_CONFIG &
else
    rviz2 &
fi

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Simulation Running!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${CYAN}A* Path Planner Usage:${NC}"
echo -e "  Send a goal pose to make the rover navigate:"
echo ""
echo -e "${YELLOW}  # Navigate to position (5, 5):${NC}"
echo -e "  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\"
echo -e "    '{header: {frame_id: \"map\"}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}}}'"
echo ""
echo -e "${YELLOW}  # Navigate to position (-5, -5):${NC}"
echo -e "  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\"
echo -e "    '{header: {frame_id: \"map\"}, pose: {position: {x: -5.0, y: -5.0, z: 0.0}}}'"
echo ""
echo -e "${CYAN}Available Topics:${NC}"
echo -e "  /goal_pose         - Send navigation goals"
echo -e "  /planned_path      - View computed A* path"
echo -e "  /path_markers      - Path visualization markers"
echo -e "  /local_costmap     - Occupancy grid from LiDAR"
echo ""
echo -e "${GREEN}Press Ctrl+C to stop the simulation${NC}"
echo ""

# Keep the script running
wait

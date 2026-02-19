#!/bin/bash

# =============================================================================
# ROS2 Gazebo Simulation with Point Cloud Mapping
# =============================================================================
# This script launches:
#   1. Gazebo simulation with TurtleBot rover and moving humans
#   2. Point Cloud Mapper (builds map from LiDAR scans)
#   3. Occupancy Grid Map generation
#   4. RViz2 for visualization
# =============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  ROS2 Gazebo + Point Cloud Simulation ${NC}"
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

# Copy project to workspace if it's not already linked or there
TARGET_DIR=$WORKSPACE_DIR/src/ros_humans_ros2
if [ ! -d "$TARGET_DIR" ]; then
    echo -e "${YELLOW}Copying project to workspace...${NC}"
    cp -r $PROJECT_SOURCE_DIR $WORKSPACE_DIR/src/
else
    echo -e "${GREEN}Project already in workspace.${NC}"
fi

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
    # Kill all ROS2 related processes
    pkill -f "ros2" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ruby" 2>/dev/null || true
    echo -e "${GREEN}Simulation stopped.${NC}"
}

# Set trap to cleanup on script exit
trap cleanup EXIT INT TERM

echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}Starting Simulation Components:${NC}"
echo -e "  - Gazebo with large_messy_room world"
echo -e "  - TurtleBot3 rover (autonomous exploration)"
echo -e "  - 5 moving humans"
echo -e "  - Point Cloud Mapper"
echo -e "  - Occupancy Grid Map Generator"
echo -e "  - 360° Camera View"
echo -e "${BLUE}========================================${NC}"

# Launch the main simulation (Gazebo + all nodes)
echo -e "${YELLOW}Launching Gazebo simulation...${NC}"
ros2 launch ros_humans_ros2 demo.launch.py &
GAZEBO_PID=$!

# Wait for Gazebo to start and topics to be available
echo -e "${YELLOW}Waiting for simulation to initialize (15 seconds)...${NC}"
sleep 15

# Check if simulation is running
if ! ps -p $GAZEBO_PID > /dev/null 2>&1; then
    echo -e "${RED}Error: Gazebo failed to start!${NC}"
    exit 1
fi

# Launch RViz2 for visualization
echo -e "${YELLOW}Launching RViz2 for point cloud and map visualization...${NC}"
if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d $RVIZ_CONFIG --ros-args -p use_sim_time:=true &
    RVIZ_PID=$!
    echo -e "${GREEN}RViz2 started with custom config${NC}"
else
    echo -e "${YELLOW}RViz config not found, launching with default settings...${NC}"
    rviz2 --ros-args -p use_sim_time:=true &
    RVIZ_PID=$!
fi

echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}Simulation is now running!${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "Available ROS2 Topics:"
echo -e "  ${GREEN}/scan${NC}            - LiDAR scan data"
echo -e "  ${GREEN}/scan_pointcloud${NC} - Current scan as point cloud"
echo -e "  ${GREEN}/map_pointcloud${NC}  - Accumulated point cloud map"
echo -e "  ${GREEN}/map${NC}             - Occupancy Grid map"
echo -e "  ${GREEN}/camera/panorama${NC} - 360° camera view"
echo ""
echo -e "In RViz2 you can see:"
echo -e "  - Gray cells    = Free space (explored)"
echo -e "  - Black cells   = Occupied (walls/obstacles)"
echo -e "  - White cells   = Unexplored areas"
echo -e "  - Red points    = LiDAR scan"
echo -e "  - Colored cloud = Accumulated map"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop the simulation${NC}"
echo -e "${BLUE}========================================${NC}"

# Wait for user to stop
wait $GAZEBO_PID

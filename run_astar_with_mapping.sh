#!/bin/bash
# ==============================================================================
# A* Path Planning with Point Cloud Mapping & World Visualization
# ==============================================================================
# This script provides a complete navigation stack:
#   1. Converts Gazebo .world file to ROS2 occupancy grid map
#   2. Launches Gazebo simulation with TurtleBot rover
#   3. Starts A* path planner for goal-based navigation
#   4. Activates point cloud mapper for real-time mapping
#   5. Opens RViz for visualization
#
# Usage:
#   ./run_astar_with_mapping.sh                           # Use default world
#   ./run_astar_with_mapping.sh messy_road.world          # Specify world file
#   ./run_astar_with_mapping.sh custom.world 5.0 5.0      # Custom goal position
# ==============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# Default values
WORLD_FILE="${1:-large_messy_room.world}"
GOAL_X="${2:-5.0}"
GOAL_Y="${3:-5.0}"
RESOLUTION="0.05"
OUTPUT_DIR="$HOME/ros2_maps"

echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}A* Navigation with Point Cloud Mapping${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}Configuration:${NC}"
echo -e "  World File      : ${WORLD_FILE}"
echo -e "  Default Goal    : (${GOAL_X}, ${GOAL_Y})"
echo -e "  Map Resolution  : ${RESOLUTION} m/pixel"
echo -e "  Output Dir      : ${OUTPUT_DIR}"
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS2 not sourced!${NC}"
    echo -e "${YELLOW}Please run: source /opt/ros/jazzy/setup.bash${NC}"
    exit 1
fi

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

if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Build failed!${NC}"
    exit 1
fi

# Source the workspace
echo -e "${GREEN}Sourcing workspace...${NC}"
source install/setup.bash

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Get package directory
PKG_SHARE=$(ros2 pkg prefix ros_humans_ros2)/share/ros_humans_ros2

# Determine world file path
if [[ "$WORLD_FILE" = /* ]]; then
    # Absolute path
    WORLD_PATH="$WORLD_FILE"
else
    # Relative to package worlds directory
    WORLD_PATH="$PKG_SHARE/worlds/$WORLD_FILE"
fi

# Check if world file exists
if [ ! -f "$WORLD_PATH" ]; then
    echo -e "${RED}Error: World file not found: $WORLD_PATH${NC}"
    echo ""
    echo -e "${YELLOW}Available worlds in package:${NC}"
    ls -1 "$PKG_SHARE/worlds/"*.world 2>/dev/null || echo "  No worlds found"
    exit 1
fi

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${CYAN}Step 1: Converting World to Map${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Get base name without extension
WORLD_BASENAME=$(basename "$WORLD_FILE" .world)
OUTPUT_BASE="$OUTPUT_DIR/${WORLD_BASENAME}_map"

# Run the converter
python3 -c "
import sys
sys.path.insert(0, '$PKG_SHARE/../../../lib/python3.12/site-packages')
from ros_humans_ros2.world_to_map import main
sys.argv = [
    'world_to_map.py',
    '$WORLD_PATH',
    '-o', '$OUTPUT_BASE',
    '-r', '$RESOLUTION',
    '--inflate', '0.1',
    '--padding', '1.0'
]
main()
"

if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Failed to convert world file!${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}✓ Map conversion complete!${NC}"
echo -e "  PGM file  : ${OUTPUT_BASE}.pgm"
echo -e "  YAML file : ${OUTPUT_BASE}.yaml"
echo ""

# Check if map files were created
if [ ! -f "${OUTPUT_BASE}.pgm" ] || [ ! -f "${OUTPUT_BASE}.yaml" ]; then
    echo -e "${RED}Error: Map files not generated!${NC}"
    exit 1
fi

# Function to cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}Shutting down all processes...${NC}"
    kill $MAP_PUB_PID 2>/dev/null || true
    kill $GAZEBO_PID 2>/dev/null || true
    kill $RVIZ_PID 2>/dev/null || true
    pkill -f "ros2" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ruby" 2>/dev/null || true
    wait 2>/dev/null || true
    echo -e "${GREEN}Cleanup complete!${NC}"
}

# Set trap for cleanup
trap cleanup EXIT INT TERM

echo -e "${BLUE}========================================${NC}"
echo -e "${CYAN}Step 2: Starting Map Publisher${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Start map publisher in background
ros2 run ros_humans_ros2 map_publisher \
    --ros-args \
    -p yaml_file:="${OUTPUT_BASE}.yaml" \
    -p publish_rate:=1.0 \
    -p frame_id:=map \
    -p use_sim_time:=true &

MAP_PUB_PID=$!

# Wait a moment for publisher to start
sleep 2

if ! ps -p $MAP_PUB_PID > /dev/null 2>&1; then
    echo -e "${RED}Error: Map publisher failed to start!${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Map publisher started (PID: $MAP_PUB_PID)${NC}"
echo ""

echo -e "${BLUE}========================================${NC}"
echo -e "${CYAN}Step 3: Launching Gazebo & Navigation${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}Starting:${NC}"
echo -e "  - Gazebo simulation with $WORLD_BASENAME world"
echo -e "  - TurtleBot3 rover"
echo -e "  - ${CYAN}A* Path Planner${NC} (goal: $GOAL_X, $GOAL_Y)"
echo -e "  - ${CYAN}Point Cloud Mapper${NC} (real-time mapping)"
echo ""

# Launch the A* navigation simulation
cd $WORKSPACE_DIR
ros2 launch ros_humans_ros2 astar_navigation.launch.py \
    world:="$WORLD_PATH" \
    default_goal_x:=$GOAL_X \
    default_goal_y:=$GOAL_Y &

GAZEBO_PID=$!

# Wait for Gazebo to start
echo -e "${YELLOW}Waiting for simulation to initialize (15 seconds)...${NC}"
sleep 15

# Check if simulation is running
if ! ps -p $GAZEBO_PID > /dev/null 2>&1; then
    echo -e "${RED}Error: Gazebo failed to start!${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Simulation started (PID: $GAZEBO_PID)${NC}"
echo ""

echo -e "${BLUE}========================================${NC}"
echo -e "${CYAN}Step 4: Launching RViz${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Launch RViz2 for visualization
# OGRE_RTT_MODE=Copy fixes RViz2 GLSL "active samplers with different type" error
export OGRE_RTT_MODE=Copy
if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG" &
    RVIZ_PID=$!
    echo -e "${GREEN}✓ RViz launched with config (PID: $RVIZ_PID)${NC}"
else
    rviz2 &
    RVIZ_PID=$!
    echo -e "${YELLOW}Warning: RViz config not found, using default${NC}"
    echo -e "${GREEN}✓ RViz launched (PID: $RVIZ_PID)${NC}"
fi

sleep 2

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}🚀 Complete Navigation Stack Active! 🚀${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

echo -e "${MAGENTA}═══ SYSTEM STATUS ═══${NC}"
echo -e "  ${GREEN}✓${NC} Static Map Publisher    : /map (from world file)"
echo -e "  ${GREEN}✓${NC} Gazebo Simulation       : Running"
echo -e "  ${GREEN}✓${NC} A* Path Planner         : Active, auto-navigating to ($GOAL_X, $GOAL_Y)"
echo -e "  ${GREEN}✓${NC} Point Cloud Mapper      : Building real-time map"
echo -e "  ${GREEN}✓${NC} RViz Visualization      : Displaying all data"
echo ""

echo -e "${MAGENTA}═══ RVIZ VISUALIZATION SETUP ═══${NC}"
echo -e "${YELLOW}Configure RViz to display:${NC}"
echo -e "  1. ${CYAN}/map${NC} -> Map display (static map from world file)"
echo -e "  2. ${CYAN}/local_costmap${NC} -> Map display (dynamic obstacles from A*)"
echo -e "  3. ${CYAN}/scan_pointcloud${NC} -> PointCloud2 (current LiDAR scan)"
echo -e "  4. ${CYAN}/map_pointcloud${NC} -> PointCloud2 (accumulated map)"
echo -e "  5. ${CYAN}/planned_path${NC} -> Path display (A* computed path)"
echo -e "  6. ${CYAN}/path_markers${NC} -> MarkerArray (start/goal/path)"
echo -e "  7. ${CYAN}TF${NC} -> Show all transforms (robot frames)"
echo -e "  ${YELLOW}Fixed Frame:${NC} ${CYAN}map${NC}"
echo ""

echo -e "${MAGENTA}═══ AVAILABLE TOPICS ═══${NC}"
echo -e "  ${CYAN}/map${NC}                  - Static occupancy grid (from world)"
echo -e "  ${CYAN}/local_costmap${NC}        - Dynamic costmap (A* planner)"
echo -e "  ${CYAN}/scan${NC}                 - LiDAR scan data"
echo -e "  ${CYAN}/scan_pointcloud${NC}      - Current scan as point cloud"
echo -e "  ${CYAN}/map_pointcloud${NC}       - Accumulated map point cloud"
echo -e "  ${CYAN}/goal_pose${NC}            - Send custom navigation goals"
echo -e "  ${CYAN}/planned_path${NC}         - A* computed path"
echo -e "  ${CYAN}/path_markers${NC}         - Path visualization markers"
echo -e "  ${CYAN}/cmd_vel${NC}              - Robot velocity commands"
echo -e "  ${CYAN}/odom${NC}                 - Robot odometry"
echo ""

echo -e "${MAGENTA}═══ SEND CUSTOM GOALS ═══${NC}"
echo -e "${YELLOW}Navigate to different positions:${NC}"
echo ""
echo -e "${CYAN}# Goal at (5, 5):${NC}"
echo -e "  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\"
echo -e "    '{header: {frame_id: \"map\"}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}}}'"
echo ""
echo -e "${CYAN}# Goal at (-5, -5):${NC}"
echo -e "  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\"
echo -e "    '{header: {frame_id: \"map\"}, pose: {position: {x: -5.0, y: -5.0, z: 0.0}}}'"
echo ""
echo -e "${CYAN}# Goal at (0, 8):${NC}"
echo -e "  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\"
echo -e "    '{header: {frame_id: \"map\"}, pose: {position: {x: 0.0, y: 8.0, z: 0.0}}}'"
echo ""

echo -e "${MAGENTA}═══ MONITORING COMMANDS ═══${NC}"
echo -e "${CYAN}# View map info:${NC}"
echo -e "  ros2 topic echo /map --once"
echo ""
echo -e "${CYAN}# View current path:${NC}"
echo -e "  ros2 topic echo /planned_path --once"
echo ""
echo -e "${CYAN}# List all topics:${NC}"
echo -e "  ros2 topic list"
echo ""
echo -e "${CYAN}# Monitor robot position:${NC}"
echo -e "  ros2 topic echo /odom"
echo ""
echo -e "${CYAN}# View A* planner logs:${NC}"
echo -e "  ros2 node info /astar_path_planner"
echo ""
echo -e "${CYAN}# View point cloud mapper logs:${NC}"
echo -e "  ros2 node info /pointcloud_mapper"
echo ""

echo -e "${MAGENTA}═══ FEATURES ═══${NC}"
echo -e "  ${GREEN}➤${NC} ${YELLOW}A* Path Planning:${NC} Optimal path finding with obstacle avoidance"
echo -e "  ${GREEN}➤${NC} ${YELLOW}Point Cloud Mapping:${NC} Real-time 3D map building from LiDAR"
echo -e "  ${GREEN}➤${NC} ${YELLOW}Dynamic Replanning:${NC} Automatic path updates when obstacles detected"
echo -e "  ${GREEN}➤${NC} ${YELLOW}Occupancy Grid:${NC} Both static (world) and dynamic (sensor-based)"
echo -e "  ${GREEN}➤${NC} ${YELLOW}Goal-Based Navigation:${NC} Send goal poses via ROS2 topics"
echo -e "  ${GREEN}➤${NC} ${YELLOW}Visual Feedback:${NC} Path, markers, and costmap in RViz"
echo ""

echo -e "${YELLOW}Press Ctrl+C to stop all processes${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Keep the script running and wait for user to stop
wait $RVIZ_PID

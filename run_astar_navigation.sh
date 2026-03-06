#!/bin/bash

# =============================================================================
# ROS2 A* Path Planning Navigation Simulation
# =============================================================================
# Flow:
#   Step 0 – Generate occupancy-grid map from the Gazebo .world file
#   Step 1 – Build & source the ROS2 workspace
#   Step 2 – Launch Gazebo + map_publisher + A* planner + point-cloud mapper
#   Step 3 – Launch RViz with all layers stacked on the generated map
# =============================================================================

set -e  # Exit on error

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  ROS2 A* Path Planning Navigation     ${NC}"
echo -e "${BLUE}========================================${NC}"

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_SOURCE_DIR="$SCRIPT_DIR/scan_project/human_models/ros_humans_ros2"
RVIZ_CONFIG="$SCRIPT_DIR/scan_project/rviz_config.rviz"
WORKSPACE_DIR=~/ros2_ws
WORLD_NAME="large_messy_room"
MAP_DIR="$HOME/ros2_maps"
MAP_YAML="$MAP_DIR/${WORLD_NAME}_map.yaml"

# ── Source ROS 2 ──────────────────────────────────────────────────────────────
echo -e "${YELLOW}Sourcing ROS 2 Jazzy...${NC}"
source /opt/ros/jazzy/setup.bash

# ── Prepare workspace ─────────────────────────────────────────────────────────
mkdir -p "$WORKSPACE_DIR/src"
TARGET_DIR="$WORKSPACE_DIR/src/ros_humans_ros2"
echo -e "${YELLOW}Updating project in workspace...${NC}"
rm -rf "$TARGET_DIR"
cp -r "$PROJECT_SOURCE_DIR" "$WORKSPACE_DIR/src/"

echo -e "${YELLOW}Building workspace...${NC}"
cd "$WORKSPACE_DIR"
colcon build --symlink-install --packages-select ros_humans_ros2

echo -e "${GREEN}Sourcing workspace...${NC}"
source "$WORKSPACE_DIR/install/setup.bash"

# ── Step 0: Generate world map ────────────────────────────────────────────────
echo ""
echo -e "${BLUE}────────────────────────────────────────${NC}"
echo -e "${CYAN}Step 0: Generating map from ${WORLD_NAME}.world${NC}"
echo -e "${BLUE}────────────────────────────────────────${NC}"

mkdir -p "$MAP_DIR"

PKG_SHARE="$(ros2 pkg prefix ros_humans_ros2)/share/ros_humans_ros2"
WORLD_FILE="$PKG_SHARE/worlds/${WORLD_NAME}.world"

if [ ! -f "$WORLD_FILE" ]; then
    echo -e "${RED}Error: world file not found: $WORLD_FILE${NC}"
    exit 1
fi

echo -e "${YELLOW}  World : $WORLD_FILE${NC}"
echo -e "${YELLOW}  Output: $MAP_YAML${NC}"

ros2 run ros_humans_ros2 world_to_map \
    "$WORLD_FILE" \
    -o "$MAP_DIR/${WORLD_NAME}_map" \
    -r 0.05 \
    --inflate 0.10 \
    --padding 1.5

if [ ! -f "$MAP_YAML" ]; then
    echo -e "${RED}Error: map generation failed – $MAP_YAML not found${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Map ready: $MAP_YAML${NC}"

# ── Cleanup trap ──────────────────────────────────────────────────────────────
cleanup() {
    echo -e "\n${YELLOW}Shutting down simulation...${NC}"
    pkill -f "ros2 launch" 2>/dev/null || true
    pkill -f "rviz2"       2>/dev/null || true
    pkill -f "gz sim"      2>/dev/null || true
    pkill -f "ruby"        2>/dev/null || true
    echo -e "${GREEN}Simulation stopped.${NC}"
}
trap cleanup EXIT INT TERM

# ── Step 1: Launch simulation ─────────────────────────────────────────────────
echo ""
echo -e "${BLUE}────────────────────────────────────────${NC}"
echo -e "${CYAN}Step 1: Launching simulation${NC}"
echo -e "${BLUE}────────────────────────────────────────${NC}"
echo -e "  World map   : ${MAP_YAML}"
echo -e "  A* goal     : (5.0, 5.0) [auto-start]"
echo -e "  Point cloud : /pointcloud_map overlay"

ros2 launch ros_humans_ros2 astar_navigation.launch.py \
    map_yaml:="$MAP_YAML" \
    default_goal_x:=5.0 \
    default_goal_y:=5.0 &
LAUNCH_PID=$!

echo -e "${YELLOW}Waiting 15 s for Gazebo to initialise...${NC}"
sleep 15

if ! ps -p $LAUNCH_PID > /dev/null 2>&1; then
    echo -e "${RED}Error: launch failed!${NC}"
    exit 1
fi

# ── Step 2: Launch RViz ───────────────────────────────────────────────────────
echo -e "${YELLOW}Launching RViz2...${NC}"
# OGRE_RTT_MODE=Copy fixes RViz2 GLSL "active samplers with different type" error
export OGRE_RTT_MODE=Copy
if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG" &
else
    rviz2 &
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Simulation Running!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${CYAN}RViz layers (bottom → top):${NC}"
echo -e "  1. ${BLUE}/map${NC}            – static world map (from ${WORLD_NAME}.world)"
echo -e "  2. ${CYAN}/pointcloud_map${NC} – LiDAR sensor map (built as rover drives)"
echo -e "  3. ${YELLOW}/local_costmap${NC}  – A* inflated obstacle grid"
echo -e "  4. ${GREEN}/planned_path${NC}   – GREEN shortest path (followed by rover)"
echo -e "     ${BLUE}/planned_path_2${NC} – BLUE  2nd shortest path"
echo -e "     ${RED}/planned_path_3${NC} – RED   3rd shortest path"
echo -e "  5. /path_markers   – colored paths + distance labels + start/goal spheres"
echo ""
echo -e "${CYAN}Send a custom goal:${NC}"
echo -e "  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\"
echo -e "    '{header: {frame_id: \"map\"}, pose: {position: {x: 5.0, y: 5.0}}}'"
echo ""
echo -e "${GREEN}Press Ctrl+C to stop${NC}"
echo ""

wait

#!/bin/bash
# =============================================================================
# ROS2 Navigation — Refactored Architecture
# =============================================================================
# Global Planner (A* on weighted grid)  +  Local Planner (WAIT / REROUTE)
#
# Flow:
#   Step 0 – Generate occupancy map from the Gazebo .world file
#   Step 1 – Build & source the ROS2 workspace
#   Step 2 – Launch everything via navigation.launch.py
#   Step 3 – Open RViz
# =============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  ROS2 Navigation (Refactored)          ${NC}"
echo -e "${BLUE}  Global + Local Planner Architecture   ${NC}"
echo -e "${BLUE}========================================${NC}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_SOURCE_DIR="$SCRIPT_DIR/scan_project/human_models/ros_humans_ros2"
RVIZ_CONFIG="$SCRIPT_DIR/scan_project/rviz_config.rviz"
WORKSPACE_DIR=~/ros2_ws
WORLD_NAME="large_messy_room"
MAP_DIR="$HOME/ros2_maps"
MAP_YAML="$MAP_DIR/${WORLD_NAME}_map.yaml"

# Default goal (can be overridden with arguments)
GOAL_X="${1:-5.0}"
GOAL_Y="${2:-5.0}"

# ── Source ROS 2 ──────────────────────────────────────────────────────
echo -e "${YELLOW}Sourcing ROS 2 Jazzy...${NC}"
source /opt/ros/jazzy/setup.bash

# ── Prepare workspace ────────────────────────────────────────────────
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

# ── Step 0: Generate world map ───────────────────────────────────────
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

ros2 run ros_humans_ros2 world_to_map \
    "$WORLD_FILE" \
    -o "$MAP_DIR/${WORLD_NAME}_map" \
    -r 0.05 \
    --inflate 0.10 \
    --padding 1.5

if [ ! -f "$MAP_YAML" ]; then
    echo -e "${RED}Error: map generation failed!${NC}"
    exit 1
fi
echo -e "${GREEN}Map generated: $MAP_YAML${NC}"

# ── Step 1: Launch navigation stack ──────────────────────────────────
echo ""
echo -e "${BLUE}────────────────────────────────────────${NC}"
echo -e "${CYAN}Step 1: Launching navigation stack${NC}"
echo -e "${BLUE}────────────────────────────────────────${NC}"
echo -e "${YELLOW}  Goal: ($GOAL_X, $GOAL_Y)${NC}"
echo -e "${YELLOW}  Architecture:${NC}"
echo -e "${YELLOW}    • Localization Node  → /robot_pose${NC}"
echo -e "${YELLOW}    • Global Planner     → A* on weighted grid${NC}"
echo -e "${YELLOW}    • Local Planner      → WAIT / REROUTE${NC}"
echo -e "${YELLOW}    • Human Detector     → red shapes only${NC}"

ros2 launch ros_humans_ros2 navigation.launch.py \
    map_yaml:="$MAP_YAML" \
    default_goal_x:="$GOAL_X" \
    default_goal_y:="$GOAL_Y" &
LAUNCH_PID=$!

echo -e "${GREEN}Navigation launched (PID: $LAUNCH_PID)${NC}"

# ── Step 2: Launch RViz ──────────────────────────────────────────────
echo ""
echo -e "${BLUE}────────────────────────────────────────${NC}"
echo -e "${CYAN}Step 2: Opening RViz${NC}"
echo -e "${BLUE}────────────────────────────────────────${NC}"

sleep 3
if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG" &
else
    rviz2 &
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  System running!                       ${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${CYAN}  Set a new goal:${NC}"
echo -e "${CYAN}    ros2 topic pub /goal_pose geometry_msgs/PoseStamped \\${NC}"
echo -e "${CYAN}      '{pose: {position: {x: 5.0, y: 5.0}}}'${NC}"
echo ""
echo -e "${CYAN}  Topics to watch:${NC}"
echo -e "${CYAN}    /robot_pose        – localised pose${NC}"
echo -e "${CYAN}    /global_path       – planned path${NC}"
echo -e "${CYAN}    /weighted_grid_viz – edge weights visualisation${NC}"
echo -e "${CYAN}    /detected_humans   – red-shape detections${NC}"
echo ""

wait $LAUNCH_PID

#!/bin/bash
# =============================================================================
# ROS2 Test Scenario Runner
# =============================================================================
# Launches one of four mini Gazebo test environments to observe path planning
# behaviour with stationary humans.
#
# Usage:
#   ./run_test_scenario.sh corridor_blocked   # Straight corridor, human blocking
#   ./run_test_scenario.sh l_corridor         # L-shaped corridor, human at corner
#   ./run_test_scenario.sh open_room          # Open room, human on diagonal
#   ./run_test_scenario.sh doorway            # Two rooms + doorway, human near door
#
# Each scenario has a pre-generated map and spawns the robot and goal
# automatically.  The A* planner will compute the initial path, and the
# local planner will reroute when it detects the human.
# =============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_SOURCE_DIR="$SCRIPT_DIR/scan_project/human_models/ros_humans_ros2"
RVIZ_CONFIG="$SCRIPT_DIR/scan_project/rviz_config.rviz"
WORKSPACE_DIR=~/ros2_ws
MAP_DIR="$HOME/ros2_maps"

SCENARIO="${1:-corridor_blocked}"

# ── Scenario lookup table ────────────────────────────────────────────
case "$SCENARIO" in
    corridor_blocked)
        WORLD_NAME="test_corridor_blocked"
        SPAWN_X="-4.0"
        SPAWN_Y="0.0"
        SPAWN_YAW="0.0"
        GOAL_X="4.0"
        GOAL_Y="0.0"
        ROOM_MIN_X="-5.5"
        ROOM_MAX_X="5.5"
        ROOM_MIN_Y="-2.0"
        ROOM_MAX_Y="2.0"
        MAP_SIZE="14.0"
        DESC="Straight 10m corridor — human blocking the centre"
        ;;
    l_corridor)
        WORLD_NAME="test_l_corridor"
        SPAWN_X="-3.0"
        SPAWN_Y="-2.75"
        SPAWN_YAW="0.0"
        GOAL_X="2.25"
        GOAL_Y="4.0"
        ROOM_MIN_X="-5.0"
        ROOM_MAX_X="4.0"
        ROOM_MIN_Y="-4.0"
        ROOM_MAX_Y="5.0"
        MAP_SIZE="12.0"
        DESC="L-shaped corridor — human at the inner corner"
        ;;
    open_room)
        WORLD_NAME="test_open_room"
        SPAWN_X="-3.0"
        SPAWN_Y="-3.0"
        SPAWN_YAW="0.7854"
        GOAL_X="3.0"
        GOAL_Y="3.0"
        ROOM_MIN_X="-4.5"
        ROOM_MAX_X="4.5"
        ROOM_MIN_Y="-4.5"
        ROOM_MAX_Y="4.5"
        MAP_SIZE="12.0"
        DESC="Open 8m×8m room — human on diagonal path + small obstacle"
        ;;
    doorway)
        WORLD_NAME="test_doorway"
        SPAWN_X="-3.0"
        SPAWN_Y="0.0"
        SPAWN_YAW="0.0"
        GOAL_X="3.0"
        GOAL_Y="0.0"
        ROOM_MIN_X="-5.5"
        ROOM_MAX_X="5.5"
        ROOM_MIN_Y="-3.0"
        ROOM_MAX_Y="3.0"
        MAP_SIZE="14.0"
        DESC="Two rooms connected by doorway — human near the doorway"
        ;;
    *)
        echo -e "${RED}Unknown scenario: $SCENARIO${NC}"
        echo ""
        echo "Available scenarios:"
        echo "  corridor_blocked  — Straight 10m corridor, human blocking centre"
        echo "  l_corridor        — L-shaped corridor, human at corner"
        echo "  open_room         — Open 8m×8m room, human on diagonal"
        echo "  doorway           — Two rooms + doorway, human near door"
        exit 1
        ;;
esac

MAP_YAML="$MAP_DIR/${WORLD_NAME}.yaml"

echo -e "${BLUE}═══════════════════════════════════════════${NC}"
echo -e "${BLUE}  Test Scenario: ${CYAN}${SCENARIO}${NC}"
echo -e "${BLUE}  ${DESC}${NC}"
echo -e "${BLUE}═══════════════════════════════════════════${NC}"
echo -e "  World:  ${YELLOW}${WORLD_NAME}${NC}"
echo -e "  Spawn:  (${SPAWN_X}, ${SPAWN_Y}) yaw=${SPAWN_YAW}"
echo -e "  Goal:   (${GOAL_X}, ${GOAL_Y})"
echo -e "  Map:    ${MAP_YAML}"
echo ""

# ── Source ROS 2 ──────────────────────────────────────────────────────
echo -e "${YELLOW}Sourcing ROS 2 Jazzy...${NC}"
source /opt/ros/jazzy/setup.bash

# ── Build workspace ──────────────────────────────────────────────────
mkdir -p "$WORKSPACE_DIR/src"
TARGET_DIR="$WORKSPACE_DIR/src/ros_humans_ros2"
echo -e "${YELLOW}Updating project in workspace...${NC}"
rm -rf "$TARGET_DIR"
cp -r "$PROJECT_SOURCE_DIR" "$WORKSPACE_DIR/src/"

echo -e "${YELLOW}Building workspace...${NC}"
cd "$WORKSPACE_DIR"
colcon build --symlink-install --packages-select ros_humans_ros2

# Force-sync Python source files that colcon copies (not symlinks) to build dir.
# When --symlink-install creates symlinks the cp will detect same-inode and skip.
PY_SRC="$WORKSPACE_DIR/src/ros_humans_ros2/ros_humans_ros2"
PY_BUILD="$WORKSPACE_DIR/build/ros_humans_ros2/ros_humans_ros2"
if [ -d "$PY_BUILD" ]; then
    for f in "$PY_SRC"/*.py; do
        src_inode=$(stat -c '%i' "$f" 2>/dev/null)
        dst="$PY_BUILD/$(basename "$f")"
        dst_inode=$(stat -c '%i' "$dst" 2>/dev/null)
        if [ "$src_inode" != "$dst_inode" ]; then
            cp "$f" "$dst"
        fi
    done
fi

echo -e "${GREEN}Sourcing workspace...${NC}"
source "$WORKSPACE_DIR/install/setup.bash"

# ── Resolve world file path ─────────────────────────────────────────
PKG_SHARE="$(ros2 pkg prefix ros_humans_ros2)/share/ros_humans_ros2"
WORLD_FILE="$PKG_SHARE/worlds/${WORLD_NAME}.world"

if [ ! -f "$WORLD_FILE" ]; then
    echo -e "${RED}Error: world file not found: $WORLD_FILE${NC}"
    exit 1
fi

# ── Check map exists ────────────────────────────────────────────────
if [ ! -f "$MAP_YAML" ]; then
    echo -e "${YELLOW}Map not found at $MAP_YAML — generating...${NC}"
    mkdir -p "$MAP_DIR"
    ros2 run ros_humans_ros2 world_to_map \
        "$WORLD_FILE" \
        -o "$MAP_DIR/${WORLD_NAME}" \
        -r 0.05 \
        --inflate 0.10 \
        --padding 1.5
fi

if [ ! -f "$MAP_YAML" ]; then
    echo -e "${RED}Error: map generation failed!${NC}"
    exit 1
fi

# ── Launch the test scenario ────────────────────────────────────────
echo ""
echo -e "${BLUE}────────────────────────────────────────${NC}"
echo -e "${CYAN}Launching scenario: $SCENARIO${NC}"
echo -e "${BLUE}────────────────────────────────────────${NC}"

ros2 launch ros_humans_ros2 test_scenario.launch.py \
    world_name:="$WORLD_NAME" \
    world_file:="$WORLD_FILE" \
    map_yaml:="$MAP_YAML" \
    spawn_x:="$SPAWN_X" \
    spawn_y:="$SPAWN_Y" \
    spawn_yaw:="$SPAWN_YAW" \
    goal_x:="$GOAL_X" \
    goal_y:="$GOAL_Y" \
    room_min_x:="$ROOM_MIN_X" \
    room_max_x:="$ROOM_MAX_X" \
    room_min_y:="$ROOM_MIN_Y" \
    room_max_y:="$ROOM_MAX_Y" \
    map_size:="$MAP_SIZE" &
LAUNCH_PID=$!

echo -e "${GREEN}Navigation launched (PID: $LAUNCH_PID)${NC}"

# ── Launch RViz ──────────────────────────────────────────────────────
echo ""
echo -e "${BLUE}────────────────────────────────────────${NC}"
echo -e "${CYAN}Opening RViz...${NC}"
echo -e "${BLUE}────────────────────────────────────────${NC}"

sleep 3
export OGRE_RTT_MODE=Copy
if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG" &
else
    rviz2 &
fi

echo ""
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}  Scenario running!  ${CYAN}${SCENARIO}${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
echo ""
echo -e "${CYAN}  Useful topics:${NC}"
echo -e "    /robot_pose        – localised pose"
echo -e "    /global_path       – A* planned path"
echo -e "    /detected_humans   – red-shape detections"
echo -e "    /weight_zones      – cost zones from local planner"
echo -e "    /replan_request    – reroute triggers"
echo ""
echo -e "${CYAN}  Set a new goal:${NC}"
echo -e "    ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \\"
echo -e "      '{header: {frame_id: map}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}}}}'"
echo ""

wait $LAUNCH_PID

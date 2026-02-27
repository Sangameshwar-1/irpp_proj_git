#!/bin/bash
# ==============================================================================
# World to Map Converter & Visualizer
# ==============================================================================
# This script converts Gazebo .world files to ROS2 occupancy grid maps
# and displays them in RViz.
#
# Usage:
#   ./convert_world_to_map.sh                    # Use default world
#   ./convert_world_to_map.sh messy_road.world   # Specify world file
#   ./convert_world_to_map.sh custom.world 0.03  # Custom resolution
# ==============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
WORLD_FILE="${1:-large_messy_room.world}"
RESOLUTION="${2:-0.05}"
OUTPUT_DIR="${3:-$HOME/ros2_maps}"

echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}World to Map Converter${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}Configuration:${NC}"
echo -e "  World File  : ${WORLD_FILE}"
echo -e "  Resolution  : ${RESOLUTION} m/pixel"
echo -e "  Output Dir  : ${OUTPUT_DIR}"
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS2 not sourced!${NC}"
    echo -e "${YELLOW}Please run: source /opt/ros/jazzy/setup.bash${NC}"
    exit 1
fi

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

echo -e "${YELLOW}Step 1: Converting world file to map...${NC}"
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

echo -e "${YELLOW}Step 2: Starting map publisher...${NC}"

# Start map publisher in background
ros2 run ros_humans_ros2 map_publisher \
    --ros-args \
    -p yaml_file:="${OUTPUT_BASE}.yaml" \
    -p publish_rate:=1.0 \
    -p frame_id:=map &

MAP_PUB_PID=$!

# Wait a moment for publisher to start
sleep 2

if ! ps -p $MAP_PUB_PID > /dev/null 2>&1; then
    echo -e "${RED}Error: Map publisher failed to start!${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Map publisher started (PID: $MAP_PUB_PID)${NC}"
echo ""

echo -e "${YELLOW}Step 3: Launching RViz...${NC}"

# Find RViz config
RVIZ_CONFIG="$PWD/scan_project/rviz_config.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    RVIZ_CONFIG="$HOME/Videos/irpp_proj_git/scan_project/rviz_config.rviz"
fi

if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG" &
    RVIZ_PID=$!
else
    rviz2 &
    RVIZ_PID=$!
    echo -e "${YELLOW}Warning: RViz config not found, using default configuration${NC}"
fi

echo ""
echo -e "${GREEN}✓ RViz launched (PID: $RVIZ_PID)${NC}"
echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}Map Visualization Active!${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}RViz Configuration:${NC}"
echo -e "  1. In RViz, click 'Add' button (bottom left)"
echo -e "  2. Select 'By topic' tab"
echo -e "  3. Add '/map' -> 'Map' display"
echo -e "  4. Set Fixed Frame to 'map' (in Global Options)"
echo ""
echo -e "${YELLOW}Map Information:${NC}"
echo -e "  Topic     : /map"
echo -e "  Frame     : map"
echo -e "  Rate      : 1 Hz"
echo ""
echo -e "${YELLOW}Additional Commands:${NC}"
echo -e "  View map info:"
echo -e "    ros2 topic echo /map --once"
echo ""
echo -e "  List all topics:"
echo -e "    ros2 topic list"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop all processes${NC}"
echo -e "${BLUE}========================================${NC}"

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}Shutting down...${NC}"
    kill $MAP_PUB_PID 2>/dev/null || true
    kill $RVIZ_PID 2>/dev/null || true
    wait $MAP_PUB_PID 2>/dev/null || true
    wait $RVIZ_PID 2>/dev/null || true
    echo -e "${GREEN}Cleanup complete!${NC}"
}

# Set trap for cleanup
trap cleanup EXIT INT TERM

# Wait for user to stop
wait $RVIZ_PID

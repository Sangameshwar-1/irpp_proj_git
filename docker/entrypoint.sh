#!/bin/bash
# =============================================================================
#  Container entrypoint — sources ROS 2 + workspace, then runs the command
# =============================================================================
set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source the built workspace
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
fi

# Generate the map if it doesn't already exist
MAP_DIR="/root/ros2_maps"
PKG_SHARE="$(ros2 pkg prefix ros_humans_ros2 2>/dev/null)/share/ros_humans_ros2" || true

# Auto-generate the default map on first run
if [ -d "$PKG_SHARE/worlds" ] && [ ! -f "$MAP_DIR/large_messy_room_map.yaml" ]; then
    echo "[entrypoint] Generating default map from large_messy_room.world..."
    ros2 run ros_humans_ros2 world_to_map \
        "$PKG_SHARE/worlds/large_messy_room.world" \
        -o "$MAP_DIR/large_messy_room_map" \
        -r 0.05 --inflate 0.10 --padding 1.5 || true
fi

# Execute whatever command was passed (default: bash)
exec "$@"

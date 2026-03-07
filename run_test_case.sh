#!/bin/bash
# ============================================================
#  run_test_case.sh — Social Navigation Planning Test Runner
# ============================================================
#
#  Usage:
#    ./run_test_case.sh case1    — static human blocking path
#    ./run_test_case.sh case2    — human approaching head-on
#    ./run_test_case.sh case3    — human crossing (collision cone)
#    ./run_test_case.sh case4a   — human ahead, same direction
#    ./run_test_case.sh case4b   — human behind, faster (give way)
#    ./run_test_case.sh case5    — fast human from behind (replan)
#    ./run_test_case.sh case6a   — diagonal crossing from upper-left
#    ./run_test_case.sh case6b   — diagonal crossing from lower-left
#    ./run_test_case.sh case6c   — diagonal approach from upper-right
#    ./run_test_case.sh case6d   — diagonal approach from lower-right
#
#  Prerequisites:
#    1. Workspace built:
#         cd ~/ros2_ws && colcon build --symlink-install
#    2. Map generated (first run only):
#         ./run_test_case.sh --gen-map
#    3. RViz open with rviz_config.rviz for visualisation
#
#  What this script does:
#    • Sources the workspace
#    • Selects world/map/spawn/goal parameters for the chosen case
#    • Launches Gazebo + ROS 2 nav stack (NO perception, ground-truth only)
#    • Displays a reminder of what social behaviour to expect
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_SOURCE_DIR="$SCRIPT_DIR/scan_project/human_models/ros_humans_ros2"
WORKSPACE_DIR="$HOME/ros2_ws"
MAPS_DIR="$HOME/ros2_maps"
RVIZ_CONFIG="$SCRIPT_DIR/scan_project/rviz_config.rviz"

# ── Source ROS 2 base ───────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash

# ── Copy + build in ~/ros2_ws (same as run_test_scenario.sh) ───────────────
mkdir -p "$WORKSPACE_DIR/src"
TARGET_DIR="$WORKSPACE_DIR/src/ros_humans_ros2"
echo "Syncing source to workspace..."
rm -rf "$TARGET_DIR"
cp -r "$PROJECT_SOURCE_DIR" "$WORKSPACE_DIR/src/"

echo "Building workspace..."
cd "$WORKSPACE_DIR"
colcon build --symlink-install --packages-select ros_humans_ros2

# Sync Python files that colcon copies instead of symlinking
PY_SRC="$WORKSPACE_DIR/src/ros_humans_ros2/ros_humans_ros2"
PY_BUILD="$WORKSPACE_DIR/build/ros_humans_ros2/ros_humans_ros2"
if [[ -d "$PY_BUILD" ]]; then
    for f in "$PY_SRC"/*.py; do
        src_inode=$(stat -c '%i' "$f" 2>/dev/null)
        dst="$PY_BUILD/$(basename "$f")"
        dst_inode=$(stat -c '%i' "$dst" 2>/dev/null)
        if [[ "$src_inode" != "$dst_inode" ]]; then
            cp "$f" "$dst"
        fi
    done
fi

source "$WORKSPACE_DIR/install/setup.bash"

# ── Locate installed package share ─────────────────────────────────────────
PKG_SHARE="$(ros2 pkg prefix ros_humans_ros2)/share/ros_humans_ros2"
WORLD_FILE="$PKG_SHARE/worlds/test_case_arena.world"
MAP_YAML="${MAPS_DIR}/test_case_arena.yaml"

if [[ ! -f "$WORLD_FILE" ]]; then
    echo "[ERROR] world file not found after build: $WORLD_FILE"
    exit 1
fi

# ── Generate map if requested (or auto-generate if missing) ─────────────────
_gen_map() {
    echo ""
    echo "╔══════════════════════════════════════════════════════╗"
    echo "║         Generating map for test_case_arena           ║"
    echo "╚══════════════════════════════════════════════════════╝"
    mkdir -p "$MAPS_DIR"
    ros2 run ros_humans_ros2 world_to_map \
        "$WORLD_FILE" \
        -o "${MAPS_DIR}/test_case_arena" \
        -r 0.05 \
        --inflate 0.10 \
        --padding 1.5
    echo "[OK] Map saved to ${MAPS_DIR}/test_case_arena.yaml"
}

if [[ "$1" == "--gen-map" ]]; then
    _gen_map
    exit 0
fi

# Auto-generate map if not present
if [[ ! -f "$MAP_YAML" ]]; then
    echo "Map not found — generating automatically..."
    _gen_map
fi

if [[ ! -f "$MAP_YAML" ]]; then
    echo "[ERROR] Map generation failed. Check world_to_map output above."
    exit 1
fi

# ── Case parameter table ────────────────────────────────────────────────────
CASE="${1:-case1}"

case "$CASE" in

    case1)
        SPAWN_X=-4.0; SPAWN_Y=0.0; SPAWN_YAW=0.0
        GOAL_X=4.5;   GOAL_Y=0.0
        LABEL="CASE 1 — Static Blocker"
        EXPECT="Human is stationary on the direct path (0,0).
  Expected: Robot detects a static obstacle, publishes a weight zone,
  requests A* reroute, and navigates AROUND the human."
        ;;

    case2)
        SPAWN_X=-4.0; SPAWN_Y=0.0; SPAWN_YAW=0.0
        GOAL_X=5.0;   GOAL_Y=0.0
        LABEL="CASE 2 — Head-On Approach"
        EXPECT="Human starts at (3.5, 0) moving LEFT at -0.4 m/s (directly toward robot).
  Expected: Robot SLOWS DOWN proportionally as human approaches,
  comes to a STOP within personal zone, then resumes once human passes."
        ;;

    case3)
        SPAWN_X=-4.0; SPAWN_Y=0.0; SPAWN_YAW=0.0
        GOAL_X=5.0;   GOAL_Y=0.0
        LABEL="CASE 3 — Crossing (Collision Cone)"
        EXPECT="Human starts at (0, 4.0) moving DOWN at -0.30 m/s — same arrival time as robot.
  At full speed both arrive at (0,0) at t=13.3s: CERTAIN collision without action.
  Expected: Collision cone turns RED, robot SLOWS to ~0.22 m/s so it arrives
  AFTER human's personal zone (0.8m) has fully cleared, passing BEHIND.
  Robot maintains >0.8m personal zone clearance throughout."
        ;;

    case4a)
        SPAWN_X=-4.0; SPAWN_Y=0.0; SPAWN_YAW=0.0
        GOAL_X=5.0;   GOAL_Y=0.0
        LABEL="CASE 4a — Ahead, Same Direction (No Overtake)"
        EXPECT="Human starts at (-1.0, 0) moving RIGHT at 0.2 m/s (same direction, ahead).
  Expected: Robot MATCHES the human's speed (0.22 m/s) within OVERTAKE_WARN
  distance, never overtaking.  Keeps ~2 m gap behind human."
        ;;

    case4b)
        SPAWN_X=-3.0; SPAWN_Y=0.0; SPAWN_YAW=0.0
        GOAL_X=5.0;   GOAL_Y=0.0
        LABEL="CASE 4b — Behind, Faster (Give Way)"
        EXPECT="Human starts at (-5.0, 0) moving RIGHT at 0.4 m/s (same direction, behind).
  Robot spawns at (-3.0, 0).  Human is faster and will overtake robot.
  Expected: Robot GIVES WAY — slows to 0.12 m/s + slight lateral offset
  so the faster human can overtake safely."
        ;;

    case5)
        SPAWN_X=-4.0; SPAWN_Y=0.0; SPAWN_YAW=0.0
        GOAL_X=5.0;   GOAL_Y=0.0
        LABEL="CASE 5 — Fast Human From Behind (Replan)"
        EXPECT="Human starts at (-7.0, 0) moving RIGHT at 0.60 m/s (same direction, behind).
  Human is TWICE the robot speed (0.30 m/s) and will catch up in ~10 s.
  Expected: When human enters the social circle (blue ring, 1.5 m),
  robot REPLANS its path to move aside and let the human pass."
        ;;

    case6a)
        SPAWN_X=-4.0; SPAWN_Y=0.0; SPAWN_YAW=0.0
        GOAL_X=5.0;   GOAL_Y=0.0
        LABEL="CASE 6a — Diagonal Crossing (Upper-Left)"
        EXPECT="Human starts at (-1.0, 3.5) moving diagonally DOWN-RIGHT
  at vel (0.20, -0.30) m/s — crosses the robot's straight-line path.
  Expected: Robot detects diagonal crossing via collision cone,
  SLOWS or REPLANS to pass BEHIND the human."
        ;;

    case6b)
        SPAWN_X=-4.0; SPAWN_Y=0.0; SPAWN_YAW=0.0
        GOAL_X=5.0;   GOAL_Y=0.0
        LABEL="CASE 6b — Diagonal Crossing (Lower-Left)"
        EXPECT="Human starts at (-1.0, -3.5) moving diagonally UP-RIGHT
  at vel (0.20, 0.30) m/s — crosses the robot's straight-line path.
  Expected: Robot detects diagonal crossing via collision cone,
  SLOWS or REPLANS to pass BEHIND the human."
        ;;

    case6c)
        SPAWN_X=-4.0; SPAWN_Y=0.0; SPAWN_YAW=0.0
        GOAL_X=5.0;   GOAL_Y=0.0
        LABEL="CASE 6c — Diagonal Approach (Upper-Right)"
        EXPECT="Human starts at (3.0, 2.5) moving diagonally DOWN-LEFT
  at vel (-0.30, -0.15) m/s — approaches robot from upper-right.
  Expected: Robot SLOWS as human enters social zone, then REPLANS
  to avoid collision when entering social circle."
        ;;

    case6d)
        SPAWN_X=-4.0; SPAWN_Y=0.0; SPAWN_YAW=0.0
        GOAL_X=5.0;   GOAL_Y=0.0
        LABEL="CASE 6d — Diagonal Approach (Lower-Right)"
        EXPECT="Human starts at (3.0, -2.5) moving diagonally UP-LEFT
  at vel (-0.30, 0.15) m/s — approaches robot from lower-right.
  Expected: Robot SLOWS as human enters social zone, then REPLANS
  to avoid collision when entering social circle."
        ;;

    *)
        echo ""
        echo "Unknown case: '$CASE'"
        echo ""
        echo "Valid cases: case1  case2  case3  case4a  case4b  case5  case6a  case6b  case6c  case6d"
        echo "Usage:       ./run_test_case.sh <case>"
        echo "Map gen:     ./run_test_case.sh --gen-map"
        echo ""
        exit 1
        ;;
esac

# ── Print test info ─────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
printf "║  %-61s║\n" "$LABEL"
echo "╠══════════════════════════════════════════════════════════════╣"
while IFS= read -r line; do
    printf "║  %-61s║\n" "$line"
done <<< "$EXPECT"
echo "╠══════════════════════════════════════════════════════════════╣"
printf "║  Robot:  spawn (%-5.1f, %-5.1f)  →  goal (%-5.1f, %-5.1f)         ║\n" \
    "$SPAWN_X" "$SPAWN_Y" "$GOAL_X" "$GOAL_Y"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  RViz: open rviz_config.rviz — enable 'SocialNavMarkers'    ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "Launching in 2 seconds…  Press Ctrl+C to abort"
sleep 2

# ── Launch nav stack in background ──────────────────────────────────────────
ros2 launch ros_humans_ros2 social_nav_cases.launch.py \
    case:="$CASE" \
    world_name:=test_case_arena \
    world_file:="$WORLD_FILE" \
    map_yaml:="$MAP_YAML" \
    spawn_x:="$SPAWN_X" \
    spawn_y:="$SPAWN_Y" \
    spawn_yaw:="$SPAWN_YAW" \
    goal_x:="$GOAL_X" \
    goal_y:="$GOAL_Y" \
    room_min_x:=-6.5 \
    room_max_x:=6.5 \
    room_min_y:=-4.5 \
    room_max_y:=4.5 \
    map_size:=16.0 &
LAUNCH_PID=$!

echo "Navigation launched (PID: $LAUNCH_PID)"

# ── Launch RViz ──────────────────────────────────────────────────────────────
sleep 3
export OGRE_RTT_MODE=Copy
if [[ -f "$RVIZ_CONFIG" ]]; then
    rviz2 -d "$RVIZ_CONFIG" &
else
    rviz2 &
fi

echo ""
echo "  Topics to monitor:"
echo "    /social_nav_markers  — collision cone + decision text"
echo "    /global_path         — A* planned path"
echo "    /robot_pose          — localised pose"
echo "    /detected_humans     — ground-truth human position"
echo ""

wait $LAUNCH_PID

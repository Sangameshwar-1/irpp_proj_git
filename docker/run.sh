#!/bin/bash
# =============================================================================
#  Run the IRPP container  —  Gazebo + RViz + Camera always enabled
# =============================================================================
#
#  Usage:
#    ./run.sh navigation               # GT perception + Gazebo + RViz
#    ./run.sh navigation camera         # camera perception + Gazebo + RViz
#    ./run.sh navigation camera 8.0 3.0 # custom goal
#    ./run.sh test case1               # test case + Gazebo + RViz
#    ./run.sh test case3               # crossing test
#    ./run.sh scenario corridor_blocked # test scenario + Gazebo + RViz
#    ./run.sh scenario open_room
#    ./run.sh shell                     # interactive bash inside container
#    ./run.sh stop                      # stop all containers
#
# =============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Allow X11 connections from the container
xhost +local:docker 2>/dev/null || true

# Kill any stale Gazebo processes from previous runs (host networking
# shares gz-transport between container and host — leftover gz sim
# servers cause the "default world with poles" to appear alongside
# the custom world).
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gz-sim" 2>/dev/null || true
pkill -9 -f "ruby.*gz" 2>/dev/null || true
pkill -9 -f "gzserver" 2>/dev/null || true
pkill -9 -f "gz-gui" 2>/dev/null || true
# Also stop any lingering Docker containers from previous runs
docker stop irpp-social-nav 2>/dev/null || true
docker rm -f irpp-social-nav 2>/dev/null || true
sleep 0.5

MODE="${1:-shell}"

# ── Helpers ─────────────────────────────────────────────────────────
_source_cmd() {
    echo "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash"
}

_mapgen_cmd() {               # $1 = world filename, $2 = output basename
    echo "mkdir -p /root/ros2_maps && \
PKG_SHARE=\$(ros2 pkg prefix ros_humans_ros2)/share/ros_humans_ros2 && \
ros2 run ros_humans_ros2 world_to_map \
    \$PKG_SHARE/worlds/$1 \
    -o /root/ros2_maps/$2 \
    -r 0.05 --inflate 0.10 --padding 1.5"
}

echo -e "${BLUE}═══════════════════════════════════════════${NC}"
echo -e "${BLUE}  IRPP Social Navigation (Docker)          ${NC}"
echo -e "${BLUE}  Gazebo GUI + RViz + Camera — always on   ${NC}"
echo -e "${BLUE}═══════════════════════════════════════════${NC}"

case "$MODE" in

    # ═════════════════════════════════════════════════════════════════
    #  NAVIGATION  (Gazebo GUI + RViz + nav stack)
    # ═════════════════════════════════════════════════════════════════
    navigation|nav)
        PERCEPTION="${2:-camera}"
        GOAL_X="${3:-5.0}"
        GOAL_Y="${4:-5.0}"

        echo -e "${CYAN}  Mode:       Full Navigation Stack${NC}"
        echo -e "${CYAN}  Perception: $PERCEPTION${NC}"
        echo -e "${CYAN}  Goal:       ($GOAL_X, $GOAL_Y)${NC}"
        echo -e "${GREEN}  Gazebo GUI: ✓   RViz: ✓   Camera: ✓${NC}"
        echo ""

        docker compose run --rm irpp bash -c "\
$(_source_cmd) && \
$(_mapgen_cmd large_messy_room.world large_messy_room_map) && \
echo '' && echo 'Launching Gazebo + RViz + Navigation ...' && \
ros2 launch ros_humans_ros2 navigation.launch.py \
    map_yaml:=/root/ros2_maps/large_messy_room_map.yaml \
    default_goal_x:=$GOAL_X \
    default_goal_y:=$GOAL_Y \
    perception_mode:=$PERCEPTION \
    headless:=false & \
LAUNCH_PID=\$! && \
sleep 6 && echo 'Starting RViz ...' && \
rviz2 -d /root/rviz_config.rviz & \
wait \$LAUNCH_PID"
        ;;

    # ═════════════════════════════════════════════════════════════════
    #  TEST CASE  (Gazebo GUI + RViz + test)
    # ═════════════════════════════════════════════════════════════════
    test)
        CASE="${2:-case1}"

        echo -e "${CYAN}  Mode: Isolated Test Case${NC}"
        echo -e "${CYAN}  Case: $CASE${NC}"
        echo -e "${GREEN}  Gazebo GUI: ✓   RViz: ✓   Camera: ✓${NC}"
        echo ""

        # Print case description
        case "$CASE" in
            case1)  echo -e "${YELLOW}  → Static Blocker — robot reroutes around stationary human${NC}" ;;
            case2)  echo -e "${YELLOW}  → Head-On — robot slows/stops as human approaches${NC}" ;;
            case3)  echo -e "${YELLOW}  → Crossing — collision cone, robot slows to pass behind${NC}" ;;
            case4a) echo -e "${YELLOW}  → Same direction ahead — robot matches speed${NC}" ;;
            case4b) echo -e "${YELLOW}  → Same direction behind — robot gives way${NC}" ;;
            case5)  echo -e "${YELLOW}  → Fast from behind — robot replans${NC}" ;;
            case6a) echo -e "${YELLOW}  → Diagonal crossing (upper-left)${NC}" ;;
            case6b) echo -e "${YELLOW}  → Diagonal crossing (lower-left)${NC}" ;;
            case6c) echo -e "${YELLOW}  → Diagonal approach (upper-right)${NC}" ;;
            case6d) echo -e "${YELLOW}  → Diagonal approach (lower-right)${NC}" ;;
        esac
        echo ""

        docker compose run --rm irpp bash -c "\
$(_source_cmd) && \
$(_mapgen_cmd test_case_arena.world test_case_arena) && \
echo '' && echo 'Launching Gazebo + RViz + Test Case: $CASE ...' && \
ros2 launch ros_humans_ros2 social_nav_cases.launch.py \
    case:=$CASE \
    map_yaml:=/root/ros2_maps/test_case_arena.yaml \
    headless:=false & \
LAUNCH_PID=\$! && \
sleep 6 && echo 'Starting RViz ...' && \
rviz2 -d /root/rviz_config.rviz & \
wait \$LAUNCH_PID"
        ;;

    # ═════════════════════════════════════════════════════════════════
    #  SCENARIO  (Gazebo GUI + RViz + scenario)
    # ═════════════════════════════════════════════════════════════════
    scenario)
        SCENARIO="${2:-corridor_blocked}"

        case "$SCENARIO" in
            corridor_blocked)
                WORLD_NAME="test_corridor_blocked"
                SPAWN_X="-4.0"; SPAWN_Y="0.0"; SPAWN_YAW="0.0"
                GOAL_X="4.0";   GOAL_Y="0.0"
                ROOM_MIN_X="-5.5"; ROOM_MAX_X="5.5"
                ROOM_MIN_Y="-2.0"; ROOM_MAX_Y="2.0"
                MAP_SIZE="14.0"
                DESC="Straight 10m corridor — human blocking the centre"
                ;;
            l_corridor)
                WORLD_NAME="test_l_corridor"
                SPAWN_X="-3.0"; SPAWN_Y="-2.75"; SPAWN_YAW="0.0"
                GOAL_X="2.25";  GOAL_Y="4.0"
                ROOM_MIN_X="-5.0"; ROOM_MAX_X="4.0"
                ROOM_MIN_Y="-4.0"; ROOM_MAX_Y="5.0"
                MAP_SIZE="12.0"
                DESC="L-shaped corridor — human at the inner corner"
                ;;
            open_room)
                WORLD_NAME="test_open_room"
                SPAWN_X="-3.0"; SPAWN_Y="-3.0"; SPAWN_YAW="0.7854"
                GOAL_X="3.0";   GOAL_Y="3.0"
                ROOM_MIN_X="-4.5"; ROOM_MAX_X="4.5"
                ROOM_MIN_Y="-4.5"; ROOM_MAX_Y="4.5"
                MAP_SIZE="12.0"
                DESC="Open 8m×8m room — human on diagonal + obstacle"
                ;;
            doorway)
                WORLD_NAME="test_doorway"
                SPAWN_X="-3.0"; SPAWN_Y="0.0"; SPAWN_YAW="0.0"
                GOAL_X="3.0";   GOAL_Y="0.0"
                ROOM_MIN_X="-5.5"; ROOM_MAX_X="5.5"
                ROOM_MIN_Y="-3.0"; ROOM_MAX_Y="3.0"
                MAP_SIZE="14.0"
                DESC="Two rooms + doorway — human near the doorway"
                ;;
            *)
                echo -e "${RED}Unknown scenario: $SCENARIO${NC}"
                echo ""
                echo "Available: corridor_blocked  l_corridor  open_room  doorway"
                exit 1
                ;;
        esac

        echo -e "${CYAN}  Mode:     Test Scenario${NC}"
        echo -e "${CYAN}  Scenario: $SCENARIO${NC}"
        echo -e "${YELLOW}  $DESC${NC}"
        echo -e "${CYAN}  Spawn:    ($SPAWN_X, $SPAWN_Y) yaw=$SPAWN_YAW${NC}"
        echo -e "${CYAN}  Goal:     ($GOAL_X, $GOAL_Y)${NC}"
        echo -e "${GREEN}  Gazebo GUI: ✓   RViz: ✓   Camera: ✓${NC}"
        echo ""

        docker compose run --rm irpp bash -c "\
$(_source_cmd) && \
$(_mapgen_cmd ${WORLD_NAME}.world ${WORLD_NAME}) && \
echo '' && echo 'Launching Gazebo + RViz + Scenario: $SCENARIO ...' && \
ros2 launch ros_humans_ros2 test_scenario.launch.py \
    world_name:=$WORLD_NAME \
    world_file:=\$(ros2 pkg prefix ros_humans_ros2)/share/ros_humans_ros2/worlds/${WORLD_NAME}.world \
    map_yaml:=/root/ros2_maps/${WORLD_NAME}.yaml \
    spawn_x:=$SPAWN_X spawn_y:=$SPAWN_Y spawn_yaw:=$SPAWN_YAW \
    goal_x:=$GOAL_X goal_y:=$GOAL_Y \
    room_min_x:=$ROOM_MIN_X room_max_x:=$ROOM_MAX_X \
    room_min_y:=$ROOM_MIN_Y room_max_y:=$ROOM_MAX_Y \
    map_size:=$MAP_SIZE & \
LAUNCH_PID=\$! && \
sleep 6 && echo 'Starting RViz ...' && \
rviz2 -d /root/rviz_config.rviz & \
wait \$LAUNCH_PID"
        ;;

    # ═════════════════════════════════════════════════════════════════
    #  SHELL  (interactive bash)
    # ═════════════════════════════════════════════════════════════════
    shell|bash)
        echo -e "${CYAN}  Mode: Interactive Shell${NC}"
        echo ""
        docker compose run --rm irpp
        ;;

    # ═════════════════════════════════════════════════════════════════
    #  STOP
    # ═════════════════════════════════════════════════════════════════
    stop)
        echo -e "${CYAN}  Stopping all containers...${NC}"
        docker compose down
        echo -e "${GREEN}  Done.${NC}"
        ;;

    # ═════════════════════════════════════════════════════════════════
    #  HELP / UNKNOWN
    # ═════════════════════════════════════════════════════════════════
    *)
        echo -e "${RED}Unknown mode: $MODE${NC}"
        echo ""
        echo "Usage:"
        echo "  ./run.sh navigation [camera|no_camera] [goal_x] [goal_y]"
        echo "  ./run.sh test [case1|case2|case3|case4a|case4b|case5|case6a-d]"
        echo "  ./run.sh scenario [corridor_blocked|l_corridor|open_room|doorway]"
        echo "  ./run.sh shell                # interactive bash"
        echo "  ./run.sh stop                 # stop containers"
        echo ""
        echo "Every mode launches Gazebo GUI + RViz + Camera automatically."
        exit 1
        ;;
esac

# Revoke X11 access
xhost -local:docker 2>/dev/null || true

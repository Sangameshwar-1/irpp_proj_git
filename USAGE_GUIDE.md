# Usage Guide

---

## Method 1 — Shell Scripts (recommended, build included)

All scripts automatically build and source the workspace before launching.
Run from the repo root: `~/Videos/irpp_proj_git/`

### Navigation

```bash
./run_navigation.sh [camera|no_camera] [goal_x] [goal_y]
```

| Mode | Description |
|---|---|
| `no_camera` (default) | Ground-truth human injection via `move_humans` |
| `camera` | HSV red-blob + Kalman tracker via `human_detector_red` |

Examples:
```bash
./run_navigation.sh                        # no_camera, goal (5.0, 5.0)
./run_navigation.sh camera 3.0 4.0        # camera mode, custom goal
./run_navigation.sh no_camera 5.0 5.0
```

---

### Test Cases (social navigation scenarios)

```bash
./run_test_case.sh <case>
```

| Case | Description |
|---|---|
| `case1` | Static human blocking path |
| `case2` | Human approaching head-on |
| `case3` | Human crossing (collision cone) |
| `case4a` | Human ahead, same direction (no overtake) |
| `case4b` | Human behind, faster (give way) |
| `case5` | Fast human from behind (replan) |
| `case6a` | Diagonal crossing from upper-left |
| `case6b` | Diagonal crossing from lower-left |
| `case6c` | Diagonal approach from upper-right |
| `case6d` | Diagonal approach from lower-right |

Examples:
```bash
./run_test_case.sh case1
./run_test_case.sh case3
./run_test_case.sh --gen-map    # regenerate map only
```

---

### Test Scenarios (environment layouts)

```bash
./run_test_scenario.sh <scenario>
```

| Scenario | Description |
|---|---|
| `corridor_blocked` | Straight 10m corridor, human blocking centre |
| `l_corridor` | L-shaped corridor, human at corner |
| `open_room` | Open 8m×8m room, human on diagonal |
| `doorway` | Two rooms + doorway, human near door |

Examples:
```bash
./run_test_scenario.sh corridor_blocked
./run_test_scenario.sh open_room
```

---

## Method 2 — Manual Build + Run

Use this when running individual nodes or making quick code changes.

### Build

```bash
cd ~/ros2_ws
cp -r ~/Videos/irpp_proj_git/scan_project/human_models/ros_humans_ros2 src/
colcon build --symlink-install --packages-select ros_humans_ros2
source install/setup.bash
```

### Launch files

```bash
ros2 launch ros_humans_ros2 navigation.launch.py
ros2 launch ros_humans_ros2 test_scenario.launch.py
ros2 launch ros_humans_ros2 social_nav_cases.launch.py
ros2 launch ros_humans_ros2 world_to_map.launch.py
```

### Individual nodes

```bash
ros2 run ros_humans_ros2 move_humans
ros2 run ros_humans_ros2 human_detector_red
ros2 run ros_humans_ros2 world_to_map
ros2 run ros_humans_ros2 map_publisher
ros2 run ros_humans_ros2 global_planner
ros2 run ros_humans_ros2 localization_node
ros2 run ros_humans_ros2 social_nav_planner
ros2 run ros_humans_ros2 human_case_controller
ros2 run ros_humans_ros2 live_visualization_node
ros2 run ros_humans_ros2 pointcloud_mapper
ros2 run ros_humans_ros2 camera_view_360
```

---

## Key Topics

| Topic | Description |
|---|---|
| `/robot_pose` | Localised robot pose |
| `/global_path` | A* planned path |
| `/detected_humans` | Human detections (GT or camera) |
| `/human_ground_truth` | Raw GT positions (always published) |
| `/weighted_grid_viz` | Edge weights visualisation |
| `/social_nav_markers` | Collision cone + decision text |
| `/goal_pose` | Publish a new navigation goal |

### Set a new goal at runtime

```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 5.0, y: 5.0}}}'
```
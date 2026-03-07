# Social Navigation for Mobile Robots — IRPP Project

> **ROS 2 Jazzy · Gazebo Harmonic · Ubuntu 24.04 · Python 3.12**

A complete socially-aware robot navigation system that detects humans
through vision (or uses ground-truth injection), localises via scan-matched
odometry, plans globally with A\*, and applies **five distinct social
behaviour cases** in real-time — all running in a Gazebo simulation with
full RViz visualisation.

---

## Table of Contents

1.  [System Overview](#1-system-overview)
2.  [High-Level Architecture](#2-high-level-architecture)
3.  [Directory Structure](#3-directory-structure)
4.  [Software Stack & Libraries](#4-software-stack--libraries)
5.  [Node Reference (Detailed)](#5-node-reference-detailed)
    - 5.1  [Simulation & Bridges](#51-simulation--bridges)
    - 5.2  [Map Publisher](#52-map-publisher-map_publisherpy)
    - 5.3  [Pointcloud Mapper](#53-pointcloud-mapper-pointcloud_mapperpy)
    - 5.4  [Localization Node](#54-localization-node-localization_nodepy)
    - 5.5  [Human Mover](#55-human-mover-move_humanspy)
    - 5.6  [Human Detector (Red)](#56-human-detector-red-human_detector_redpy)
    - 5.7  [Camera View 360](#57-camera-view-360-camera_view_360py)
    - 5.8  [Weighted Grid](#58-weighted-grid-module-weighted_gridpy)
    - 5.9  [Global Planner](#59-global-planner-global_plannerpy)
    - 5.10 [Local Planner](#510-local-planner-local_plannerpy)
    - 5.11 [Social Nav Planner](#511-social-navigation-planner-social_nav_plannerpy)
    - 5.12 [Human Case Controller](#512-human-case-controller-human_case_controllerpy)
    - 5.13 [Live Visualization Node](#513-live-visualization-node-live_visualization_nodepy)
    - 5.14 [A\* Path Planner (Legacy)](#514-a-path-planner-legacy-astar_path_plannerpy)
    - 5.15 [World-to-Map Converter](#515-world-to-map-converter-world_to_mappy)
6.  [Data Flow & Topic Map](#6-data-flow--topic-map)
7.  [Coordinate Frames (TF Tree)](#7-coordinate-frames-tf-tree)
8.  [Social Navigation Cases](#8-social-navigation-cases)
9.  [Planning Architecture — How A\* Rerouting Works](#9-planning-architecture--how-a-rerouting-works)
10. [Launch Files](#10-launch-files)
11. [Gazebo Worlds](#11-gazebo-worlds)
12. [Quick Start](#12-quick-start)
13. [Shell Scripts Reference](#13-shell-scripts-reference)
14. [Key Parameters & Constants](#14-key-parameters--constants)
15. [Troubleshooting](#15-troubleshooting)

---

## 1. System Overview

The system drives a **TurtleBot3-Rover** through Gazebo worlds populated
with walking human models. Three software layers cooperate in a
publisher–subscriber ROS 2 graph:

| Layer            | Responsibility                                                                  |
|------------------|---------------------------------------------------------------------------------|
| **Perception**   | Detect humans from 4 RGB cameras (or use ground-truth injection), estimate world position & velocity |
| **Localisation** | Correct differential-drive odometry with correlative LiDAR scan-matching        |
| **Planning**     | Global A\* path on a weighted grid, social behaviour rules, velocity control    |

The planner classifies every detected human into one of **five social
navigation cases** and adjusts robot speed, heading, or route
accordingly — never entering a human's personal zone (0.8 m) and
reducing speed inside the social zone (1.5 m).

### Two Operating Modes

| Mode                    | Launch File                  | Perception Source         | Planner          |
|-------------------------|------------------------------|---------------------------|------------------|
| **Full Stack**          | `navigation.launch.py`      | `move_humans` (GT inject) | `social_nav_planner` |
| **Isolated Test Cases** | `social_nav_cases.launch.py` | `human_case_controller`  | `social_nav_planner` |

---

## 2. High-Level Architecture

```
            ┌──────────────────────────────────────────────────────────┐
            │                    Gazebo Simulation                     │
            │  TurtleBot3-Rover  ·  Human Models  ·  Environment      │
            └────────┬────────────────┬──────────────────┬─────────────┘
                     │ /odom          │ /camera/*        │ /scan
            ┌────────▼───────┐  ┌─────▼───────────┐  ┌──▼──────────────┐
            │  Localisation  │  │   Perception    │  │  Map Publisher   │
            │  (scan-match   │  │  move_humans or │  │  (PGM → /map    │
            │   odometry)    │  │  human_detector │  │   OccupancyGrid)│
            └────────┬───────┘  └─────┬───────────┘  └──┬──────────────┘
                     │ /robot_pose    │ /detected_      │ /map
                     │                │   humans        │
            ┌────────▼────────────────▼─────────────────▼────────────────┐
            │                    Planning Layer                          │
            │                                                            │
            │  ┌────────────────────────────────────────────────────┐    │
            │  │           global_planner  (A* on WeightedGrid)    │    │
            │  │  /map → build grid → A* → /global_path            │    │
            │  │  Proactive monitor: reroutes BEFORE encounter     │    │
            │  │  Reactive replan: /weight_zones + /replan_request  │    │
            │  └────────────────────┬───────────────────────────────┘    │
            │                       │ /global_path                       │
            │  ┌────────────────────▼───────────────────────────────┐    │
            │  │       social_nav_planner  (case-based VO)         │    │
            │  │  Classifies each human → 5 social cases           │    │
            │  │  Collision cone · Crossing timing · Speed control │    │
            │  │  → /cmd_vel  → /weight_zones  → /replan_request  │    │
            │  └────────────────────┬───────────────────────────────┘    │
            │                       │                                    │
            └───────────────────────┼────────────────────────────────────┘
                                    │ /cmd_vel
                            ┌───────▼───────┐
                            │   Robot Base   │
                            │   (Gazebo)     │
                            └───────────────┘
```

### Closed-Loop Summary

1. **Gazebo** publishes `/odom`, `/scan`, `/camera/*` sensor data.
2. **map_publisher** loads a PGM/YAML occupancy map and publishes it on `/map` (TRANSIENT_LOCAL).
3. **pointcloud_mapper** publishes the TF tree (`map→odom→base_footprint→sensors`).
4. **localization_node** fuses odometry + LiDAR scan-matching → publishes corrected `/robot_pose`.
5. **move_humans** (or `human_case_controller`) animates human models in Gazebo and publishes ground-truth `/detected_humans` + `/human_velocities`.
6. **global_planner** builds a `WeightedGrid` from `/map`, runs A\*, publishes `/global_path`. It proactively monitors humans and replans if any threaten the current path.
7. **social_nav_planner** follows `/global_path`, classifies each human into a social case, adjusts speed/heading, sends `/cmd_vel` to Gazebo, and requests replans from the global planner when needed.
8. **live_visualization_node** builds live occupancy grids and overlay markers for RViz comparison (GT vs perception).

---

## 3. Directory Structure

```
irpp_proj_git/
├── README.md                          ← This file
├── run_simulation.sh                  ← Basic Gazebo + pointcloud demo
├── run_navigation.sh                  ← Full social navigation stack
├── run_test_case.sh                   ← Isolated social nav case tests
├── run_test_scenario.sh               ← Multi-scenario test runner
├── run_astar_navigation.sh            ← Legacy A* planner launch
├── run_astar_with_mapping.sh          ← Legacy A* + mapping launch
├── convert_world_to_map.sh            ← Convert .world → .pgm/.yaml
├── setup_dependencies.sh              ← Install ROS 2 + Gazebo + tools
│
├── scan_project/
│   ├── rviz_config.rviz               ← RViz saved configuration
│   └── human_models/
│       └── ros_humans_ros2/           ← ROS 2 Python package
│           ├── package.xml            ← ROS 2 package manifest
│           ├── setup.py               ← Python packaging / entry points
│           ├── setup.cfg              ← setuptools config
│           ├── resource/              ← ament resource marker
│           │
│           ├── launch/                ← Launch files (see §10)
│           │   ├── navigation.launch.py
│           │   ├── social_nav_cases.launch.py
│           │   ├── test_scenario.launch.py
│           │   ├── astar_navigation.launch.py
│           │   ├── demo.launch.py
│           │   ├── messy_road.launch.py
│           │   ├── iiit_messy_road.launch.py
│           │   └── world_to_map.launch.py
│           │
│           ├── ros_humans_ros2/       ← Python source (ROS 2 nodes)
│           │   ├── __init__.py
│           │   ├── social_nav_planner.py    ← Social navigation planner
│           │   ├── global_planner.py        ← A* on WeightedGrid
│           │   ├── local_planner.py         ← Reactive WAIT/REROUTE planner
│           │   ├── weighted_grid.py         ← Edge-weighted grid + A*
│           │   ├── localization_node.py     ← Scan-matched odometry
│           │   ├── map_publisher.py         ← PGM/YAML → /map
│           │   ├── pointcloud_mapper.py     ← LiDAR→pointcloud + TF
│           │   ├── move_humans.py           ← Animate humans in Gazebo
│           │   ├── human_detector_red.py    ← Red-colour human detector
│           │   ├── human_case_controller.py ← GT human injector (test mode)
│           │   ├── live_visualization_node.py ← GT vs perception grids
│           │   ├── camera_view_360.py       ← 4-camera panorama combiner
│           │   ├── astar_path_planner.py    ← Legacy monolithic planner
│           │   ├── world_to_map.py          ← .world → .pgm/.yaml converter
│           │   ├── rover_explorer.py        ← Autonomous exploration
│           │   ├── human_detector_cv.py     ← OpenCV-based detector (alt)
│           │   ├── publish_human_pose.py    ← Static human pose publisher
│           │   ├── messy_road_mover.py      ← Messy road human mover
│           │   ├── messy_road_pose_publisher.py
│           │   └── iiit_messy_road_mover.py
│           │
│           └── worlds/                ← Gazebo SDF world files
│               ├── large_messy_room.world    ← Primary test world (25×25 m)
│               ├── test_case_arena.world     ← Small arena for isolated tests
│               ├── test_open_room.world
│               ├── test_corridor_blocked.world
│               ├── test_doorway.world
│               ├── test_l_corridor.world
│               ├── indoor_with_humans.world
│               ├── messy_road.world
│               ├── iiit_messy_road.world
│               ├── iiit_hyderabad.world
│               ├── institute_city.world
│               ├── complex_grid_map.world
│               └── custom_map.world
```

---

## 4. Software Stack & Libraries

### 4.1 Core Framework

| Component              | Version / Details                                         |
|------------------------|-----------------------------------------------------------|
| **ROS 2**              | Jazzy Jalisco (Ubuntu 24.04)                              |
| **Gazebo**             | Harmonic (default sim for ROS 2 Jazzy)                    |
| **Python**             | 3.12                                                      |
| **Build system**       | `colcon` + `ament_python`                                 |

### 4.2 ROS 2 Packages Used

| Package                 | Purpose                                                     |
|-------------------------|-------------------------------------------------------------|
| `rclpy`                 | ROS 2 Python client library — nodes, publishers, subscribers, timers, services |
| `geometry_msgs`         | `Pose`, `PoseStamped`, `PoseArray`, `Twist`, `Point`, `TransformStamped`  |
| `nav_msgs`              | `OccupancyGrid`, `Odometry`, `Path`, `MapMetaData`         |
| `sensor_msgs`           | `LaserScan`, `Image`, `PointCloud2`, `PointField`           |
| `visualization_msgs`    | `Marker`, `MarkerArray` (RViz overlays)                     |
| `std_msgs`              | `Bool`, `ColorRGBA`, `Header`                               |
| `tf2_ros`               | `TransformBroadcaster`, `StaticTransformBroadcaster`        |
| `ros_gz_bridge`         | Bridges Gazebo topics ↔ ROS 2 topics                        |
| `ros_gz_interfaces`     | `SetEntityPose` service, `Entity` message (teleport models) |
| `ros_gz_sim`            | Gazebo launch integration (`gz_sim.launch.py`)              |

### 4.3 Python Libraries

| Library      | Purpose                                                            |
|--------------|--------------------------------------------------------------------|
| `numpy`      | Grid math, array operations, occupancy grid manipulation, Kalman filter matrices |
| `math`       | Trigonometry, quaternion/yaw conversion, distance calculations     |
| `heapq`      | Priority queue for A\* search                                      |
| `json`       | Serialise/deserialise human waypoint configurations                |
| `yaml`       | Parse map YAML metadata files                                      |
| `PIL`        | Load PGM map images (`Pillow`)                                     |
| `cv2`        | OpenCV — HSV thresholding, contour detection, morphology (optional)|
| `cv_bridge`  | Convert ROS `Image` ↔ OpenCV `Mat` (optional)                     |
| `struct`     | Binary packing for PointCloud2 fields                              |
| `xml.etree`  | Parse SDF .world files for offline map generation                  |
| `argparse`   | CLI argument parsing for the world_to_map tool                     |

---

## 5. Node Reference (Detailed)

### 5.1 Simulation & Bridges

Gazebo Harmonic runs the physics simulation. **`ros_gz_bridge`** parameter bridges connect Gazebo and ROS 2:

| Bridge               | Direction        | Gazebo Topic                         | ROS 2 Topic       | Type                    |
|----------------------|------------------|--------------------------------------|--------------------|-------------------------|
| Clock                | Gz → ROS         | `/clock`                             | `/clock`           | `Clock`                 |
| Command velocity     | ROS → Gz         | `/cmd_vel`, `/model/…/cmd_vel`       | `/cmd_vel`         | `Twist`                 |
| Odometry             | Gz → ROS         | `/odom`                              | `/odom`            | `Odometry`              |
| LiDAR                | Gz → ROS         | `/scan`                              | `/scan`            | `LaserScan`             |
| Cameras (×4)         | Gz → ROS         | `/camera/{front,right,back,left}/image` | same            | `Image`                 |
| Set Entity Pose      | ROS ↔ Gz         | `/world/<name>/set_pose`             | same               | `SetEntityPose` (service) |

### 5.2 Map Publisher (`map_publisher.py`)

Loads a pre-generated occupancy map from **PGM + YAML** files and publishes it on `/map` with **TRANSIENT_LOCAL** durability so late-joining subscribers receive it.

- **Input:** YAML file (resolution, origin) + PGM image (white=free, black=occupied)
- **Output:** `/map` (`OccupancyGrid`)
- **Key detail:** Image rows are flipped (PGM is top-down, ROS maps are bottom-up)

### 5.3 Pointcloud Mapper (`pointcloud_mapper.py`)

Provides the **TF tree** and optional live pointcloud mapping:

| TF Frame Chain | Transform Type |
|---|---|
| `map` → `odom` | **Static** (spawn position + yaw rotation) |
| `odom` → `base_footprint` | **Dynamic** (from `/odom` messages) |
| `base_footprint` → `base_link` | Static (z = 0.08 m) |
| `base_link` → `lidar_link` | Static (z = 0.10 m) |
| `base_link` → `camera_link` | Static (z = 0.14 m) |

Also converts LiDAR scans to world-frame pointclouds with odom-time synchronisation for correct RViz overlay. Includes an **odom jump filter** to reject DiffDrive collision spikes (clamped to 0.6 m/s max velocity, 2.5 rad/s max yaw rate).

### 5.4 Localization Node (`localization_node.py`)

**Scan-matched odometry** — corrects raw DiffDrive drift by matching LiDAR scans against the static map.

**Algorithm:**
1. Raw odom provides fast pose updates (~50 Hz).
2. Every 0.5 s, a **correlative scan matcher** searches a small window (±0.6 m, ±0.15 rad) around the odom-predicted pose.
3. For each candidate (dx, dy, dθ), up to 120 sub-sampled scan endpoints are projected into map coordinates and scored by how many hit occupied cells.
4. The best correction is **EMA-filtered** (`α = 0.4`) to prevent single-match teleportation.
5. Per-step clamp (0.15 m, 0.05 rad) and total correction clamp (1.0 m) prevent runaway drift.

**Publishes:**
- `/robot_pose` — corrected pose in the map frame
- `/robot_pose_gt` — raw odom-only pose (for comparison)

### 5.5 Human Mover (`move_humans.py`)

Animates **5 human models** in Gazebo along pre-defined waypoint loops via the `SetEntityPose` service. Each human's motion pattern is designed to trigger a specific social navigation case:

| Human | Speed (m/s) | Behaviour | Social Case |
|-------|-------------|-----------|-------------|
| `human_moving_1` | 0.03 | Near-stationary blocker at (1, −2) | Case 1 |
| `human_moving_2` | 0.35 | North-south oscillation (head-on path) | Case 2 |
| `human_moving_3` | 0.45 | East-west crossing at y ≈ 1 | Case 3 |
| `human_moving_4` | 0.22 | NE diagonal, slower than rover | Case 4a |
| `human_moving_5` | 0.58 | Northward from y = −12, faster than rover | Case 5 |

**Publishes:**
- `/human_ground_truth` (`PoseArray`) — world positions (for visualization)
- `/detected_humans` (`PoseArray`) — same positions (consumed by planners)
- `/human_velocities` (`PoseArray`) — instantaneous velocity vectors (vx, vy)

### 5.6 Human Detector (Red) (`human_detector_red.py`)

Vision-based human detection using **HSV red-colour thresholding** on 4 camera feeds.

**Detection Pipeline (per camera frame):**
1. Convert BGR → HSV colour space.
2. Threshold for RED hue (H ≈ 0–10 and H ≈ 170–180, high S, high V).
3. Morphological open/close to clean noise.
4. Find contours, filter by minimum area (40 px²).
5. Estimate range from contour size using a **pinhole camera model**.
6. Estimate bearing from centroid column position.
7. Convert (bearing, range) → world-frame (x, y) using the robot's pose.
8. Track detections across frames with a **Kalman filter**.

**Kalman Tracker (`KalmanTrackedTarget`):**
- State vector: `[x, y, vx, vy]`; full predict + update cycle.
- Measurement matrix observes position only; velocity is estimated by the filter.
- Rejects implausible speed jumps (> 1.5 m/s) to handle sensor noise.
- Optional **LiDAR fusion**: when a LiDAR ray aligns with a camera-detection bearing (within ±0.15 rad), the more-accurate LiDAR range replaces the pinhole estimate.

### 5.7 Camera View 360 (`camera_view_360.py`)

Combines the 4 directional camera feeds into a **2×2 panoramic grid** image (front/right/left/back) and publishes on `/camera/panorama`. Uses OpenCV for resizing and compositing; shows placeholder labels when a camera feed is not yet available.

### 5.8 Weighted Grid Module (`weighted_grid.py`)

A pure-Python **edge-weighted graph** on an 8-connected 2D grid. This is the core data structure used by the global planner.

```
edge_cost(A → B) = move_distance(A, B) × base_weight[B] × dynamic_weight[B]
```

- **`base_weight`** — static safety cost from obstacle proximity (set once from `/map`). Computed via obstacle inflation with exponential falloff. Lethal zone (< 0.5 m from obstacle) treated as impassable.
- **`dynamic_weight`** — human/threat cost (updated by the social nav planner or local planner). Starts at 1.0, increased in zones around detected humans.

**Key methods:**

| Method | Purpose |
|---|---|
| `from_occupancy_grid(msg)` | Build grid from OccupancyGrid, downsample to planner resolution, inflate obstacles |
| `astar(start, goal)` | A\* search with octile-distance heuristic on the 8-connected grid |
| `update_dynamic_weights(zones)` | Apply human cost zones: list of `(wx, wy, radius, weight)` |
| `simplify_path(grid_path)` | Grid→world conversion, line-of-sight pruning (remove A\* staircase), corner smoothing |
| `to_occupancy_data()` | Export weighted grid as list for RViz OccupancyGrid visualization |

### 5.9 Global Planner (`global_planner.py`)

Owns the `WeightedGrid` and orchestrates A\* path planning.

**Responsibilities:**
1. Build a WeightedGrid from the static `/map` (one-time initialisation).
2. Plan a path when `/goal_pose` is received (or auto-start with default goal).
3. **Reactive replan:** When `/replan_request` arrives, apply `/weight_zones` (human costs) to the grid, then re-run A\* from the **current** robot position.
4. **Proactive monitoring (1 Hz):** Continuously checks whether any detected human — at their current position OR velocity-predicted future position — comes within `proactive_replan_radius` (0.75 m) of **any segment** of the remaining path. If so, builds weight zones and replans before the robot reaches the threat.

**Deviation analysis:** After each replan, the planner computes path overlap with the previous route to distinguish minor deviations from full reroutes.

**Published topics:**
- `/global_path` (`Path`) — planned path for downstream planners
- `/weighted_grid_viz` (`OccupancyGrid`) — cost grid for RViz
- `/path_markers` (`MarkerArray`) — path line, start/goal spheres, distance label
- `/global_planner_status` (`Bool`) — success/failure notification

### 5.10 Local Planner (`local_planner.py`)

A **reactive** path follower with WAIT / REROUTE human-avoidance (used in the legacy `astar_navigation.launch.py`).

**State machine:** `IDLE → FOLLOWING → WAITING/REACHED`

**Decision logic:**

| Situation | Action |
|---|---|
| Human CROSSING the path (perpendicular, will clear quickly) | **WAIT** — stop and resume when clear |
| Human STATIONARY on the path | **REROUTE** — publish weight zones + replan |
| Human APPROACHING the path (collision course) | **REROUTE** |
| Human MOVING AWAY from path | **IGNORE** |
| Wait timeout exceeded (8 s default) | **REROUTE** |

**Progressive speed reduction (3 zones):**
- Approach zone (`social_slowdown_r` .. `threat_dist`): 1.0 → 0.5
- Social zone (`threat_dist` .. `zone_radius`): 0.5 → 0.2
- Intimate zone (< `zone_radius`): 0.2 → 0.1

**Rerouting:** Publishes weight zones along the human's current + predicted trajectory (using `predict_horizon` = 3 s) on `/weight_zones` and sends `/replan_request` to the global planner. A 6 s cooldown prevents replan spam. A 20 s grace period after receiving a replanned path uses a tighter threat threshold so the robot follows the detour without re-triggering.

### 5.11 Social Navigation Planner (`social_nav_planner.py`)

The **primary planner** for the refactored architecture. A case-based social navigation controller with **velocity obstacle (VO) collision cone** computation and rich RViz visualisation. Runs at **10 Hz**.

**Social Cases:**

| Case | Situation | Action |
|------|-----------|--------|
| **Case 1** | Stationary human on path | Publish weight zone (r=1.2 m, w=20×), request A\* reroute, slow/crawl |
| **Case 2** | Human approaching head-on | Slow proportionally to approach speed + distance; stop if very close; publish trajectory weight zones |
| **Case 3** | Human crossing robot's path | Compute collision cone; calculate crossing timing; slow to pass BEHIND the human |
| **Case 4a** | Human ahead, same direction | Do NOT overtake; match/reduce speed within social radius |
| **Case 4b / Case 5** | Human behind, faster | Give way: slow down + lateral angular nudge so faster human can pass; publish trajectory weight zones |

**Classification pipeline (per human, per tick):**
1. Compute distance, relative position, human speed.
2. If speed < 0.05 m/s → **Case 1** (stationary).
3. Compute collision cone (velocity obstacle).
4. Check heading alignment: if `cos(angle) > 0.70` → same direction → **Case 4** or **Case 5**.
5. Check approach rate + opposing velocity component → **Case 2** (head-on).
6. Default → **Case 3** (crossing), with crossing-timing analysis.

**Speed computation stack (layered):**
1. Per-case handler computes an initial target speed.
2. **Proximity safety cap:** Within `SOCIAL_RAD` (1.5 m), max speed is scaled by `distance / SOCIAL_RAD`.
3. **Collision-cone speed cap:** Based on VO entry/exit times.
4. **Personal zone guard:** Within 0.8 m → crawl (0.02 m/s); crossing humans use the tighter `COLL_RAD` (0.5 m).

**Centralised replan trigger (`_check_social_circle_replan`):**
When the rover's current path enters any human's social circle:
- Publish weight zones (current position + predicted trajectory with side-pass bias).
- Request the global planner to replan.
- 3 s cooldown prevents replan spam.
- On social-circle exit, replan is triggered to restore the original shorter path.

**RViz markers published on `/social_nav_markers`:**
- Collision cone fan (red = conflict, yellow = caution, green = clear)
- Velocity arrows (robot = green, human = orange)
- Human social zones (personal = orange ring, social = blue ring)
- Human sphere (orange ball at human position)
- Predicted trajectory lines (5 s ahead)
- Crossing point × (red X where human path crosses robot path)
- Decision banner (floating text above robot: case + action)
- Speed readout

### 5.12 Human Case Controller (`human_case_controller.py`)

Ground-truth human injector for **isolated planning tests**. Replaces both `human_detector_red` AND `move_humans` for single-case testing.

Animates **one human model** in Gazebo via `SetEntityPose` and publishes perfect, noise-free `/detected_humans` + `/human_velocities`.

**Available test cases:**

| Case | Start Position | Velocity (m/s) | Description |
|------|---------------|----------------|-------------|
| `case1` | (0, 0) | (0, 0) | Static blocker |
| `case2` | (3.5, 0) | (−0.4, 0) | Head-on approach |
| `case3` | (0, 4) | (0, −0.3) | Crossing path |
| `case4a` | (−1, 0) | (0.2, 0) | Ahead, same direction |
| `case4b` | (−5, 0) | (0.4, 0) | Behind, faster |
| `case5` | (−7, 0) | (0.6, 0) | Behind, much faster |
| `case6a` | (−1, 3.5) | (0.2, −0.3) | Diagonal from upper-left |
| `case6b` | (−1, −3.5) | (0.2, 0.3) | Diagonal from lower-left |
| `case6c` | (3, 2.5) | (−0.3, −0.15) | Diagonal from upper-right |
| `case6d` | (3, −2.5) | (−0.3, 0.15) | Diagonal from lower-right |

### 5.13 Live Visualization Node (`live_visualization_node.py`)

Publishes **two complementary OccupancyGrid maps** and overlay markers for comparing ground truth vs robot perception in RViz:

| Topic | Type | Content |
|-------|------|---------|
| `/viz/gt_occupancy` | OccupancyGrid | Static map + GT human positions |
| `/viz/perception_occupancy` | OccupancyGrid | Static map + detected human positions |
| `/viz/robot_estimate` | MarkerArray | Cyan arrow (estimated pose), orange spheres (detected humans) |
| `/viz/gt_markers` | MarkerArray | Green arrow (GT pose), magenta spheres (GT humans) |
| `/viz/social_circles` | MarkerArray | Intimate / personal / social zone rings |

Zone radii follow **Hall's proxemic zones:**
- **Intimate:** 0.5 m
- **Personal:** 1.2 m
- **Social:** 3.0 m

### 5.14 A\* Path Planner (Legacy) (`astar_path_planner.py`)

An earlier **monolithic** planner (~1600 lines) that combines grid building, A\* search, path following, human-aware replanning, and multiple candidate path selection in a single node. Features pure-pursuit path following with cross-track error correction. Maintained for backward compatibility with `astar_navigation.launch.py`. The refactored architecture splits these responsibilities across `weighted_grid.py`, `global_planner.py`, and `social_nav_planner.py`.

### 5.15 World-to-Map Converter (`world_to_map.py`)

Converts a Gazebo SDF `.world` file directly into a ROS 2 occupancy map (`.pgm` + `.yaml`) **without running Gazebo or SLAM**.

**Supported geometry:** box (with rotation), cylinder, sphere, cone.
**Skips:** floor, ground_plane, sun, human models, cameras, ceiling lights.

Rasterises each collision geometry onto a numpy grid using vectorised drawing functions (`_draw_box`, `_draw_circle`), then outputs a standard ROS 2 `map_server`-compatible PGM image + YAML descriptor.

---

## 6. Data Flow & Topic Map

| Topic                     | Type              | Publisher                  | Subscriber(s)                                    |
|---------------------------|-------------------|----------------------------|--------------------------------------------------|
| `/odom`                   | `Odometry`        | Gazebo (DiffDrive)         | localization_node, pointcloud_mapper, live_viz    |
| `/scan`                   | `LaserScan`       | Gazebo (LiDAR)             | localization_node, social_nav_planner, pointcloud_mapper |
| `/camera/*/image`         | `Image`           | Gazebo (4 cameras)         | human_detector_red, camera_view_360               |
| `/clock`                  | `Clock`           | Gazebo                     | All nodes (via `use_sim_time`)                    |
| `/map`                    | `OccupancyGrid`   | map_publisher              | global_planner, localization_node, live_viz       |
| `/robot_pose`             | `PoseStamped`     | localization_node          | global_planner, social_nav_planner, live_viz      |
| `/robot_pose_gt`          | `PoseStamped`     | localization_node          | (comparison / debug)                              |
| `/detected_humans`        | `PoseArray`       | move_humans / human_case_ctrl | social_nav_planner, global_planner, live_viz |
| `/human_velocities`       | `PoseArray`       | move_humans / human_case_ctrl | social_nav_planner, global_planner            |
| `/human_ground_truth`     | `PoseArray`       | move_humans                | live_visualization_node                           |
| `/global_path`            | `Path`            | global_planner             | social_nav_planner / local_planner                |
| `/cmd_vel`                | `Twist`           | social_nav_planner         | Gazebo (DiffDrive)                                |
| `/weight_zones`           | `PoseArray`       | social_nav_planner         | global_planner                                    |
| `/replan_request`         | `PoseStamped`     | social_nav_planner         | global_planner                                    |
| `/global_planner_status`  | `Bool`            | global_planner             | (debug)                                           |
| `/social_nav_markers`     | `MarkerArray`     | social_nav_planner         | RViz                                              |
| `/path_markers`           | `MarkerArray`     | global_planner             | RViz                                              |
| `/weighted_grid_viz`      | `OccupancyGrid`   | global_planner             | RViz                                              |
| `/viz/gt_occupancy`       | `OccupancyGrid`   | live_visualization_node    | RViz                                              |
| `/viz/perception_occupancy` | `OccupancyGrid` | live_visualization_node    | RViz                                              |
| `/viz/robot_estimate`     | `MarkerArray`     | live_visualization_node    | RViz                                              |
| `/viz/gt_markers`         | `MarkerArray`     | live_visualization_node    | RViz                                              |
| `/viz/social_circles`     | `MarkerArray`     | live_visualization_node    | RViz                                              |
| `/camera/panorama`        | `Image`           | camera_view_360            | (debug display)                                   |

### Weight Zone Encoding

Weight zones are sent as `PoseArray` messages where each `Pose` encodes one zone:
- `position.x`, `position.y` — zone centre (world metres)
- `position.z` — zone radius (metres)
- `orientation.w` — weight multiplier (higher = stronger avoidance)

---

## 7. Coordinate Frames (TF Tree)

```
  map ──[static: spawn_x, spawn_y, spawn_yaw]──► odom
                                                    │
                                          [dynamic: from /odom]
                                                    │
                                                    ▼
                                             base_footprint
                                                    │
                                          [static: z = 0.08]
                                                    │
                                                    ▼
                                               base_link
                                              /          \
                                   [z = 0.10]             [z = 0.14]
                                      │                       │
                                      ▼                       ▼
                                 lidar_link             camera_link
```

- **`map`** — world-fixed frame, origin at the map's YAML origin.
- **`odom`** — spawns at `(spawn_x, spawn_y)` with `spawn_yaw` rotation. DiffDrive odometry accumulates from here. The `map→odom` transform encodes both translation and rotation so chaining `map→odom→base_footprint` yields the correct world pose.
- **`base_footprint`** — robot centre on the ground plane.
- **`base_link`** — raised 8 cm above footprint (chassis height).
- **`lidar_link`** — LiDAR sensor, 10 cm above `base_link`.
- **`camera_link`** — camera sensor, 14 cm above `base_link`.

The **localization_node** additionally computes a scan-match correction offset that is applied on top of the raw `odom→map` transform for the `/robot_pose` topic. This correction does **not** modify the TF tree itself — it is published as a separate topic so other nodes can choose which pose estimate to use.

---

## 8. Social Navigation Cases

The `social_nav_planner` implements five distinct social navigation behaviours based on human motion classification:

### Case 1 — Stationary Blocker
> Human standing still on the robot's planned path.

- **Detection:** Human speed < 0.05 m/s (`STATIC_THR`).
- **Action:** Publish a weight zone (radius 1.2 m, weight 20×) around the human. When the robot's path enters the human's social circle, request an A\* reroute. The robot crawls if within the personal zone (0.8 m) and slows proportionally within the social zone (1.5 m). If the rerouted path no longer passes through the human's zone, proceed at normal speed.

### Case 2 — Head-On Approach
> Human walking toward the robot from the opposite direction.

- **Detection:** Human approach rate > 0.15 m/s AND velocity component opposing the robot's heading < −0.15 m/s.
- **Action:** Speed scaled inversely with both human velocity and distance. Publish weight zones along the human's predicted trajectory (with pass-side bias) so the global planner can reroute the path to one side. Full stop within personal zone.

### Case 3 — Crossing
> Human crossing the robot's forward path perpendicularly.

- **Detection:** Default case for moving, non-aligned humans.
- **Action:** Compute a **collision cone** (velocity obstacle). Calculate the time for the human's personal zone to fully clear the crossing point. Set speed so the robot arrives **after** the human has passed:
  ```
  v_safe = dist_to_crossing / (t_human_clears + CROSS_BUF)
  ```
  Human velocity scaling: faster crossing human → robot slows more aggressively. A social-radius collision cone provides defense-in-depth even when timing calculations say "safe".

### Case 4a — Ahead, Same Direction (No Overtake)
> Human ahead of the robot, both moving the same way, human is slower.

- **Detection:** Heading similarity > 0.70 (`SAME_DIR_THR`) AND human is in front of the robot.
- **Action:** Within `OVERTAKE_WARN` (2.0 m), match the human's speed × 0.95. Do not attempt to pass.

### Case 5 (& Case 4b) — Fast Human From Behind
> Human behind the robot, moving faster in the same direction.

- **Detection:** Same heading + human behind + relative speed > 0.15 m/s.
- **Action:** Slow down (speed scaled by human velocity: faster human → slower rover) + apply lateral angular nudge (0.25–0.30 rad) so the faster human can pass. Publish weight zones along the human's trajectory for potential global reroute.

### Priority Ordering

When multiple humans are present, the planner picks the highest-priority action:

```
EMERGENCY (< 0.3 m) > Case 1 > Case 2 = Case 5 > Case 3 (conflict)
    = Case 4b > Case 4a > Case 3 (safe) > NONE
```

---

## 9. Planning Architecture — How A\* Rerouting Works

```
 1. /map received → global_planner builds WeightedGrid (one time)
                    All traversable edges start with EQUAL weight (1.0)
                    Obstacle inflation sets base_weight near walls

 2. Goal received → A* runs on grid → /global_path published

 3. During navigation, two replan triggers exist:

    ┌─────────────────────────────────────────────────────────┐
    │  PROACTIVE  (global_planner, 1 Hz)                      │
    │  • Scans entire remaining path vs all detected humans   │
    │  • Checks current + velocity-predicted positions        │
    │  • If human < 0.75 m from any path SEGMENT              │
    │    → build weight zones → A* replan                     │
    │  • 10 s cooldown between proactive replans              │
    └─────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────┐
    │  REACTIVE  (social_nav_planner, per-tick)                │
    │  • When rover enters a human's social circle AND        │
    │    the current path passes through it                   │
    │  • Publishes /weight_zones + /replan_request            │
    │  • 3 s cooldown; also replans on social-circle EXIT     │
    └─────────────────────────────────────────────────────────┘

 4. global_planner receives /weight_zones:
    • Resets dynamic_weights to 1.0
    • Applies each zone with radial falloff:
        cell_weight = 1.0 + (multiplier - 1.0) × (1 - dist/radius)

 5. global_planner receives /replan_request:
    • Runs A* from CURRENT robot position (not original start)
    • A* naturally avoids high-cost zones → detour path
    • Deviation analysis: checks overlap with old path
    • Publishes new /global_path + markers

 6. social_nav_planner receives new /global_path → follows it
    with case-based speed control
```

---

## 10. Launch Files

| Launch File | Description | Key Nodes |
|---|---|---|
| `navigation.launch.py` | **Full social nav stack** in `large_messy_room` with 5 moving humans | Gazebo, 9 bridges, map_publisher, pointcloud_mapper, camera_view_360, localization_node, move_humans, global_planner, social_nav_planner, live_visualization_node |
| `social_nav_cases.launch.py` | **Isolated test cases** in `test_case_arena`. One human, no perception. Configurable via `case` argument | Gazebo, 5 bridges, map_publisher, pointcloud_mapper, localization_node, human_case_controller, global_planner, social_nav_planner, live_visualization_node |
| `astar_navigation.launch.py` | Legacy A\* planner with local_planner | Gazebo, bridges, map_publisher, pointcloud_mapper, localization_node, move_humans, global_planner, local_planner |
| `test_scenario.launch.py` | Multi-scenario test runner (varies worlds, spawn points) | Similar to social_nav_cases |
| `demo.launch.py` | Basic simulation with human animation | Gazebo, bridges, move_humans, pointcloud_mapper |
| `world_to_map.launch.py` | Convert .world → map files offline | world_to_map |

---

## 11. Gazebo Worlds

| World File | Description | Size |
|---|---|---|
| `large_messy_room.world` | Primary test environment — room with furniture, walls, obstacles, 5 human models | 25 × 25 m |
| `test_case_arena.world` | Small symmetric arena for isolated social nav case tests | ~10 × 10 m |
| `test_open_room.world` | Open room scenario | Variable |
| `test_corridor_blocked.world` | Narrow corridor with blockage | Variable |
| `test_doorway.world` | Doorway passage scenario | Variable |
| `test_l_corridor.world` | L-shaped corridor | Variable |
| `indoor_with_humans.world` | Indoor environment with static humans | Variable |
| `messy_road.world` | Outdoor road with obstacles | Variable |
| `iiit_messy_road.world` | IIIT Hyderabad campus road variant | Variable |
| `iiit_hyderabad.world` | IIIT Hyderabad campus simulation | Variable |
| `institute_city.world` | City-scale institute simulation | Variable |
| `complex_grid_map.world` | Grid-based test map | Variable |
| `custom_map.world` | User-customisable world | Variable |

---

## 12. Quick Start

### Prerequisites

```bash
# Install ROS 2 Jazzy + Gazebo Harmonic + build tools
./setup_dependencies.sh
```

### Step 1 — Generate the occupancy map (one-time)

```bash
# Converts the Gazebo .world file to a PGM/YAML map
./convert_world_to_map.sh
```

### Step 2 — Build the workspace

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Step 3 — Run the full navigation stack

```bash
# Default goal (5, 5):
./run_navigation.sh

# Custom goal:
./run_navigation.sh 8.0 3.0
```

### Step 4 — Open RViz (separate terminal)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2 -d scan_project/rviz_config.rviz
```

### Run isolated test cases

```bash
# Test case 1 (static blocker):
./run_test_case.sh case1

# Test case 3 (crossing with collision cone):
./run_test_case.sh case3

# All available: case1, case2, case3, case4a, case4b, case5, case6a-d
```

---

## 13. Shell Scripts Reference

| Script | Purpose |
|---|---|
| `setup_dependencies.sh` | Install ROS 2 Jazzy Desktop, Gazebo Harmonic, colcon build tools |
| `run_simulation.sh` | Basic Gazebo + pointcloud mapping demo |
| `run_navigation.sh` | Full social navigation (global + social planner, 5 humans) |
| `run_test_case.sh <case>` | Isolated social nav case test (`case1` … `case6d`) |
| `run_test_scenario.sh` | Multi-scenario test with different worlds |
| `run_astar_navigation.sh` | Legacy A\* planner launch |
| `run_astar_with_mapping.sh` | Legacy A\* with live mapping |
| `convert_world_to_map.sh` | Convert `.world` → `.pgm` + `.yaml` map files |

All scripts automatically:
1. Source ROS 2 Jazzy (`/opt/ros/jazzy/setup.bash`).
2. Copy the package to `~/ros2_ws/src/`.
3. Run `colcon build --symlink-install`.
4. Source the workspace.
5. Launch the appropriate launch file.

---

## 14. Key Parameters & Constants

### 14.1 Social Navigation Constants (`social_nav_planner.py`)

| Constant | Value | Unit | Meaning |
|---|---|---|---|
| `STATIC_THR` | 0.05 | m/s | Human speed below this → Case 1 (stationary) |
| `COLL_RAD` | 0.50 | m | Combined robot + human collision radius |
| `SOCIAL_RAD` | 1.50 | m | Social zone outer ring |
| `PERSONAL_RAD` | 0.80 | m | Personal zone inner ring |
| `SAME_DIR_THR` | 0.70 | cos | Heading similarity for "same direction" (~45°) |
| `APPROACH_THR` | 0.15 | m/s | Minimum approach rate for head-on classification |
| `CROSS_BUF` | 2.50 | s | Safety buffer after human clears crossing |
| `MIN_SPD` | 0.02 | m/s | Minimum forward speed |
| `DEF_MAX_SPD` | 0.30 | m/s | Default robot max speed |
| `DEF_ANG_SPD` | 0.30 | rad/s | Default angular speed |
| `GIVE_WAY_DIST` | 1.50 | m | Trigger give-way for faster human from behind |
| `OVERTAKE_WARN` | 2.00 | m | No-overtake zone when following ahead human |
| `WP_TOL` | 0.30 | m | Waypoint reached tolerance |
| `EMERG_DIST` | 0.35 | m | LiDAR emergency stop distance |
| `EMERG_ARC` | 0.52 | rad | Emergency stop angular window (±30°) |

### 14.2 Global Planner Parameters

| Parameter | Default | Unit | Meaning |
|---|---|---|---|
| `planner_resolution` | 0.2 | m | Grid cell size for A\* |
| `inflation_radius` | 0.35 | m | Obstacle inflation for base weights |
| `proactive_replan_radius` | 0.75 | m | Human-to-path distance for proactive replan |
| `proactive_zone_radius` | 0.80 | m | Weight zone radius for proactive replanning |
| `proactive_weight` | 20.0 | × | Weight multiplier for human zones |
| `proactive_cooldown` | 10.0 | s | Minimum time between proactive replans |
| `predict_horizon` | 3.0 | s | How far ahead to predict human positions |

### 14.3 Localization Parameters

| Parameter | Default | Test Arena | Unit | Meaning |
|---|---|---|---|---|
| `correction_interval` | 0.50 | 0.50 | s | Time between scan-match corrections |
| `search_xy` | 0.60 | 0.15 | m | Scan-match search half-window (position) |
| `search_yaw` | 0.15 | 0.08 | rad | Scan-match search half-window (heading) |
| `correction_alpha` | 0.40 | 0.30 | — | EMA blend factor for corrections |
| `xy_step` | 0.05 | 0.05 | m | Search grid resolution |
| `yaw_step` | 0.02 | 0.02 | rad | Yaw search resolution |
| `max_scan_range` | 6.0 | 6.0 | m | Ignore rays beyond this |

### 14.4 Local Planner Parameters

| Parameter | Default | Unit | Meaning |
|---|---|---|---|
| `linear_speed` | 0.08 | m/s | Path-following linear speed |
| `angular_speed` | 0.12 | rad/s | Path-following angular speed |
| `human_threat_dist` | 3.0 | m | Distance to start threat analysis |
| `human_zone_radius` | 0.8 | m | Weight zone radius for rerouting |
| `weight_multiplier` | 10.0 | × | Cost increase for human zones |
| `max_wait_time` | 8.0 | s | Max seconds to wait for crossing human |
| `social_slowdown_radius` | 5.0 | m | Start slowing before social circle |

---

## 15. Troubleshooting

| Problem | Solution |
|---|---|
| **"Waiting for /map…"** | Ensure `map_publisher` is running and the map YAML/PGM files exist in `~/ros2_maps/`. Generate with `./convert_world_to_map.sh`. |
| **Robot doesn't move** | Check `/cmd_vel` is being published: `ros2 topic echo /cmd_vel`. Verify the cmd_vel bridge is running. |
| **Robot spins in place** | Localization may have a bad initial correction. Check `/robot_pose`. Ensure `spawn_x`, `spawn_y`, `spawn_yaw` match the world file. |
| **No humans visible** | Check `/detected_humans`: `ros2 topic echo /detected_humans`. Ensure `move_humans` or `human_case_controller` is launched. |
| **A\* finds no path** | Goal may be inside an obstacle. Check `/weighted_grid_viz` in RViz. Try adjusting `inflation_radius`. |
| **LiDAR overlay drifts** | The `odom→map` TF uses the spawn pose. Verify `spawn_x/y/yaw` parameters match the robot's actual world-file position. |
| **Scan-match teleportation** | Lower `correction_alpha` (e.g., 0.2) for smoother corrections, or reduce `search_xy`/`search_yaw`. |
| **Gazebo set_pose fails** | Verify the world name in the bridge topic matches: `/world/<world_name>/set_pose`. |
| **`rclpy` context error on shutdown** | Normal during Ctrl+C. The node was interrupted during a spin wait; the error is harmless. |

---

*Last updated: March 7, 2026*

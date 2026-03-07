# Social Navigation for Mobile Robots — IRPP Project

> **ROS 2 Jazzy · Gazebo Harmonic · Ubuntu 24.04 · Python 3.12**

A complete socially-aware robot navigation system that detects humans
through vision, localises via scan-matched odometry, plans globally
with A\*, and applies four distinct social behaviour cases in real-time.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [System Architecture](#2-system-architecture)
3. [Software Dependencies](#3-software-dependencies)
4. [Directory Structure](#4-directory-structure)
5. [Perception Pipeline — Human Detection](#5-perception-pipeline--human-detection)
6. [Localisation Pipeline — Scan-Matched Odometry](#6-localisation-pipeline--scan-matched-odometry)
7. [Planning Pipeline](#7-planning-pipeline)
8. [Social Navigation — The Four Cases](#8-social-navigation--the-four-cases)
9. [Test Framework](#9-test-framework)
10. [RViz Visualisation](#10-rviz-visualisation)
11. [Quick Start](#11-quick-start)
12. [Shell Scripts Reference](#12-shell-scripts-reference)
13. [Troubleshooting](#13-troubleshooting)
14. [Node Reference](#14-node-reference)
15. [Key Parameters & Constants](#15-key-parameters--constants)

---

## 1. System Overview

The system drives a TurtleBot3-Rover through Gazebo worlds populated
with walking humans.  Three software layers cooperate:

| Layer            | Responsibility                                                  |
|------------------|-----------------------------------------------------------------|
| **Perception**   | Detect humans from 4 RGB cameras, estimate world position & velocity |
| **Localisation** | Correct differential-drive odometry with LiDAR scan-matching    |
| **Planning**     | Global A\* path, local path following, social behaviour rules   |

The planner classifies every detected human into one of **four social
navigation cases** and adjusts robot speed, heading, or route
accordingly — never entering a human's personal zone (0.8 m) and
reducing speed inside the social zone (1.5 m).

```
                ┌──────────────────────────────────────────────────────┐
                │                   Gazebo Simulation                  │
                │  TurtleBot3-Rover  ·  Human Models  ·  Environment  │
                └───────┬─────────────────┬──────────────────┬────────┘
                        │ /odom           │ /camera/*        │ /scan
                ┌───────▼──────┐  ┌───────▼──────────┐  ┌───▼────────────┐
                │ Localisation │  │   Perception     │  │  Map Publisher  │
                │ (scan-match) │  │ (human_detector) │  │  (static map)  │
                └───────┬──────┘  └───────┬──────────┘  └───┬────────────┘
                        │ /robot_pose     │ /detected_      │ /map
                        │                 │   humans        │
                ┌───────▼─────────────────▼─────────────────▼────────────┐
                │                   Planning Layer                       │
                │  global_planner (A*)  ─►  social_nav_planner / local  │
                │                                                        │
                │  Case 1  ·  Case 2  ·  Case 3  ·  Case 4a  ·  Case 4b│
                └───────────────────────────┬────────────────────────────┘
                                            │ /cmd_vel
                                    ┌───────▼───────┐
                                    │   Robot Base   │
                                    └───────────────┘
```

---

## 2. System Architecture

### 2.1 Topic Flow

| Topic                  | Type              | From → To                                           |
|------------------------|-------------------|------------------------------------------------------|
| `/odom`                | `Odometry`        | Gazebo → localization_node, pointcloud_mapper        |
| `/scan`                | `LaserScan`       | Gazebo → localization_node, social_nav_planner       |
| `/camera/*/image`      | `Image`           | Gazebo → human_detector_red / human_detector_cv      |
| `/robot_pose`          | `PoseStamped`     | localization_node → global_planner, social_nav_planner |
| `/map`                 | `OccupancyGrid`   | map_publisher → global_planner, localization_node    |
| `/detected_humans`     | `PoseArray`       | human_detector_red → social_nav_planner              |
| `/human_velocities`    | `PoseArray`       | human_detector_red → social_nav_planner              |
| `/global_path`         | `Path`            | global_planner → social_nav_planner / local_planner  |
| `/cmd_vel`             | `Twist`           | social_nav_planner → Gazebo                          |
| `/weight_zones`        | `PoseArray`       | social_nav_planner → global_planner                  |
| `/replan_request`      | `PoseStamped`     | social_nav_planner → global_planner                  |
| `/social_nav_markers`  | `MarkerArray`     | social_nav_planner → RViz                            |

### 2.2 Coordinate Frames

```
  map ──(static)──► odom ──(dynamic)──► base_footprint ──► sensors
        (localization corrects odom→map offset)
```

- **map** — world-fixed frame, origin at map YAML origin
- **odom** — starts at robot spawn, drifts over time
- **base_footprint** — robot centre on the ground plane

---

## 3. Software Dependencies

### 3.1 Operating System & Middleware

| Component        | Version / Details             |
|------------------|-------------------------------|
| Ubuntu           | 24.04 LTS (Noble Numbat)      |
| ROS 2            | Jazzy Jalisco                 |
| Gazebo Sim       | Harmonic                      |
| Python           | 3.12                          |
| Colcon           | `colcon-common-extensions`    |

### 3.2 ROS 2 Packages

```bash
sudo apt install \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-interfaces \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-visualization-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-rviz2
```

### 3.3 Python Libraries

```bash
pip install numpy opencv-python pyyaml Pillow defusedxml lxml
```

| Library       | Usage                                               |
|---------------|-----------------------------------------------------|
| `numpy`       | Scan matching, occupancy grids, image processing    |
| `opencv-python` | HSV thresholding, contour detection, camera I/O   |
| `pyyaml`      | Map YAML loading                                    |
| `Pillow`      | PGM/PNG map image I/O                               |
| `defusedxml`  | Safe SDF world file parsing                         |
| `lxml`        | SDF geometry extraction                             |

### 3.4 Gazebo Models

The TurtleBot3-Rover model and human models (red-cylinder humans) are
included in the `worlds/` directory as inline SDF.  No external model
downloads are required.

---

## 4. Directory Structure

```
irpp_proj_git/
├── README.md                          ← this file
├── run_test_case.sh                   ← Run social nav test cases (case1–case4b)
├── run_astar_navigation.sh            ← Full A* demo with perception
├── run_astar_with_mapping.sh          ← A* demo with live mapping
├── run_navigation.sh                  ← Basic local-planner navigation
├── run_simulation.sh                  ← Gazebo-only (no planning)
├── run_test_scenario.sh               ← Alternate test runner
├── convert_world_to_map.sh            ← Generate map from .world file
├── setup_dependencies.sh              ← Install all apt/pip dependencies
│
└── scan_project/human_models/ros_humans_ros2/
    ├── package.xml                    ← ROS 2 package manifest
    ├── setup.py                       ← Python package setup (22 console_scripts)
    ├── setup.cfg                      ← Colcon build config
    │
    ├── ros_humans_ros2/               ← Python source files
    │   ├── __init__.py
    │   │
    │   │  ── PERCEPTION ──
    │   ├── human_detector_red.py      ← HSV red-blob detection + Kalman tracker
    │   ├── human_detector_cv.py       ← Dual-colour (white body + red head) detector
    │   ├── camera_view_360.py         ← 360° panorama from 4 cameras
    │   │
    │   │  ── LOCALISATION ──
    │   ├── localization_node.py       ← Scan-matched odometry correction
    │   │
    │   │  ── MAP & GRID ──
    │   ├── map_publisher.py           ← Loads and publishes static OccupancyGrid
    │   ├── pointcloud_mapper.py       ← LiDAR → accumulated point cloud + TF
    │   ├── weighted_grid.py           ← 8-connected A* grid with dynamic weights
    │   ├── world_to_map.py            ← SDF → PGM/YAML offline conversion
    │   │
    │   │  ── PLANNING ──
    │   ├── global_planner.py          ← A* global path with weight zones
    │   ├── astar_path_planner.py      ← Standalone A* with human-inflated costmap
    │   ├── local_planner.py           ← Reactive follow + WAIT/REROUTE logic
    │   ├── social_nav_planner.py      ← Social behaviour (4 cases) + RViz markers
    │   │
    │   │  ── HUMAN ANIMATION ──
    │   ├── human_case_controller.py   ← Ground-truth human injector (test cases)
    │   ├── move_humans.py             ← Waypoint-based human mover (demo)
    │   ├── publish_human_pose.py      ← TF → PoseArray publisher
    │   │
    │   │  ── SCENARIO-SPECIFIC ──
    │   ├── messy_road_mover.py        ← Entity mover for messy_road world
    │   ├── messy_road_pose_publisher.py ← Pose publisher for messy_road entities
    │   ├── iiit_messy_road_mover.py   ← Entity mover for IIIT campus world
    │   ├── rover_explorer.py          ← Autonomous LiDAR exploration
    │   │
    │   │  ── VISUALISATION ──
    │   └── live_visualization_node.py ← Side-by-side GT vs perception grids
    │
    ├── launch/                        ← Launch files
    │   ├── social_nav_cases.launch.py ← Social nav test cases (main test launcher)
    │   ├── astar_navigation.launch.py ← Full A* navigation demo
    │   ├── navigation.launch.py       ← Local-planner navigation
    │   ├── demo.launch.py             ← Gazebo + basic bridges
    │   ├── messy_road.launch.py       ← Messy road scenario
    │   ├── iiit_messy_road.launch.py  ← IIIT Hyderabad campus scenario
    │   ├── world_to_map.launch.py     ← Offline map generation
    │   └── test_scenario.launch.py    ← Alternate test scenario launcher
    │
    ├── worlds/                        ← Gazebo SDF world files
    │   ├── test_case_arena.world      ← 13×9 m rectangular arena (test cases)
    │   ├── messy_road.world           ← Urban road with humans + vehicles
    │   ├── iiit_messy_road.world      ← IIIT Hyderabad messy road
    │   ├── iiit_hyderabad.world       ← IIIT campus environment
    │   ├── indoor_with_humans.world   ← Indoor scene with human models
    │   ├── custom_map.world           ← Custom obstacle layout
    │   ├── complex_grid_map.world     ← Grid-based obstacles
    │   ├── large_messy_room.world     ← Large room scenario
    │   ├── institute_city.world       ← City-scale environment
    │   ├── test_corridor_blocked.world
    │   ├── test_doorway.world
    │   ├── test_l_corridor.world
    │   └── test_open_room.world
    │
    └── resource/                      ← Ament resource index
```

---

## 5. Perception Pipeline — Human Detection

### 5.1 Primary Detector: `human_detector_red.py`

This is the main perception node used in full-demo mode.  It processes
images from **four cameras** (front, right, back, left) mounted on the
TurtleBot to achieve 360° coverage.

**Pipeline:**

```
  Camera Image (RGB)
       │
       ▼
  ┌──────────────┐
  │  BGR → HSV   │   Convert colour space
  └──────┬───────┘
         ▼
  ┌──────────────────┐
  │  HSV Threshold   │   Isolate RED pixels
  │  H: 0–10, 160–180│   Two hue ranges for red wrap-around
  │  S: 80–255       │
  │  V: 80–255       │
  └──────┬───────────┘
         ▼
  ┌──────────────────┐
  │ Morphological Ops│   erode → dilate to remove noise
  └──────┬───────────┘
         ▼
  ┌──────────────────┐
  │ Contour Finding  │   cv2.findContours → filter by area
  └──────┬───────────┘
         ▼
  ┌──────────────────────────┐
  │ Pinhole Projection      │   pixel (u, v) + known human height
  │                          │   → range d, bearing θ in robot frame
  │  d = (H_real × f_y)     │
  │      / (v - c_y)        │
  │  θ = atan2(u - c_x, f_x)│   + camera offset angle
  └──────┬───────────────────┘
         ▼
  ┌──────────────────────────┐
  │ Coordinate Transform    │   robot-local (d, θ) → map frame (x, y)
  │  x = robot_x + d·cos(θ + robot_yaw)
  │  y = robot_y + d·sin(θ + robot_yaw)
  └──────┬───────────────────┘
         ▼
  ┌──────────────────────────┐
  │ Kalman Filter Tracker   │   Match detections to existing tracks
  │                          │   State: [x, y, vx, vy]
  │  • Hungarian assignment  │   Estimate velocity over time
  │  • ID persistence        │
  └──────┬───────────────────┘
         ▼
  /detected_humans (PoseArray)     ← position (x, y) per human
  /human_velocities (PoseArray)    ← velocity (vx, vy) per human
```

### 5.2 Alternative Detector: `human_detector_cv.py`

Uses a dual-colour model: detects the **white cylindrical body** and
**red spherical head** separately, then requires both to be spatially
co-located for a confirmed human detection.  More robust to false
positives but computationally heavier.

### 5.3 Panoramic View: `camera_view_360.py`

Horizontally stitches the four directional camera feeds into a single
panoramic image published on `/camera_360/image` for debugging.

---

## 6. Localisation Pipeline — Scan-Matched Odometry

### Node: `localization_node.py`

**Problem:** Pure differential-drive odometry drifts over time (wheel
slip, encoder noise).

**Solution:** A correlative scan matcher that periodically aligns the
live LiDAR scan against the known static map to compute a correction
offset.

### 6.1 Algorithm

```
  Every 0.5 s:
  ┌────────────────────────────────────────────────────────────────┐
  │  1. Extract scan endpoints in robot-local frame               │
  │     • Subsample to ≤120 rays for speed                        │
  │                                                                │
  │  2. Brute-force search over (dx, dy, dθ) grid                │
  │     • Window: ±search_xy m, ±search_yaw rad                  │
  │     • Step:   xy_step m, yaw_step rad                         │
  │     • For each candidate pose:                                 │
  │       – Transform scan points to map frame                     │
  │       – Count how many land on occupied cells                  │
  │       – Track the highest-scoring (dx, dy, dθ)                │
  │                                                                │
  │  3. Per-step clamp: |dx|, |dy| ≤ 0.15 m, |dθ| ≤ 0.05 rad    │
  │     Prevents a single bad match from teleporting the robot     │
  │                                                                │
  │  4. EMA blend: correction += alpha × best_delta               │
  │     alpha = 0.3 for smooth convergence                         │
  │                                                                │
  │  5. Total clamp: √(corr_dx² + corr_dy²) ≤ 1.0 m              │
  │     Absolute limit prevents runaway accumulation               │
  └────────────────────────────────────────────────────────────────┘
```

### 6.2 Output

- **`/robot_pose`** — corrected pose (map frame)
- **`/robot_pose_gt`** — raw odom-only pose (for comparison / debugging)

### 6.3 Drift Protection

| Guard                | Value    | Purpose                                              |
|----------------------|----------|------------------------------------------------------|
| Per-step XY clamp    | 0.15 m   | No single match can shift > 0.15 m                   |
| Per-step yaw clamp   | 0.05 rad | No single match can rotate > 2.9°                    |
| Total XY clamp       | 1.0 m    | Accumulated correction never exceeds 1 m              |
| EMA alpha            | 0.3      | Smooth blending, 1 bad match contributes only 30%    |
| Odom jump filter     | 0.6 m/s  | Rejects impossible velocity spikes                    |
| Min scan points      | 40       | Skips correction if too few valid rays                |

---

## 7. Planning Pipeline

### 7.1 Map Publisher (`map_publisher.py`)

Loads a pre-generated PGM + YAML occupancy-grid map and publishes it
on `/map` with `TRANSIENT_LOCAL` durability so late-joining nodes
receive it immediately.

Maps can be generated offline from Gazebo `.world` files using:
```bash
./convert_world_to_map.sh <world_file>
```

### 7.2 Weighted Grid (`weighted_grid.py`)

A library module (not a ROS node) that converts an `OccupancyGrid`
into an 8-connected graph with configurable edge weights:

- **Base weight** — 1.0 for free cells, ∞ for occupied
- **Inflation** — cells near obstacles get elevated cost (configurable radius)
- **Dynamic weight zones** — the social planner pushes penalty zones
  around detected humans; the global planner re-routes around them

### 7.3 Global Planner (`global_planner.py`)

Builds a `WeightedGrid` from the static map, runs **A\*** to find the
shortest path from robot pose to goal, and publishes it as a
`nav_msgs/Path`.

**Features:**
- Auto-starts planning on first `/robot_pose` if `auto_start=True`
- Subscribes to `/weight_zones` for dynamic human penalties
- Subscribes to `/replan_request` to reroute when the local/social
  planner detects a blocked path (Case 1)
- Configurable resolution, inflation radius, goal tolerance

### 7.4 Local Planner (`local_planner.py`)

A reactive path-follower that:
1. Follows the global path waypoint by waypoint
2. Classifies nearby humans (crossing / stationary / approaching)
3. Publishes `WAIT` or `REROUTE` decisions
4. Sends `/cmd_vel` and `/weight_zones` / `/replan_request`

Used in `navigation.launch.py`.  For test cases, replaced by the
**Social Navigation Planner**.

### 7.5 A\* Path Planner (`astar_path_planner.py`)

A standalone planner that builds a local costmap from LiDAR +
detected humans, plans up to 3 candidate A\* paths, scores them, and
follows the best one using pure-pursuit steering.

Used in `astar_navigation.launch.py`.

### 7.6 Social Navigation Planner (`social_nav_planner.py`)

The **core node** for socially-aware behaviour.  Subscribes to
ground-truth human positions and velocities, classifies each human
into one of four cases, and computes the appropriate speed and
steering command.

See [Section 8](#8-social-navigation--the-four-cases) for detailed
case descriptions.

---

## 8. Social Navigation — The Four Cases

The social planner maintains three spatial zones around every human:

```
           ┌─────────────────────────────────────┐
           │            Social Zone               │   r = 1.50 m
           │      ┌───────────────────┐           │   — robot slows
           │      │   Personal Zone   │           │     proportionally
           │      │    ┌─────────┐    │           │
           │      │    │  Human  │    │           │   r = 0.80 m
           │      │    │   (·)   │    │           │   — robot must
           │      │    └─────────┘    │           │     NEVER enter
           │      └───────────────────┘           │
           └─────────────────────────────────────┘
                  r = 0.50 m (collision radius)
```

### Classification Logic

For each detected human, the planner computes:

| Quantity      | Formula                                          | Meaning                           |
|---------------|--------------------------------------------------|-----------------------------------|
| `same_dir`    | $\vec{r} \cdot \vec{h}$                          | +1 = same heading, −1 = opposite  |
| `approach`    | $-(\vec{v_h} \cdot \hat{d})$                     | >0 = human approaching robot      |
| `along_rh`    | $\vec{v_h} \cdot \hat{r}$                        | <0 = human opposes robot heading  |
| `ahead_d`     | $\vec{d} \cdot \hat{r}$                          | >0 = human is in front of robot   |

Classification priority:

```
  ┌─ human speed < 0.05 m/s ──────► Case 1 (static)
  │
  ├─ same_dir > 0.70 ─────────────► Case 4 (same direction)
  │   ├─ ahead_d > 0 ─────────────►   4a (ahead)
  │   └─ ahead_d ≤ 0 ─────────────►   4b (behind)
  │
  ├─ approach > 0.15 m/s
  │   AND along_rh < −0.15 ───────► Case 2 (head-on)
  │
  └─ else ─────────────────────────► Case 3 (crossing)
```

### Priority Table

| Priority | Case            | Behaviour                       |
|----------|-----------------|---------------------------------|
| 0        | EMERGENCY       | LiDAR obstacle < 0.35 m → STOP |
| 1        | Case 1          | Static blocker → reroute        |
| 2        | Case 2          | Head-on → slow / stop           |
| 3        | Case 3 conflict | Crossing → SLOW to pass behind  |
| 3        | Case 4b         | Give way to faster human behind |
| 4        | Case 4a         | Follow ahead human (no overtake)|
| 5        | Case 3 safe     | Crossing — computed safe timing |
| 9        | NONE            | No threat — full speed          |

---

### Case 1 — Static Human Blocking the Path

**Scenario:** A stationary human stands directly on the robot's planned
path (speed < 0.05 m/s).

**Behaviour:**
1. Robot stops (speed → 0).
2. Publishes a **weight zone** (penalty radius 1.2 m, weight 20.0)
   centred on the human to `/weight_zones`.
3. Sends a `/replan_request` to the global planner.
4. Global planner re-runs A\* with the elevated cost zone, producing
   a path that goes around the human.
5. Replan has an 8 s cooldown to avoid thrashing.

```
    ┌────────────────────────────────────┐
    │          Original Path             │
    │   Robot ═══════ X ═══════► Goal    │
    │                 ↑                  │
    │              Human (static)        │
    │                                    │
    │   Robot ═══╗               ╔═► Goal│
    │            ║  Reroute      ║       │
    │            ╚═══════════════╝       │
    └────────────────────────────────────┘
```

---

### Case 2 — Human Approaching Head-On

**Scenario:** A moving human walks toward the robot along the robot's
heading direction (approach rate > 0.15 m/s AND velocity opposes robot
heading).

**Behaviour:**
- **Outside social zone** (> 1.5 m): Robot runs at 60% speed.
- **Inside social zone** (0.8 – 1.5 m): Speed scales linearly from
  60% → near-zero based on distance.
- **Inside personal zone** (< 0.8 m): Robot STOPS.
- **After human passes:** Robot resumes full speed.

```
  d > 1.5m:    spd = 0.60 × max_speed
  0.8 < d < 1.5m:   spd ∝ (d − 0.80) / 0.70
  d < 0.8m:    spd = 0 (STOP)
```

---

### Case 3 — Human Crossing the Robot's Path

**Scenario:** A human walks perpendicular (or at an angle) across the
robot's forward path, creating a potential collision at the crossing
point.

This is the most complex case.  The planner uses **two independent
safety checks:**

#### 3A. Timing-Based Speed Control

Compute where the human's trajectory intersects the robot's forward
path (ray–ray intersection):

$$\text{cross} = \text{robot\_pos} + t \cdot \hat{r}$$

where $t$ (metres) is the distance from robot to crossing along its
heading, and $s$ (seconds) is the time for the human to reach the
crossing.

The robot slows so it arrives **after** the human has cleared the
crossing by `PERSONAL_RAD` plus a `CROSS_BUF` safety buffer:

$$v_{\text{safe}} = \frac{d_{\text{cross}}}{t_{\text{human\_clears}} + \text{CROSS\_BUF}}$$

where:
- $t_{\text{human\_clears}} = s + \frac{\text{PERSONAL\_RAD}}{v_h}$
- $\text{CROSS\_BUF} = 2.5\text{ s}$

#### 3B. Social-Radius Collision Cone (Defense-in-Depth)

If the timing calculation says "safe" ($v_{\text{safe}} \geq 0.98 \times v_{\max}$), the planner
performs a **second check** using a collision cone with `SOCIAL_RAD`
(1.5 m) instead of `COLL_RAD` (0.5 m).

This solves the relative velocity quadratic:

$$a\,t^2 + b\,t + c = 0$$

where $a = |\vec{v}_{\text{rel}}|^2$, $b = -2(\vec{p} \cdot \vec{v}_{\text{rel}})$,
$c = |\vec{p}|^2 - R_{\text{social}}^2$.

If the discriminant is ≥ 0 and a future entry time exists ($t_1 > 0$
or $t_2 > 0$), the robot will enter the human's social zone — it
slows to 50% max speed.

This defence-in-depth check is robust to localization drift up to
approximately 2.7 m.

```
    Robot ──────────────────► Goal
                   ×  ← crossing point
                   │
                   │  Human
                   │  moving ↓
                   ▼

    Robot must arrive AFTER human has passed the × by ≥ 0.8 m
```

#### Collision Cone Visualisation

The standard velocity-obstacle collision cone is always computed and
displayed in RViz:

- **Red fan** — current heading is inside the cone (collision predicted)
- **Yellow fan** — outside but close
- **Green fan** — clear

---

### Case 4a — Human Ahead, Same Direction

**Scenario:** A human walks in roughly the same direction as the robot
and is ahead of it (heading similarity > 0.70, ahead_d > 0).

**Behaviour:**
- If distance < 2.0 m: match human speed × 0.95 (never overtake)
- If distance ≥ 2.0 m: full speed (will naturally close gap slowly)

The 0.95 factor ensures the robot never creeps past the human.

---

### Case 4b — Human Behind, Same Direction, Faster

**Scenario:** A human walks in the same direction as the robot but
faster, approaching from behind.

**Behaviour:**
- If distance < 1.5 m and human faster: slow to 40% + lateral nudge
  (angular push ±0.35 rad/s) to yield the lane.
- If distance < 3.0 m and human faster: reduce to 70% speed
  (prepare to give way).
- Otherwise: full speed.

```
    Human (fast) ───────►  Robot (slow) ───────► Goal
         approaching from behind
         Robot nudges sideways to let human pass
```

---

## 9. Test Framework

### 9.1 Components

The test framework bypasses perception entirely and uses ground-truth
data:

| Component                  | Purpose                                             |
|----------------------------|-----------------------------------------------------|
| `human_case_controller.py` | Teleports one human in Gazebo + publishes GT poses  |
| `social_nav_cases.launch.py` | Launches Gazebo + bridges + planning + GT controller |
| `run_test_case.sh`         | One-command runner: build + launch + RViz            |
| `test_case_arena.world`    | 13×9 m rectangular Gazebo world                     |

### 9.2 Test Scenarios

| Case    | Human Start   | Human Velocity    | Robot Start  | Goal       |
|---------|---------------|-------------------|--------------|------------|
| case1   | (0.0, 0.0)    | (0.0, 0.0)        | (−4.0, 0.0)  | (4.5, 0.0) |
| case2   | (3.5, 0.0)    | (−0.40, 0.0)      | (−4.0, 0.0)  | (4.5, 0.0) |
| case3   | (0.0, 4.0)    | (0.0, −0.30)      | (−4.0, 0.0)  | (4.5, 0.0) |
| case4a  | (−1.0, 0.0)   | (0.20, 0.0)       | (−4.0, 0.0)  | (4.5, 0.0) |
| case4b  | (−5.0, 0.0)   | (0.40, 0.0)       | (−4.0, 0.0)  | (4.5, 0.0) |

### 9.3 Case 3 Collision Geometry

Case 3 is designed as an **exact simultaneous collision** at origin:

- Robot at (−4, 0) heading right at 0.30 m/s → reaches (0, 0) in 13.33 s
- Human at (0, 4) heading down at 0.30 m/s → reaches (0, 0) in 13.33 s

If both go at full speed, they arrive at the crossing at the same
instant.  The social planner must detect this and slow the robot to
approximately 0.22 m/s so it arrives ≈ 5 s after the human has
cleared the crossing:

$$v_{\text{safe}} = \frac{4.0}{16.0 + 2.5} = 0.216 \text{ m/s}$$

### 9.4 Running a Test

```bash
cd ~/Videos/irpp_proj_git
./run_test_case.sh case3
```

**What to observe in RViz:**
- Robot starts at (−4, 0) and heads toward goal (4.5, 0)
- Human walks down from (0, 4) toward the crossing at (0, 0)
- Collision cone turns RED when on collision course
- Red × marker appears at crossing point
- Decision banner shows `case3  Crossing — SLOW 0.22m/s`
- Robot slows well before crossing, human passes through first
- After human clears, robot accelerates back to 0.30 m/s

---

## 10. RViz Visualisation

The `social_nav_planner` publishes a rich set of markers on
`/social_nav_markers`:

| Marker               | Type           | Colour         | Description                           |
|----------------------|----------------|----------------|---------------------------------------|
| Social ring          | LINE_STRIP     | Blue (0.3,0.6,1)| 1.5 m radius circle around human     |
| Personal ring        | LINE_STRIP     | Orange         | 0.8 m radius circle around human      |
| Human sphere         | SPHERE         | Orange         | 0.55 m ball at human position          |
| Human velocity arrow | ARROW          | Orange         | 3.5× amplified velocity vector         |
| Human trajectory     | LINE_STRIP     | Orange (faded) | Predicted path 5 s ahead (0.5 s steps) |
| Collision cone       | TRIANGLE_LIST  | Red/Yellow     | Velocity-obstacle fan                  |
| Crossing ×           | LINE_LIST      | Red            | Path intersection point                |
| Robot velocity arrow | ARROW          | Green          | 4× amplified velocity vector           |
| Robot trajectory     | LINE_STRIP     | Green (faded)  | Predicted path 3 s ahead               |
| Decision banner      | TEXT           | Case-dependent | Case name + speed + distance           |
| Human label          | TEXT           | Orange         | Human ID + speed                       |

All text markers auto-expire at 300 ms to prevent ghost text.

The RViz configuration file is at `scan_project/rviz_config.rviz`.

---

## 11. Quick Start

### 11.1 Install Dependencies

```bash
cd ~/Videos/irpp_proj_git
./setup_dependencies.sh
```

### 11.2 Generate Map (One-Time)

```bash
./convert_world_to_map.sh
```

### 11.3 Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Videos/irpp_proj_git/scan_project/human_models/ros_humans_ros2
rm -rf ~/ros2_ws/src/ros_humans_ros2
cp -r . ~/ros2_ws/src/ros_humans_ros2
cd ~/ros2_ws
colcon build --symlink-install --packages-select ros_humans_ros2
source install/setup.bash
```

### 11.4 Run Full A\* Demo (with Perception)

```bash
./run_astar_navigation.sh
```

### 11.5 Run Social Nav Test Cases

```bash
./run_test_case.sh case1    # static blocker → reroute
./run_test_case.sh case2    # head-on approach → slow/stop
./run_test_case.sh case3    # crossing path → slow, pass behind
./run_test_case.sh case4a   # follow ahead human
./run_test_case.sh case4b   # give way to faster human behind
```

---

## 12. Shell Scripts Reference

| Script                       | Description                                              |
|------------------------------|----------------------------------------------------------|
| `run_test_case.sh`           | Build + launch social nav test case + RViz               |
| `run_astar_navigation.sh`    | Full A\* navigation with perception + human detection    |
| `run_astar_with_mapping.sh`  | A\* navigation with live LiDAR mapping                   |
| `run_navigation.sh`          | Local-planner-based navigation                           |
| `run_simulation.sh`          | Gazebo simulation only (no planning/perception)          |
| `run_test_scenario.sh`       | Alternate test scenario runner                           |
| `convert_world_to_map.sh`    | Generate PGM + YAML map from a Gazebo .world file        |
| `setup_dependencies.sh`      | Install all apt and pip dependencies                     |

---

## 13. Troubleshooting

### Localization Drift in Test Arena

**Symptom:** Robot's reported position drifts significantly; planner
says "Crossing — safe" when it should slow down.

**Cause:** The rectangular test arena has limited features for scan
matching.  The correlative matcher may lock onto wrong wall sections.

**Fix (already applied):**
- `search_xy` reduced to 0.15 m (tighter search window)
- Per-step clamp: 0.15 m max correction per cycle
- Total correction clamp: 1.0 m absolute limit
- `correction_alpha` lowered to 0.3 for smoother blending

### Gazebo Shutdown Tracebacks

**Symptom:** `RCLError: failed to initialize wait set` on Ctrl+C.

**Cause:** ROS 2 nodes attempt shutdown operations after the context
is destroyed.  This is cosmetic — all nodes have `try/except` guards
around `rclpy.shutdown()`.

### World File Not Found

**Symptom:** `launch error: world file does not exist`

**Fix:** Ensure the package is built with `--symlink-install` and
`setup.py` includes the `worlds/` directory in `data_files`.

### Human Jumping in Gazebo

**Symptom:** Human model teleports vertically or vibrates.

**Fix:** In the `.world` SDF, ensure the human model has
`<static>true</static>` and `z=0.0` for the model root pose (the
visual offset is handled by the link, not the model pose).

### Map Not Loading

**Fix:** Generate the map first:
```bash
./convert_world_to_map.sh
```
Maps are saved to `~/ros2_maps/<world_name>.yaml` + `.pgm`.

---

## 14. Node Reference

| Executable                | Node Name              | Role                                          |
|---------------------------|------------------------|-----------------------------------------------|
| `human_detector_red`      | human_detector_red     | HSV red-blob detection + Kalman tracking       |
| `human_detector_cv`       | human_detector_cv      | Dual-colour (body+head) detection              |
| `camera_view_360`         | camera_view_360        | 360° panorama stitching                        |
| `localization_node`       | localization_node      | Scan-matched odometry correction               |
| `map_publisher`           | map_publisher          | Static map loading and publishing              |
| `pointcloud_mapper`       | pointcloud_mapper      | LiDAR → point cloud + TF broadcasts           |
| `global_planner`          | global_planner         | A\* global path planning with weight zones     |
| `local_planner`           | local_planner          | Reactive path following + WAIT/REROUTE         |
| `astar_path_planner`      | astar_path_planner     | Standalone A\* with human-inflated costmap     |
| `social_nav_planner`      | social_nav_planner     | 4-case social behaviour + RViz markers         |
| `human_case_controller`   | human_case_controller  | Ground-truth human injector (test cases)       |
| `move_humans`             | move_humans            | Waypoint-based human animation (demo)          |
| `publish_human_pose`      | publish_human_pose     | TF → PoseArray for human models                |
| `world_to_map`            | world_to_map           | Offline SDF → OccupancyGrid conversion         |
| `live_visualization_node` | live_visualization_node| GT vs perception comparison grids              |
| `rover_explorer`          | rover_explorer         | Autonomous LiDAR exploration                   |
| `messy_road_mover`        | messy_road_mover       | Entity animation for messy_road world          |
| `messy_road_pose_publisher` | messy_road_pose_publisher | Pose publisher for messy_road entities      |
| `iiit_messy_road_mover`   | iiit_messy_road_mover  | Entity animation for IIIT campus world         |

---

## 15. Key Parameters & Constants

### 15.1 Social Planner Constants (`social_nav_planner.py`)

| Constant       | Value     | Unit   | Description                                    |
|----------------|-----------|--------|------------------------------------------------|
| `COLL_RAD`     | 0.50      | m      | Physical collision radius (robot + human + margin) |
| `PERSONAL_RAD` | 0.80      | m      | Personal zone — robot must NEVER enter          |
| `SOCIAL_RAD`   | 1.50      | m      | Social zone — robot slows proportionally         |
| `SAME_DIR_THR` | 0.70      | cos    | Heading similarity for "same direction" (~45°)   |
| `APPROACH_THR` | 0.15      | m/s    | Approach rate threshold for Case 2               |
| `CROSS_BUF`    | 2.50      | s      | Safety buffer after human clears crossing        |
| `MIN_SPD`      | 0.02      | m/s    | Minimum forward speed                            |
| `DEF_MAX_SPD`  | 0.30      | m/s    | Default robot maximum speed                      |
| `DEF_ANG_SPD`  | 0.30      | rad/s  | Default angular speed                            |
| `GIVE_WAY_DIST`| 1.50      | m      | Trigger distance for Case 4b                     |
| `OVERTAKE_WARN`| 2.00      | m      | No-overtake zone for Case 4a                     |
| `WP_TOL`       | 0.30      | m      | Waypoint reached tolerance                       |
| `EMERG_DIST`   | 0.35      | m      | LiDAR emergency stop distance                    |
| `EMERG_ARC`    | 0.52      | rad    | Emergency stop angular window (±30°)             |

### 15.2 Localisation Parameters

| Parameter            | Default | Test Arena | Unit   | Description                    |
|----------------------|---------|------------|--------|--------------------------------|
| `search_xy`          | 0.60    | 0.15       | m      | Scan matcher half-window       |
| `search_yaw`         | 0.15    | 0.08       | rad    | Yaw search half-window         |
| `correction_alpha`   | 0.40    | 0.30       | —      | EMA blending factor            |
| `correction_interval`| 0.50    | 0.50       | s      | Time between corrections       |
| `xy_step`            | 0.05    | 0.05       | m      | Search grid resolution         |
| `yaw_step`           | 0.02    | 0.02       | rad    | Yaw search resolution          |
| `max_scan_range`     | 6.0     | 6.0        | m      | Ignore rays beyond this        |

### 15.3 Global Planner Parameters (Test Arena)

| Parameter                  | Value  | Description                           |
|----------------------------|--------|---------------------------------------|
| `planner_resolution`       | 0.2 m  | A\* grid cell size                    |
| `inflation_radius`         | 0.35 m | Obstacle inflation for safety margin  |
| `goal_tolerance_m`         | 0.30 m | Goal reached threshold                |
| `proactive_replan_radius`  | 0.0 m  | Disabled — planner doesn't auto-reroute |
| `proactive_zone_radius`    | 0.80 m | Weight zone radius for Case 1         |
| `proactive_weight`         | 20.0   | Cost multiplier for human zones       |

---

*Generated for the IRPP Social Navigation Project — ROS 2 Jazzy / Gazebo Harmonic*

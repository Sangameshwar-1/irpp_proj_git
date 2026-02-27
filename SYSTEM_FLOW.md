# A* Path Planning System Flow

## System Overview

```
╔═══════════════════════════════════════════════════════════════╗
║                    SHELL SCRIPT LAUNCHER                      ║
║              (run_astar_with_mapping.sh)                      ║
╚═══════════════════════════════════════════════════════════════╝
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │  Step 1: Build ROS2 Workspace           │
        │  - Copy project to ~/ros2_ws/src        │
        │  - colcon build                         │
        │  - Source workspace                     │
        └──────────────────┬──────────────────────┘
                           ▼
        ┌─────────────────────────────────────────┐
        │  Step 2: Convert World to Map           │
        │  - Parse .world file (Gazebo)           │
        │  - Extract walls/obstacles              │
        │  - Generate occupancy grid              │
        │  - Save as .pgm + .yaml                 │
        └──────────────────┬──────────────────────┘
                           ▼
        ┌─────────────────────────────────────────┐
        │  Step 3: Start Map Publisher            │
        │  - Load static map from .yaml           │
        │  - Publish to /map topic                │
        │  - Rate: 1 Hz                           │
        └──────────────────┬──────────────────────┘
                           ▼
        ┌─────────────────────────────────────────┐
        │  Step 4: Launch Gazebo Simulation       │
        │  - Load world file                      │
        │  - Spawn TurtleBot3 rover               │
        │  - Initialize sensors (LiDAR, cameras)  │
        │  - Start ROS-Gazebo bridges             │
        └──────────────────┬──────────────────────┘
                           ▼
        ┌─────────────────────────────────────────┐
        │  Step 5: Start Navigation Nodes         │
        │  - A* Path Planner                      │
        │  - Point Cloud Mapper                   │
        │  - Camera View Combiner                 │
        └──────────────────┬──────────────────────┘
                           ▼
        ┌─────────────────────────────────────────┐
        │  Step 6: Open RViz Visualization        │
        │  - Load configuration                   │
        │  - Display all topics                   │
        └─────────────────────────────────────────┘
```

## Data Flow Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                      GAZEBO SIMULATION                          │
├────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐      ┌──────────────┐      ┌──────────────┐│
│  │  TurtleBot3  │      │    LiDAR     │      │   Cameras    ││
│  │    Rover     │      │   Sensor     │      │   (4x 90°)   ││
│  │              │      │   360° Scan  │      │              ││
│  └──────┬───────┘      └──────┬───────┘      └──────┬───────┘│
│         │                     │                     │         │
│         │ /odom               │ /scan               │ /camera/│
└─────────┼─────────────────────┼─────────────────────┼─────────┘
          │                     │                     │
          ▼                     ▼                     ▼
┌─────────────────────────────────────────────────────────────────┐
│                      ROS-GAZEBO BRIDGES                          │
│  - Clock Bridge          - Odometry Bridge                       │
│  - Pose/TF Bridge        - LaserScan Bridge                      │
│  - Cmd_Vel Bridge        - Camera Bridges                        │
└────────┬────────────────────────┬────────────────────────────────┘
         │                        │
         │                        │
         ▼                        ▼
┌──────────────────┐    ┌──────────────────────────────────┐
│  MAP PUBLISHER   │    │      SENSOR DATA STREAM          │
│  (Static Map)    │    │   /odom, /scan, /camera/*        │
└────────┬─────────┘    └────────────┬─────────────────────┘
         │                           │
         │ /map                      │
         │                           │
         ▼                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                   NAVIGATION & MAPPING NODES                     │
├──────────────────────────────┬───────────────────────────────────┤
│    A* PATH PLANNER           │   POINT CLOUD MAPPER              │
├──────────────────────────────┼───────────────────────────────────┤
│ INPUT:                       │ INPUT:                            │
│  • /scan (LaserScan)         │  • /scan (LaserScan)              │
│  • /odom (Odometry)          │  • /odom (Odometry)               │
│  • /goal_pose (PoseStamped)  │                                   │
│                              │                                   │
│ PROCESSING:                  │ PROCESSING:                       │
│  1. Update occupancy grid    │  1. Convert scan to point cloud   │
│  2. Run A* algorithm         │  2. Transform to map frame        │
│  3. Simplify path            │  3. Accumulate points             │
│  4. Follow waypoints         │  4. Build occupancy grid          │
│  5. Generate cmd_vel         │  5. Ray tracing for free space    │
│                              │  6. Publish TF transforms         │
│ OUTPUT:                      │                                   │
│  • /cmd_vel (Twist)          │ OUTPUT:                           │
│  • /planned_path (Path)      │  • /scan_pointcloud (PointCloud2) │
│  • /path_markers (Marker[])  │  • /map_pointcloud (PointCloud2)  │
│  • /local_costmap (OccGrid)  │  • /map (OccupancyGrid)           │
│                              │  • TF: map->odom->base_footprint  │
└──────────────────────────────┴───────────────────────────────────┘
                           │
                           │ All topics
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                      RVIZ VISUALIZATION                          │
├─────────────────────────────────────────────────────────────────┤
│  DISPLAYS:                                                       │
│  ✓ Robot Model (TF tree)                                        │
│  ✓ Static Map (/map)                                            │
│  ✓ Dynamic Costmap (/local_costmap)                             │
│  ✓ Planned Path (/planned_path)                                 │
│  ✓ Path Markers (/path_markers)                                 │
│  ✓ Current Scan (/scan_pointcloud)                              │
│  ✓ Accumulated Map (/map_pointcloud)                            │
│  ✓ Camera Views (/camera/*/image)                               │
└─────────────────────────────────────────────────────────────────┘
```

## A* Algorithm Flow

```
START
  │
  ▼
┌────────────────────────────┐
│ Receive Goal Pose          │
│ from /goal_pose topic      │
└────────────┬───────────────┘
             │
             ▼
┌────────────────────────────┐
│ Update Occupancy Grid      │
│ from LiDAR Scan            │
│ • Mark obstacles           │
│ • Inflate safety margin    │
└────────────┬───────────────┘
             │
             ▼
┌────────────────────────────┐
│ Convert World → Grid Coords│
│ start_grid = (gx, gy)      │
│ goal_grid = (gx', gy')     │
└────────────┬───────────────┘
             │
             ▼
┌────────────────────────────┐
│ Initialize A* Algorithm    │
│ • open_set = [start]       │
│ • g_score[start] = 0       │
│ • f_score[start] = h(start)│
└────────────┬───────────────┘
             │
             ▼
      ┌──────────────┐
      │ open_set      │ No
      │ not empty?    ├──────┐
      └──────┬────────┘      │
             │ Yes            │
             ▼                │
┌────────────────────────────┐│
│ Pop node with lowest f     ││
│ current = heappop()        ││
└────────────┬───────────────┘│
             │                │
             ▼                │
      ┌──────────────┐        │
      │ Reached goal?│ Yes    │
      │ dist < 2cells├────┐   │
      └──────┬────────┘    │   │
             │ No          │   │
             ▼             │   │
┌────────────────────────────┐│   │
│ Get 8-Connected Neighbors  ││   │
│ (N, S, E, W, NE, NW, SE, SW)│   │
└────────────┬───────────────┘│   │
             │                │   │
             ▼                │   │
┌────────────────────────────┐│   │
│ For each valid neighbor:   ││   │
│ • Not obstacle             ││   │
│ • Not visited              ││   │
│ • Calculate g, f scores    ││   │
│ • Add to open_set          ││   │
└────────────┬───────────────┘│   │
             │                │   │
             └────────────────┘   │
                                  │
                                  ▼
                    ┌──────────────────────────┐
                    │ Reconstruct Path         │
                    │ • Follow came_from links │
                    │ • Reverse path           │
                    └────────────┬─────────────┘
                                 │
                                 ▼
                    ┌──────────────────────────┐
                    │ Convert Grid → World     │
                    │ path = [(x,y), ...]      │
                    └────────────┬─────────────┘
                                 │
                                 ▼
                    ┌──────────────────────────┐
                    │ Simplify Path            │
                    │ • Remove collinear points│
                    │ • Keep turning points    │
                    └────────────┬─────────────┘
                                 │
                                 ▼
                    ┌──────────────────────────┐
                    │ Publish Path & Markers   │
                    │ • /planned_path          │
                    │ • /path_markers          │
                    └────────────┬─────────────┘
                                 │
                                 ▼
                    ┌──────────────────────────┐
                    │ Start Path Following     │
                    │ state = FOLLOWING        │
                    └──────────────────────────┘
```

## Path Following Control Loop

```
┌─────────────────────────────┐
│  Control Loop (10 Hz)       │
└────────────┬────────────────┘
             │
             ▼
      ┌──────────────┐
      │ Current       │ No
      │ waypoint idx  ├───────► GOAL REACHED
      │ < path length?│          state = IDLE
      └──────┬────────┘
             │ Yes
             ▼
┌────────────────────────────┐
│ Get Current Waypoint       │
│ target = path[idx]         │
└────────────┬───────────────┘
             │
             ▼
┌────────────────────────────┐
│ Calculate Distance & Angle │
│ dx = target.x - current.x  │
│ dy = target.y - current.y  │
│ distance = √(dx² + dy²)    │
│ angle = atan2(dy, dx)      │
└────────────┬───────────────┘
             │
             ▼
      ┌──────────────┐
      │ distance <    │ Yes
      │ tolerance?    ├────► Increment waypoint idx
      └──────┬────────┘      Continue to next
             │ No
             ▼
┌────────────────────────────┐
│ Calculate Angle Error      │
│ angle_diff = target_angle  │
│            - current_yaw   │
│ (normalize to [-π, π])     │
└────────────┬───────────────┘
             │
             ▼
      ┌──────────────┐
      │ |angle_diff| │ Yes
      │ > 0.3 rad?   ├────► Turn in place
      └──────┬────────┘      linear = 0.05
             │ No             angular = ±0.5
             ▼
┌────────────────────────────┐
│ Move Toward Target         │
│ linear = min(0.3, dist*0.5)│
│ angular = angle_diff * 1.5 │
└────────────┬───────────────┘
             │
             ▼
┌────────────────────────────┐
│ Publish cmd_vel            │
│ to /cmd_vel topic          │
└────────────────────────────┘
```

## Point Cloud Mapping Flow

```
START
  │
  ▼
┌────────────────────────────┐
│ Receive LaserScan          │
│ from /scan topic           │
└────────────┬───────────────┘
             │
             ▼
┌────────────────────────────┐
│ For each scan ray:         │
│ • angle = min + i*increment│
│ • range = ranges[i]        │
└────────────┬───────────────┘
             │
             ▼
┌────────────────────────────┐
│ Convert to Robot Frame     │
│ x_robot = r * cos(angle)   │
│ y_robot = r * sin(angle)   │
└────────────┬───────────────┘
             │
             ▼
┌────────────────────────────┐
│ Transform to Map Frame     │
│ x_map = robot_x +          │
│   x_robot*cos(yaw) -       │
│   y_robot*sin(yaw)         │
└────────────┬───────────────┘
             │
             ▼
┌────────────────────────────┐
│ Ray Tracing                │
│ (Bresenham's Algorithm)    │
│ • Mark free cells (miss)   │
│ • Mark hit cell (occupied) │
└────────────┬───────────────┘
             │
             ▼
┌────────────────────────────┐
│ Update Occupancy Grid      │
│ • hit_count[cell]++        │
│ • miss_count[cell]++       │
│ • prob = hit/(hit+miss)    │
└────────────┬───────────────┘
             │
             ▼
┌────────────────────────────┐
│ Accumulate Point Cloud     │
│ • Subsample points         │
│ • Limit to max_points      │
└────────────┬───────────────┘
             │
             ▼
┌────────────────────────────┐
│ Publish Outputs            │
│ • /scan_pointcloud         │
│ • /map_pointcloud          │
│ • /map (occupancy grid)    │
│ • TF transforms            │
└────────────────────────────┘
```

## ROS2 Topic Communication

```
                    ┌──────────────┐
                    │   User       │
                    │   Input      │
                    └──────┬───────┘
                           │
                           │ ros2 topic pub
                           │ /goal_pose
                           ▼
    ┌────────────────────────────────────────┐
    │        /goal_pose (PoseStamped)        │
    └────────────────┬───────────────────────┘
                     │
                     │ Subscribe
                     ▼
    ┌────────────────────────────────────────┐
    │         A* PATH PLANNER NODE           │
    │  Subscribe: /scan, /odom, /goal_pose   │
    │  Publish: /cmd_vel, /planned_path,     │
    │           /path_markers, /local_costmap│
    └────┬──────────────────────┬────────────┘
         │                      │
         │ /cmd_vel             │ /planned_path
         │ (Twist)              │ (Path)
         │                      │
         ▼                      ▼
    ┌─────────────┐      ┌──────────────┐
    │  GAZEBO     │      │    RVIZ      │
    │  (Robot)    │      │ (Visualization)
    └─────────────┘      └──────────────┘
```

## Complete System Startup Sequence

1. **T+0s**: Script starts
2. **T+2s**: Workspace built
3. **T+3s**: World converted to map
4. **T+4s**: Map publisher started
5. **T+5s**: Gazebo launches
6. **T+15s**: Simulation initialized
7. **T+16s**: Navigation nodes start
   - A* path planner
   - Point cloud mapper
   - Camera combiner
8. **T+18s**: RViz opens
9. **T+20s**: System ready!
   - Auto-navigation begins
   - Robot moves to default goal
   - Point clouds accumulate
   - Path displayed in RViz

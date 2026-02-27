# Visual Demo Guide - A* Navigation System

## What You'll See

When you run `./run_astar_with_mapping.sh`, here's what happens visually:

## Terminal Output

```
========================================
A* Navigation with Point Cloud Mapping
========================================

Configuration:
  World File      : large_messy_room.world
  Default Goal    : (5.0, 5.0)
  Map Resolution  : 0.05 m/pixel
  Output Dir      : /home/sangam/ros2_maps

Sourcing ROS 2 Jazzy environment...
Updating project in workspace...
Building the workspace...

✓ Build complete!

========================================
Step 1: Converting World to Map
========================================

Converting world file...
  World: large_messy_room.world
  Resolution: 0.05 m/pixel
  Output: /home/sangam/ros2_maps/large_messy_room_map

✓ Map conversion complete!
  PGM file  : /home/sangam/ros2_maps/large_messy_room_map.pgm
  YAML file : /home/sangam/ros2_maps/large_messy_room_map.yaml

========================================
Step 2: Starting Map Publisher
========================================

✓ Map publisher started (PID: 12345)

========================================
Step 3: Launching Gazebo & Navigation
========================================

Starting:
  - Gazebo simulation with large_messy_room world
  - TurtleBot3 rover
  - A* Path Planner (goal: 5.0, 5.0)
  - Point Cloud Mapper (real-time mapping)

Waiting for simulation to initialize (15 seconds)...

✓ Simulation started (PID: 12346)

========================================
Step 4: Launching RViz
========================================

✓ RViz launched with config (PID: 12347)

========================================
🚀 Complete Navigation Stack Active! 🚀
========================================
```

## Gazebo Window

You'll see:

### Environment
```
┌─────────────────────────────────────────┐
│  Gazebo Simulation - large_messy_room   │
├─────────────────────────────────────────┤
│                                         │
│  ╔════════════════════════════════╗    │
│  ║  Walls    ████████████    Walls║    │
│  ║           ████    ████         ║    │
│  ║  Obstacles████    ████ Tables  ║    │
│  ║           ████████████         ║    │
│  ║                                ║    │
│  ║     🤖                         ║    │
│  ║  TurtleBot3                    ║    │
│  ║   Rover                        ║    │
│  ║      ↗                         ║    │
│  ║     Moving                     ║    │
│  ║    toward                      ║    │
│  ║     goal                       ║    │
│  ║                        🎯      ║    │
│  ║                      Goal      ║    │
│  ║                      (5,5)     ║    │
│  ╚════════════════════════════════╝    │
│                                         │
└─────────────────────────────────────────┘
```

### Robot Details
- **Blue circular base** with wheels
- **LiDAR sensor** on top (spinning)
- **Red/green laser beams** scanning 360°
- **4 cameras** facing different directions

## RViz Window

### Initial View (Empty)
```
┌─────────────────────────────────────────────────────┐
│ File Edit View Panels Help                          │
├─────────────────────────────────────────────────────┤
│ Fixed Frame: map ▼                                   │
├───────────────┬─────────────────────────────────────┤
│ Displays      │                                     │
│               │     Empty 3D View                   │
│ [ ] Grid      │                                     │
│ [ ] TF        │     Click "Add" button below       │
│ [ ] Map       │     to add displays                │
│ [ ] Path      │                                     │
│               │                                     │
├───────────────┼─────────────────────────────────────┤
│ [Add] [Remove]│                                     │
└───────────────┴─────────────────────────────────────┘
```

### After Configuration (What You Should Add)
```
┌─────────────────────────────────────────────────────────────┐
│ File Edit View Panels Help                    Time: 0:45    │
├─────────────────────────────────────────────────────────────┤
│ Fixed Frame: map ▼               [+] [-] [◉] [⊕] [↻]       │
├───────────────┬─────────────────────────────────────────────┤
│ Displays      │                                             │
│               │          3D Visualization View              │
│ ✓ Grid        │   ┌───────────────────────────────────┐   │
│ ✓ TF          │   │  ████████████    ███████████      │   │
│ ✓ Map         │   │  ███      ███    ███      ███     │   │
│ ✓ Local       │   │  ███      ███    ███      ███     │   │
│   Costmap     │   │  ████████████    ███████████      │   │
│ ✓ Path        │   │                                   │   │
│ ✓ Markers     │   │  🟦 Start                         │   │
│ ✓ Scan PC     │   │   ╲                              │   │
│ ✓ Map PC      │   │    ╲ ────────────────           │   │
│ ✓ LaserScan   │   │     ╲                ╲          │   │
│               │   │      ╲                ╲         │   │
│               │   │       🤖 Robot         ╲        │   │
│               │   │        (TF frames)      ╲       │   │
│               │   │                          ╲      │   │
│               │   │                           🔴    │   │
│               │   │                          Goal   │   │
│               │   └───────────────────────────────────┘   │
│               │                                             │
├───────────────┤  Legend:                                   │
│ Tool          │  ████ = Walls (from static map)           │
│ Properties    │  🟦 = Start marker (blue sphere)          │
│               │  🔴 = Goal marker (red sphere)            │
│ [Add] [Remove]│  ──── = Planned A* path (green line)     │
│               │  🤖 = Robot with TF frames                │
└───────────────┴─────────────────────────────────────────────┘
```

### Color Scheme
- **Static Map**: Gray (obstacles), white (free space)
- **Dynamic Costmap**: Red (obstacles), blue (free), gray (unknown)
- **Planned Path**: Bright green line
- **Start Marker**: Blue sphere
- **Goal Marker**: Red sphere
- **Point Cloud (scan)**: Rainbow colored by distance
- **Point Cloud (map)**: White/gray accumulated points
- **TF Frames**: RGB axes (Red=X, Green=Y, Blue=Z)
- **LiDAR Scan**: Red beams

## Robot Movement Behavior

### Phase 1: Initial Rotation
```
Time: 0-2 seconds

  █████████
  █       █
  █       █    🤖 ↻
  █       █   Rotating
  █       █   toward goal
  █████████   direction
```

### Phase 2: Following Path
```
Time: 2-15 seconds

  █████████
  █       █
  █   🤖  █ ──→  Moving along
  █    │  █      green path line
  █    │  █      
  █    ↓  █     
  █   🔴  █      
  █████████      
```

### Phase 3: Approaching Goal
```
Time: 15-20 seconds

  █████████
  █       █
  █       █     
  █       █  🤖 ↘  Slowing down
  █       █       as getting closer
  █     🔴 █      
  █████████       
```

### Phase 4: Goal Reached
```
Time: 20 seconds

  █████████
  █       █
  █       █     
  █       █      
  █       █  🤖 = 🔴  Robot stopped
  █       █     Goal reached!
  █████████      
```

## Point Cloud Visualization

### LiDAR Scan (Current)
```
        ●  ●
      ●      ●
    ●          ●
   ●            ●
  ●    🤖       ●
   ●            ●
    ●          ●
      ●      ●
        ●  ●

Points = 360 (one per degree)
Colors = Rainbow (by distance)
Update = 10 Hz
```

### Accumulated Map
```
████████████████████
███  ●  ●  ●  ●  ███
███ ●●●●●●●●●●●● ███
███ ●          ● ███
███ ●   🤖    ● ███
███ ●          ● ███
███ ●●●●●●●●●●●● ███
███  ●  ●  ●  ●  ███
████████████████████

● = Scanned points (white/gray)
█ = Known obstacles (from scans)
  = Free space (explored)
Update = Continuous accumulation
```

## Console Output During Navigation

```
[astar_path_planner] INFO: A* Path Planner started!
[astar_path_planner] INFO: Waiting for odometry and scan data...
[astar_path_planner] INFO: Odometry received! Position: (0.00, -8.00)
[astar_path_planner] INFO: Scan data received!
[astar_path_planner] INFO: Sensors ready! Starting navigation...
[astar_path_planner] INFO: Auto-navigating to (5.0, 5.0)
[astar_path_planner] INFO: Planning path: (0.00, -8.00) -> (5.00, 5.00)
[astar_path_planner] INFO: Grid coords: (50, 10) -> (75, 90)
[astar_path_planner] INFO: A* found path with 68 waypoints in 342 iterations
[astar_path_planner] INFO: Path planned with 32 waypoints
[astar_path_planner] INFO: Following: wp 1/32, pos=(0.12,-7.85), target=(0.20,-7.60), dist=0.26
[astar_path_planner] INFO: Waypoint 1/32 reached, pos=(0.21, -7.59)
[astar_path_planner] INFO: Following: wp 2/32, pos=(0.25,-7.51), target=(0.40,-7.20), dist=0.35
...
[astar_path_planner] INFO: Following: wp 31/32, pos=(4.75,4.82), target=(4.90,4.95), dist=0.19
[astar_path_planner] INFO: Waypoint 31/32 reached, pos=(4.91, 4.96)
[astar_path_planner] INFO: Reached final goal at (5.00, 5.00)!
[astar_path_planner] INFO: Goal reached! Waiting for new goal...

[pointcloud_mapper] INFO: Point Cloud Mapper started - building map
[pointcloud_mapper] INFO: Published static transforms
[pointcloud_mapper] INFO: Map has 5432 points
[pointcloud_mapper] INFO: Map has 12567 points
[pointcloud_mapper] INFO: Occupancy grid: 234 occupied, 8765 free cells
...
```

## Typical Navigation Timeline

```
T+0s    ▶ System starts
T+2s    ▶ Workspace built
T+3s    ▶ World converted to map
T+4s    ▶ Map publisher active
T+5s    ▶ Gazebo launches
T+15s   ▶ Sensors initialize
T+16s   ▶ Navigation begins
T+18s   ▶ RViz displays data
T+20s   ▶ Robot starts moving
T+25s   ▶ Path following
T+35s   ▶ Approaching goal
T+40s   ▶ Goal reached!
```

## What Success Looks Like

### ✅ In Gazebo
- Robot smoothly navigates around obstacles
- Avoids walls and furniture
- Turns smoothly at corners
- Stops precisely at goal location

### ✅ In RViz
- Green path line visible
- Robot follows the line
- Point clouds accumulate
- Maps update in real-time
- No collision with obstacles

### ✅ In Terminal
- No error messages (except network warnings - normal)
- Regular position updates
- Waypoint completion messages
- "Goal reached!" message appears

## What to Do Next

### Send New Goals
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: -5.0, y: -5.0}}}'
```

Watch the robot:
1. Stop current motion
2. Plan new path
3. Start following new path
4. Reach new goal

### Monitor Topics
```bash
# Watch path updates
ros2 topic echo /planned_path

# Monitor position
ros2 topic echo /odom

# View point cloud size
ros2 topic hz /map_pointcloud
```

## Stop the System

Press **Ctrl+C** in terminal:
```
^C
Shutting down all processes...
✓ Cleanup complete!
```

All processes (Gazebo, RViz, map publisher, navigation nodes) will stop cleanly.

---

**Tip**: First time running? It may take 20-30 seconds for everything to initialize. Be patient!

**Note**: The "Network is unreachable" messages from Gazebo are normal and can be ignored. They don't affect functionality.

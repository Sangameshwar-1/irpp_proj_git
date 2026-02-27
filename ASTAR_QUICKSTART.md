# Quick Start Guide - A* Navigation with Point Cloud Mapping

## 🚀 Quick Launch

```bash
# Default: large_messy_room world, goal at (5, 5)
./run_astar_with_mapping.sh

# Custom world
./run_astar_with_mapping.sh messy_road.world

# Custom world and goal position
./run_astar_with_mapping.sh iiit_messy_road.world 8.0 -3.0
```

## 📋 What It Does

1. ✅ Builds ROS2 workspace
2. ✅ Converts .world to occupancy map
3. ✅ Publishes static map
4. ✅ Launches Gazebo simulation
5. ✅ Starts A* path planner
6. ✅ Activates point cloud mapper
7. ✅ Opens RViz visualization

## 🎮 Send Goals to Robot

```bash
# Goal at (5, 5)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}}}'

# Goal at (-5, -5)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: -5.0, y: -5.0, z: 0.0}}}'

# Goal at (0, 8)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 8.0, z: 0.0}}}'
```

## 🖥️ RViz Setup

**Set Fixed Frame**: `map`

**Add These Displays**:
- `/map` → Map (static world map)
- `/local_costmap` → Map (dynamic obstacles)
- `/planned_path` → Path (A* path)
- `/path_markers` → MarkerArray (start/goal)
- `/scan_pointcloud` → PointCloud2 (current scan)
- `/map_pointcloud` → PointCloud2 (accumulated map)
- `TF` → Show robot frames

## 📊 Key Topics

| Topic | Purpose |
|-------|---------|
| `/map` | Static map from world file |
| `/local_costmap` | Dynamic obstacles (A* planner) |
| `/planned_path` | A* computed path |
| `/scan_pointcloud` | Current LiDAR scan |
| `/map_pointcloud` | Accumulated 3D map |
| `/goal_pose` | Send navigation goals here |
| `/cmd_vel` | Robot velocity commands |

## 🔍 Monitoring

```bash
# View current path
ros2 topic echo /planned_path --once

# Monitor robot position
ros2 topic echo /odom

# Check planner status
ros2 node info /astar_path_planner

# List all topics
ros2 topic list

# View map info
ros2 topic echo /map --once
```

## 🛠️ Troubleshooting

### Robot not moving?
```bash
# Check if goal was received
ros2 topic echo /goal_pose --once

# Verify cmd_vel is published
ros2 topic hz /cmd_vel
```

### No path found?
- Goal might be inside obstacle
- Try closer goal position
- Check occupancy grid: `ros2 topic echo /local_costmap --once`

### Gazebo not starting?
```bash
# Kill existing processes
pkill -f "gz sim"
pkill -f gazebo
```

## 🎯 Features

✅ **A* Path Planning** - Optimal path with obstacle avoidance  
✅ **Point Cloud Mapping** - Real-time 3D map building  
✅ **Dynamic Replanning** - Automatic path updates  
✅ **Occupancy Grids** - Static (world) + Dynamic (sensors)  
✅ **Goal-Based Navigation** - Send goals via ROS2 topics  
✅ **Visual Feedback** - Path, markers, costmap in RViz  

## 📚 More Information

See [README_ASTAR.md](README_ASTAR.md) for complete documentation.

## ⏹️ Stop Everything

Press **Ctrl+C** in the terminal running the script

``bash
cd /home/sangam/Videos/irpp_proj_git/scan_project/human_models/ros_humans_ros2 && python3 -m py_compile ros_humans_ros2/astar_path_planner.py && echo "OK syntax" && source /opt/ros/jazzy/setup.bash && cd ~/ros2_ws && colcon build --symlink-install --packages-select ros_humans_ros2 2>&1 | tail -5
```
```bash
./run_astar_navigation.sh
```






# DONT REFER THIS _______ROS2 A* Path Planning Navigation with Point Cloud Mapping

A comprehensive ROS2 Jazzy project for autonomous robot navigation using A* path planning, real-time point cloud mapping, and Gazebo simulation.

## 🌟 Features

- **A* Path Planning**: Optimal pathfinding algorithm with obstacle avoidance
- **Point Cloud Mapping**: Real-time 3D map building from LiDAR scans
- **Occupancy Grid Mapping**: Probabilistic grid mapping with ray tracing
- **Gazebo Simulation**: TurtleBot3 rover in various world environments
- **RViz2 Visualization**: Comprehensive visualization of paths, maps, and sensor data
- **360° Camera View**: Multi-camera setup for full environment awareness
- **World-to-Map Converter**: Converts Gazebo .world files to ROS2 occupancy grids
- **Dynamic Replanning**: Automatic path updates when obstacles detected

## 🚀 Quick Start

### 1. Install Dependencies
```bash
sudo ./setup_dependencies.sh
```

### 2. Run Complete A* Navigation System (Recommended)
```bash
# Default: large_messy_room world, goal at (5, 5)
./run_astar_with_mapping.sh

# Custom world and goal
./run_astar_with_mapping.sh messy_road.world 8.0 -3.0
```

### 3. Or Run A* Navigation Only
```bash
./run_astar_navigation.sh
```

### 4. Or Convert World to Map & Visualize
```bash
./convert_world_to_map.sh large_messy_room.world
```

### 5. Or Run Basic Simulation (with moving humans)
```bash
./run_simulation.sh
```

## 📖 Documentation

📑 **[Complete Documentation Index](DOCS_INDEX.md)** - Navigate all documentation

- 📚 **[A* Navigation Guide](README_ASTAR.md)** - Complete A* implementation details
- ⚡ **[Quick Start Guide](ASTAR_QUICKSTART.md)** - Quick reference for common tasks
- 🗺️ **[World Converter Guide](WORLD_TO_MAP_GUIDE.md)** - Converting world files to maps
- 🔄 **[System Flow Diagrams](SYSTEM_FLOW.md)** - Detailed system architecture
- ✅ **[Implementation Summary](IMPLEMENTATION_SUMMARY.md)** - What's been implemented
- 🎬 **[Visual Demo Guide](VISUAL_DEMO_GUIDE.md)** - What to expect when running
- 🔧 **[Troubleshooting Guide](TROUBLESHOOTING.md)** - Common issues and solutions

## 🎮 Usage

Once the simulation is running, send navigation goals:

```bash
# Navigate to position (5, 5)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}}}'

# Navigate to position (-5, -5)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: -5.0, y: -5.0, z: 0.0}}}'
```

## 📁 Project Structure

```
├── run_astar_with_mapping.sh  # Complete A* + mapping system ⭐ (NEW!)
├── run_astar_navigation.sh    # A* navigation simulation
├── convert_world_to_map.sh    # World to map converter
├── run_simulation.sh          # Basic simulation with humans
├── setup_dependencies.sh      # Install ROS2 and dependencies
│
├── README.md                  # This file
├── README_ASTAR.md            # Complete A* documentation
├── ASTAR_QUICKSTART.md        # Quick reference guide
├── SYSTEM_FLOW.md             # System architecture diagrams
├── IMPLEMENTATION_SUMMARY.md  # Implementation details
├── VISUAL_DEMO_GUIDE.md       # Visual walkthrough
├── WORLD_TO_MAP_GUIDE.md      # World converter guide
│
└── scan_project/
    ├── rviz_config.rviz       # RViz2 configuration
    └── human_models/
        └── ros_humans_ros2/   # ROS2 package
            ├── ros_humans_ros2/
            │   ├── astar_path_planner.py      # A* implementation (584 lines)
            │   ├── pointcloud_mapper.py       # Point cloud mapper (336 lines)
            │   ├── world_to_map.py            # World converter
            │   ├── map_publisher.py           # Static map publisher
            │   └── camera_view_360.py         # 360° camera view
            ├── launch/
            │   ├── astar_navigation.launch.py # Main launch file
            │   ├── messy_road.launch.py
            │   └── ...
            ├── worlds/
            │   ├── large_messy_room.world     # Default world
            │   ├── messy_road.world
            │   ├── iiit_messy_road.world
            │   └── ... (8+ worlds)
            ├── setup.py
            └── package.xml
```

## 🎯 Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/goal_pose` | PoseStamped | Send navigation goals (input) |
| `/cmd_vel` | Twist | Robot velocity commands (output) |
| `/planned_path` | Path | A* computed path |
| `/scan` | LaserScan | LiDAR scan data |
| `/odom` | Odometry | Robot position |
| `/map` | OccupancyGrid | Static map from world |
| `/local_costmap` | OccupancyGrid | Dynamic obstacles |
| `/scan_pointcloud` | PointCloud2 | Current LiDAR scan |
| `/map_pointcloud` | PointCloud2 | Accumulated 3D map |
| `/path_markers` | MarkerArray | Path visualization |

## 🔧 Configuration

### A* Planner Parameters
```yaml
linear_speed: 0.3         # m/s - forward speed
angular_speed: 0.5        # rad/s - turning speed
goal_tolerance: 0.3       # m - waypoint reaching distance
grid_resolution: 0.2      # m/cell - occupancy grid resolution
obstacle_inflation: 0.4   # m - safety margin around obstacles
auto_start: true          # automatically navigate to default goal
```

### Point Cloud Mapper Parameters
```yaml
map_resolution: 0.1       # m/cell - grid resolution
map_size: 30.0            # m - map dimensions (30m x 30m)
max_points: 100000        # maximum points in accumulated map
```
            ├── launch/        # Launch files
            ├── ros_humans_ros2/  # Python nodes
            └── worlds/        # Gazebo world files
```

## Available Worlds

- `large_messy_room.world` - Indoor environment with obstacles
- `messy_road.world` - Outdoor road scenario
- `iiit_messy_road.world` - IIIT Hyderabad campus environment
- `complex_grid_map.world` - Grid-based navigation challenge

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/goal_pose` | PoseStamped | Send navigation goals |
| `/planned_path` | Path | Computed A* path |
| `/local_costmap` | OccupancyGrid | LiDAR-based occupancy grid |
| `/scan` | LaserScan | LiDAR scan data |
| `/odom` | Odometry | Robot odometry |

## Requirements

- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Sim (Harmonic)

## License

MIT

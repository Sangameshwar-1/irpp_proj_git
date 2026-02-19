# ROS2 A* Path Planning Navigation

A ROS2 Jazzy project for autonomous robot navigation using A* path planning in Gazebo simulation.

## Features

- **A* Path Planning**: Goal-based autonomous navigation
- **Point Cloud Mapping**: Real-time map building from LiDAR scans
- **Gazebo Simulation**: TurtleBot3 rover in various world environments
- **RViz2 Visualization**: Real-time visualization of paths, maps, and sensor data
- **360° Camera View**: Multi-camera setup for full environment awareness

## Quick Start

### 1. Install Dependencies
```bash
sudo ./setup_dependencies.sh
```

### 2. Run A* Navigation Simulation
```bash
./run_astar_navigation.sh
```

### 3. Run Basic Simulation (with moving humans)
```bash
./run_simulation.sh
```

## Usage

Once the simulation is running, send navigation goals:

```bash
# Navigate to position (5, 5)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}}}'
```

## Project Structure

```
├── run_astar_navigation.sh    # Launch A* navigation simulation
├── run_simulation.sh          # Launch basic simulation with humans
├── setup_dependencies.sh      # Install ROS2 and dependencies
└── scan_project/
    ├── rviz_config.rviz       # RViz2 configuration
    └── human_models/
        └── ros_humans_ros2/   # ROS2 package
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

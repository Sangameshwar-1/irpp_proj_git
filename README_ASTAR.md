# A* Path Planning with Point Cloud Mapping

## Overview

This project implements a complete autonomous navigation system for a TurtleBot rover using:

- **A\* Path Planning Algorithm**: Optimal pathfinding with obstacle avoidance
- **Point Cloud Mapping**: Real-time 3D map building from LiDAR sensor data
- **Gazebo Simulation**: Realistic physics simulation with custom world environments
- **RViz Visualization**: Comprehensive visualization of robot, map, path, and sensors

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         GAZEBO SIMULATION                        │
│  ┌──────────────┐  ┌─────────────┐  ┌──────────────────────┐  │
│  │ TurtleBot3   │  │   LiDAR     │  │   World Environment  │  │
│  │   Rover      │  │   Sensor    │  │   (Obstacles, Walls) │  │
│  └──────────────┘  └─────────────┘  └──────────────────────┘  │
└────────────┬──────────────┬──────────────────────────────────────┘
             │              │
             │              │  /scan, /odom, /camera
             ▼              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      ROS2 MIDDLEWARE                             │
└────────────┬────────────────┬────────────────────────────────────┘
             │                │
             ▼                ▼
┌──────────────────────┐  ┌────────────────────────────────────┐
│  A* PATH PLANNER     │  │   POINT CLOUD MAPPER               │
│                      │  │                                    │
│  - Occupancy Grid    │  │  - LiDAR to PointCloud2           │
│  - A* Algorithm      │  │  - Accumulated Map Building       │
│  - Path Following    │  │  - Occupancy Grid Generation      │
│  - Obstacle Inflation│  │  - TF Transforms                  │
│  - Dynamic Replanning│  │  - Ray Tracing                    │
└──────────┬───────────┘  └────────────┬───────────────────────┘
           │                           │
           │  /cmd_vel                 │  /map_pointcloud, /map
           │  /planned_path            │  /scan_pointcloud
           │  /path_markers            │
           │  /local_costmap           │
           ▼                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                      RVIZ VISUALIZATION                          │
│  - Robot Model (TF)      - Occupancy Grids (static + dynamic)   │
│  - Planned Path          - Point Clouds (scan + accumulated)    │
│  - Goal Markers          - Costmaps                             │
│  - Camera Views          - LiDAR Scans                          │
└─────────────────────────────────────────────────────────────────┘
```

## Features

### 🎯 A* Path Planning

- **Optimal Pathfinding**: Uses A* algorithm with Euclidean heuristic
- **8-Connected Grid**: Supports diagonal movement for smoother paths
- **Dynamic Obstacle Avoidance**: Updates occupancy grid from LiDAR in real-time
- **Path Simplification**: Removes unnecessary waypoints for efficient navigation
- **Goal Tolerance**: Configurable proximity threshold for waypoint reaching
- **Obstacle Inflation**: Safety margin around obstacles

### 🗺️ Point Cloud Mapping

- **Real-Time Mapping**: Converts LiDAR scans to 3D point clouds
- **Accumulated Map**: Builds persistent map of explored environment
- **Occupancy Grid**: Probabilistic grid with hit/miss counting
- **Ray Tracing**: Bresenham's algorithm for free space marking
- **TF Management**: Publishes all required coordinate transforms
- **Configurable Resolution**: Adjustable grid resolution (default: 10cm/cell)

### 🤖 Robot Navigation

- **Pure Pursuit Controller**: Smooth trajectory following
- **Orientation Control**: Turn-in-place for large heading errors
- **Proportional Steering**: Smooth cornering with angle-based steering
- **Adaptive Speed**: Slows down near waypoints
- **Goal Reaching**: Automatic detection of goal completion
- **Multi-Waypoint Support**: Sequential waypoint navigation

### 🌍 World Integration

- **World-to-Map Converter**: Converts Gazebo .world files to ROS2 occupancy grids
- **Multiple World Support**: Compatible with various world files
- **Static Map Publishing**: Provides ground truth map from world file
- **Dynamic Map Updates**: Real-time sensor-based mapping overlays static map

## File Structure

```
irpp_proj_git/
├── run_astar_with_mapping.sh           # Main launch script (NEW!)
├── run_astar_navigation.sh             # Alternative launch script
├── convert_world_to_map.sh             # World to map converter
├── README_ASTAR.md                     # This file
├── WORLD_TO_MAP_GUIDE.md               # World conversion guide
└── scan_project/
    ├── rviz_config.rviz                # RViz configuration
    └── human_models/
        └── ros_humans_ros2/
            ├── ros_humans_ros2/
            │   ├── astar_path_planner.py      # A* implementation
            │   ├── pointcloud_mapper.py       # Point cloud mapper
            │   ├── world_to_map.py            # World converter
            │   ├── map_publisher.py           # Static map publisher
            │   └── camera_view_360.py         # 360° camera view
            ├── launch/
            │   └── astar_navigation.launch.py # Launch file
            ├── worlds/
            │   ├── large_messy_room.world     # Default world
            │   ├── messy_road.world           # Outdoor environment
            │   ├── iiit_messy_road.world      # Campus environment
            │   └── ... (other worlds)
            ├── setup.py                       # Package setup
            └── package.xml                    # Package metadata
```

## Quick Start

### Prerequisites

```bash
# ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Required packages
sudo apt install ros-jazzy-gazebo-ros-pkgs
sudo apt install ros-jazzy-ros-gz-bridge
sudo apt install ros-jazzy-navigation2
sudo apt install python3-numpy python3-opencv
```

### Basic Usage

```bash
# 1. Clone the repository
cd ~/Videos/irpp_proj_git

# 2. Run with default settings (large_messy_room, goal at 5,5)
./run_astar_with_mapping.sh

# 3. Or specify custom world and goal
./run_astar_with_mapping.sh messy_road.world 8.0 -3.0

# 4. Or use the alternative script
./run_astar_navigation.sh
```

### What the Script Does

The `run_astar_with_mapping.sh` script performs these steps:

1. **Builds the ROS2 workspace** (if needed)
2. **Converts the .world file** to a static occupancy grid map
3. **Starts the map publisher** for the static map
4. **Launches Gazebo** with the TurtleBot rover and world environment
5. **Activates the A* path planner** with automatic goal navigation
6. **Starts the point cloud mapper** for real-time mapping
7. **Opens RViz** for visualization

## Sending Custom Goals

Once the system is running, send navigation goals via ROS2 topics:

```bash
# Navigate to position (5, 5)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}}}'

# Navigate to position (-5, -5)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: -5.0, y: -5.0, z: 0.0}}}'

# Navigate to position (0, 8)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 8.0, z: 0.0}}}'
```

## RViz Configuration

Configure RViz to display the following topics:

### Essential Displays

1. **Fixed Frame**: Set to `map`
2. **Map** → Add `/map` (static map from world file)
3. **Map** → Add `/local_costmap` (dynamic obstacles from A*)
4. **Path** → Add `/planned_path` (A* computed path)
5. **MarkerArray** → Add `/path_markers` (start/goal/path markers)
6. **TF** → Enable to show robot frames
7. **PointCloud2** → Add `/scan_pointcloud` (current LiDAR scan)
8. **PointCloud2** → Add `/map_pointcloud` (accumulated map)

### Optional Displays

- **LaserScan** → `/scan` (raw LiDAR data)
- **Image** → `/camera/front/image` (front camera)
- **Image** → `/camera/360/image` (combined 360° view)
- **Odometry** → `/odom` (robot position trail)

## ROS2 Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Static map from world file |
| `/local_costmap` | `nav_msgs/OccupancyGrid` | Dynamic costmap from A* planner |
| `/planned_path` | `nav_msgs/Path` | A* computed path |
| `/path_markers` | `visualization_msgs/MarkerArray` | Path visualization |
| `/scan_pointcloud` | `sensor_msgs/PointCloud2` | Current LiDAR scan |
| `/map_pointcloud` | `sensor_msgs/PointCloud2` | Accumulated map |
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Navigation goal |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan data |
| `/odom` | `nav_msgs/Odometry` | Robot odometry |

## Parameters

### A* Path Planner Parameters

```yaml
linear_speed: 0.3           # m/s - forward speed
angular_speed: 0.5          # rad/s - turning speed
goal_tolerance: 0.3         # m - distance to consider waypoint reached
grid_resolution: 0.2        # m/cell - occupancy grid resolution
grid_size: 100              # cells - grid dimensions (100x100)
obstacle_inflation: 0.4     # m - safety margin around obstacles
auto_start: true            # automatically navigate to default goal
default_goal_x: 5.0         # m - default goal X coordinate
default_goal_y: 5.0         # m - default goal Y coordinate
```

### Point Cloud Mapper Parameters

```yaml
map_frame: "map"            # coordinate frame for map
robot_frame: "base_footprint"  # robot base frame
max_points: 100000          # maximum points in accumulated map
map_resolution: 0.1         # m/cell - occupancy grid resolution
map_size: 30.0              # m - map dimensions (30m x 30m)
```

## Monitoring and Debugging

### View Node Information

```bash
# A* planner status
ros2 node info /astar_path_planner

# Point cloud mapper status
ros2 node info /pointcloud_mapper

# List all nodes
ros2 node list
```

### Monitor Topics

```bash
# View current goal
ros2 topic echo /goal_pose --once

# View planned path
ros2 topic echo /planned_path --once

# Monitor robot position
ros2 topic echo /odom

# View map info
ros2 topic echo /map --once

# List all topics
ros2 topic list
```

### Check Transform Tree

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo map base_footprint
```

## Algorithm Details

### A* Pathfinding

The A* algorithm uses:

- **Cost Function**: `f(n) = g(n) + h(n)`
  - `g(n)`: Actual cost from start to node n
  - `h(n)`: Heuristic (Euclidean distance to goal)
- **8-Connected Grid**: Allows diagonal movement
- **Diagonal Cost**: `√2` for diagonal moves, `1` for cardinal moves
- **Obstacle Avoidance**: Cells with occupancy > 50% are blocked
- **Path Simplification**: Removes collinear waypoints

### Point Cloud Mapping

The mapper implements:

- **Coordinate Transforms**: Robot frame → Map frame
- **Ray Tracing**: Bresenham's line algorithm for free space
- **Probabilistic Occupancy**: Hit/miss counting with thresholds
  - Hit > 65% → Occupied (100)
  - Hit < 35% → Free (0)
  - Otherwise → Unknown (-1)
- **Point Accumulation**: Subsampled scan points for efficiency

### Path Following Controller

The controller uses:

- **Pure Pursuit**: Tracks waypoints sequentially
- **Orientation Control**: Turns toward target when angle error > 17°
- **Proportional Steering**: `ω = k * angle_error`
- **Adaptive Speed**: `v = min(v_max, k * distance)`
- **Waypoint Switching**: When `distance < tolerance`

## Troubleshooting

### Gazebo not starting

```bash
# Kill existing processes
pkill -f "gz sim"
pkill -f gazebo

# Check for port conflicts
netstat -tuln | grep 11345
```

### Robot not moving

```bash
# Check cmd_vel publisher
ros2 topic info /cmd_vel

# Verify goal was received
ros2 topic echo /goal_pose --once

# Check planner state
ros2 topic echo /planned_path --once
```

### No path found

- Ensure goal is reachable (not inside obstacle)
- Check occupancy grid: `ros2 topic echo /local_costmap --once`
- Reduce `obstacle_inflation` parameter
- Increase `grid_size` parameter

### Point cloud not displaying

```bash
# Check if mapper is publishing
ros2 topic hz /scan_pointcloud

# Verify LiDAR data
ros2 topic echo /scan --once

# Check TF tree
ros2 run tf2_tools view_frames
```

## Advanced Usage

### Custom World Files

1. Place your `.world` file in `scan_project/human_models/ros_humans_ros2/worlds/`
2. Run with custom world:
   ```bash
   ./run_astar_with_mapping.sh your_world.world
   ```

### Modifying Parameters

Edit the launch file:
```bash
nano scan_project/human_models/ros_humans_ros2/launch/astar_navigation.launch.py
```

Or pass via command line:
```bash
ros2 launch ros_humans_ros2 astar_navigation.launch.py \
  default_goal_x:=8.0 \
  default_goal_y:=-3.0
```

### Multiple Goals (Sequential Navigation)

```bash
# Goal 1
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 5.0}}}'

# Wait for robot to reach goal 1...

# Goal 2
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: -5.0, y: -5.0}}}'
```

## Performance Optimization

### For Large Maps

```yaml
# Increase grid size
grid_size: 200              # 200x200 cells

# Reduce resolution
grid_resolution: 0.3        # 30cm/cell

# Limit point cloud
max_points: 50000           # fewer points
```

### For Faster Navigation

```yaml
# Increase speeds
linear_speed: 0.5           # faster forward
angular_speed: 0.8          # faster turning

# Reduce tolerance
goal_tolerance: 0.2         # closer waypoint reaching
```

### For Better Obstacle Avoidance

```yaml
# Increase inflation
obstacle_inflation: 0.6     # larger safety margin

# Finer grid
grid_resolution: 0.15       # 15cm/cell
```

## Future Enhancements

- [ ] Dynamic obstacle replanning (currently disabled)
- [ ] Global path planner (Dijkstra or RRT*)
- [ ] SLAM integration (instead of static map)
- [ ] Multi-robot coordination
- [ ] Learned cost maps
- [ ] Velocity obstacles for dynamic objects

## References

- A* Algorithm: Hart, P. E.; Nilsson, N. J.; Raphael, B. (1968)
- ROS2 Navigation: https://navigation.ros.org/
- Gazebo Simulation: https://gazebosim.org/
- TurtleBot3: https://emanual.robotis.com/docs/en/platform/turtlebot3/

## License

MIT License - See LICENSE file for details

## Author

Developed for the IRPP (Intelligent Robotics Path Planning) project

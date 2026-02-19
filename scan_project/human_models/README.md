# ROS 2 Human Simulation & Complex Campus Map

This project simulates a complex environment (based on IIIT Hyderabad campus layout) with dynamic human agents in Gazebo Sim (Harmonic) using ROS 2 Jazzy.


## Quick Start (Scripts)

We provide automated scripts to set up your environment and run the simulation.

### 1. Install Dependencies
Run this once to install ROS 2 Jazzy, Gazebo Sim, and build tools.
```bash
cd ../../..  # Navigate to project root (IRPP/PROJ)
sudo ./setup_dependencies.sh
```

### 2. Build & Run Simulation
This script creates the workspace, builds the package, and launches the simulation.
```bash
cd ../../..  # Navigate to project root (IRPP/PROJ)
./run_simulation.sh
```





## Features
- **Complex World Map**: `complex_grid_map.world` features a realistic layout with:
  - Varied road widths (Arteries, internal roads, alleys).
  - Multiple building blocks (Academic, Residential, Service).
  - Dense "Warren" areas for tight navigation challenges.
- **Dynamic Human Agents**: 4 autonomous agents with distinct navigation behaviors:
  - `human_moving_1`: High-speed navigation on main arteries.
  - `human_moving_2`: Grid-based navigation in the central plaza.
  - `human_moving_3`: Complex maneuvering in tight "Warren" spaces.
  - `human_moving_4`: Alley patrol.
- **ROS 2 Integration**:
  - Full ROS 2 Jazzy support.
  - `ros_gz_bridge` for communication between Gazebo and ROS 2.
  - Publishes human poses to TF and custom topics.

## Directories
- `gazebo_worlds/`: Contains the SDF world files (`complex_grid_map.world`).
- `ros_humans_ros2/`: The ROS 2 package containing nodes for moving humans and publishing poses.


```

## Manual Usage

### Build
```bash
# Assuming your workspace is ~/ros2_ws
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch
```bash
ros2 launch ros_humans_ros2 demo.launch.py
```

## Topics & Interfaces
- **TF Frames**: `human_moving_1` ... `human_moving_4`
- **Gazebo Bridges**:
  - `/world/complex_grid_map/pose/info` (tf2_msgs/TFMessage)
  - `/world/complex_grid_map/set_pose` (Service)

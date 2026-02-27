# World to Map Conversion Guide

This guide explains how to convert Gazebo `.world` files directly to ROS 2 occupancy grid maps and visualize them in RViz without running SLAM or simulation.

## Overview

The world-to-map conversion tool extracts static obstacles from Gazebo world files and generates:
- **PGM file**: Portable GrayMap image (occupancy grid visualization)
- **YAML file**: ROS 2 map_server configuration with metadata

## Features

✅ Direct conversion from .world to map (no simulation needed)  
✅ Supports multiple geometry types: box, cylinder, sphere, cone  
✅ Handles rotated obstacles and complex poses  
✅ Configurable resolution and obstacle inflation  
✅ Automatic map bounds detection  
✅ RViz visualization with map publisher node  

## Quick Start

### Method 1: Using the Bash Script (Easiest)

```bash
# Convert default world and launch RViz
./convert_world_to_map.sh

# Convert specific world file
./convert_world_to_map.sh messy_road.world

# Custom resolution (0.03 m/pixel)
./convert_world_to_map.sh large_messy_room.world 0.03

# Custom output directory
./convert_world_to_map.sh messy_road.world 0.05 ~/my_maps
```

The script automatically:
1. Converts the world file to PGM+YAML
2. Starts a map publisher node
3. Launches RViz for visualization

### Method 2: Using ROS 2 Launch File

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash
cd ~/Videos/irpp_proj_git/scan_project/human_models/ros_humans_ros2
source install/setup.bash

# Launch with default world
ros2 launch ros_humans_ros2 world_to_map.launch.py

# Launch with specific world
ros2 launch ros_humans_ros2 world_to_map.launch.py world:=messy_road.world

# Custom resolution
ros2 launch ros_humans_ros2 world_to_map.launch.py \
  world:=large_messy_room.world \
  resolution:=0.03

# Custom output directory
ros2 launch ros_humans_ros2 world_to_map.launch.py \
  world:=messy_road.world \
  output_dir:=/tmp/my_maps

# Without RViz
ros2 launch ros_humans_ros2 world_to_map.launch.py launch_rviz:=false
```

### Method 3: Manual Conversion (Command Line)

```bash
# Build the package first
cd ~/Videos/irpp_proj_git/scan_project/human_models/ros_humans_ros2
colcon build --symlink-install
source install/setup.bash

# Convert world to map
ros2 run ros_humans_ros2 world_to_map \
  /path/to/world/file.world \
  -o ~/maps/my_map \
  -r 0.05

# Publish the generated map
ros2 run ros_humans_ros2 map_publisher \
  --ros-args \
  -p yaml_file:=~/maps/my_map.yaml \
  -p publish_rate:=1.0

# Launch RViz separately
rviz2 -d scan_project/rviz_config.rviz
```

## Available World Files

The package includes several pre-configured world files:

| World File | Description |
|------------|-------------|
| `large_messy_room.world` | Large indoor environment with obstacles |
| `messy_road.world` | Outdoor road with traffic elements |
| `iiit_messy_road.world` | IIIT Hyderabad campus environment |
| `complex_grid_map.world` | Grid-based maze environment |
| `custom_map.world` | Custom test environment |
| `indoor_with_humans.world` | Indoor space with human models |
| `institute_city.world` | City-like environment |
| `iiit_hyderabad.world` | IIIT campus detailed map |

## Command Line Options

### world_to_map Tool

```bash
ros2 run ros_humans_ros2 world_to_map [OPTIONS] WORLD_FILE
```

**Arguments:**
- `WORLD_FILE` - Path to the .world SDF file (required)

**Options:**
- `-o, --output PATH` - Output base path (without extension). Default: same as world file
- `-r, --resolution FLOAT` - Map resolution in meters/pixel (default: 0.05)
- `--xmin FLOAT` - Map left boundary in meters (auto-detected if not specified)
- `--xmax FLOAT` - Map right boundary in meters
- `--ymin FLOAT` - Map bottom boundary in meters  
- `--ymax FLOAT` - Map top boundary in meters
- `--inflate FLOAT` - Obstacle inflation radius in meters (default: 0.05)
- `--padding FLOAT` - Extra border when auto-detecting bounds (default: 1.0)
- `--list` - List all obstacles in the world and exit (no map generated)

**Examples:**

```bash
# Basic conversion
ros2 run ros_humans_ros2 world_to_map worlds/messy_road.world

# High resolution map
ros2 run ros_humans_ros2 world_to_map worlds/large_messy_room.world -r 0.02

# Custom output location
ros2 run ros_humans_ros2 world_to_map worlds/messy_road.world \
  -o ~/maps/road_map

# Explicit map bounds (useful for cropping)
ros2 run ros_humans_ros2 world_to_map worlds/large_messy_room.world \
  --xmin -10 --xmax 10 --ymin -10 --ymax 10

# Larger obstacle inflation (safer navigation)
ros2 run ros_humans_ros2 world_to_map worlds/messy_road.world \
  --inflate 0.2

# List all obstacles without generating map
ros2 run ros_humans_ros2 world_to_map worlds/messy_road.world --list
```

### map_publisher Node

```bash
ros2 run ros_humans_ros2 map_publisher --ros-args -p yaml_file:=PATH
```

**Parameters:**
- `yaml_file` (string) - Path to map YAML file (required)
- `publish_rate` (double) - Publishing frequency in Hz (default: 1.0)
- `frame_id` (string) - TF frame ID for the map (default: "map")

**Examples:**

```bash
# Publish map at default rate
ros2 run ros_humans_ros2 map_publisher \
  --ros-args -p yaml_file:=~/maps/room_map.yaml

# Higher publishing rate
ros2 run ros_humans_ros2 map_publisher \
  --ros-args \
  -p yaml_file:=~/maps/room_map.yaml \
  -p publish_rate:=10.0

# Custom frame ID
ros2 run ros_humans_ros2 map_publisher \
  --ros-args \
  -p yaml_file:=~/maps/room_map.yaml \
  -p frame_id:=world
```

## RViz Configuration

After launching RViz, follow these steps to visualize the map:

1. **Set the Fixed Frame**:
   - In Global Options panel
   - Set Fixed Frame to `map`

2. **Add Map Display**:
   - Click `Add` button (bottom left)
   - Select `By topic` tab
   - Expand `/map` topic
   - Select `Map` and click OK

3. **Adjust View**:
   - Use mouse to pan and zoom
   - Scroll to zoom in/out
   - Middle-click drag to pan

## Map Format

### PGM Image Format
- **255 (white)**: Free space (robot can move)
- **0 (black)**: Occupied space (obstacles)
- **205 (gray)**: Unknown space (not scanned)

### YAML Configuration
```yaml
image: map_name.pgm           # PGM image filename
resolution: 0.05              # meters per pixel
origin: [-12.5, -12.5, 0.0]  # map origin (x, y, yaw)
negate: 0                     # 0 = black is occupied
occupied_thresh: 0.65         # >= 65% = occupied
free_thresh: 0.196            # < 20% = free
```

## Troubleshooting

### "World file not found"
- Check the file path is correct
- Use absolute paths or relative to package
- List available worlds: `ls worlds/*.world`

### "No map appearing in RViz"
1. Check Fixed Frame is set to `map`
2. Verify map publisher is running: `ros2 node list`
3. Check map topic: `ros2 topic echo /map --once`
4. Ensure map display is added in RViz

### "Map is too large/small"
- Adjust resolution parameter (smaller = more detail, larger file)
- Use explicit bounds with `--xmin`, `--xmax`, `--ymin`, `--ymax`

### "Obstacles are missing"
- Check if models are marked as `static` in world file
- Use `--list` option to see what's being detected
- Verify obstacle isn't in skip list (humans, lights, etc.)

### "Map is rotated or offset"
- Check origin values in YAML file
- Ensure world file uses standard coordinate frame
- Verify pose values in world file

## Building the Package

If you made changes to the code, rebuild:

```bash
cd ~/Videos/irpp_proj_git/scan_project/human_models/ros_humans_ros2
colcon build --symlink-install
source install/setup.bash
```

## Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Static occupancy grid map |

## Integration with Navigation

The generated maps can be used with:
- **Nav2**: ROS 2 navigation stack
- **SLAM**: As ground truth for evaluation
- **Path Planning**: A* planner, Dijkstra, etc.
- **Localization**: AMCL (Adaptive Monte Carlo Localization)

Example with Nav2:
```bash
# Use generated map with Nav2 map_server
ros2 run nav2_map_server map_server \
  --ros-args \
  -p yaml_filename:=~/maps/room_map.yaml \
  -p use_sim_time:=true
```

## Advanced Usage

### Batch Conversion
Convert all world files in a directory:
```bash
for world in worlds/*.world; do
    ros2 run ros_humans_ros2 world_to_map "$world" \
      -o "maps/$(basename $world .world)_map" \
      -r 0.05
done
```

### Python API
Use the conversion tool programmatically:
```python
from ros_humans_ros2.world_to_map import parse_world, generate_map, write_pgm, write_yaml

# Parse world file
obstacles = parse_world("path/to/world.world")

# Generate map
grid, origin_x, origin_y = generate_map(
    obstacles,
    resolution=0.05,
    inflate=0.1
)

# Save to files
write_pgm(grid, "output_map.pgm")
write_yaml("output_map.pgm", "output_map.yaml", 0.05, origin_x, origin_y)
```

## See Also

- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/)
- [Nav2 Map Server](https://navigation.ros.org/configuration/packages/configuring-map-server.html)
- [Gazebo SDF Format](http://sdformat.org/)
- [PGM Image Format](http://netpbm.sourceforge.net/doc/pgm.html)

## Support

For issues or questions:
1. Check existing world files for examples
2. Use `--list` to debug obstacle detection
3. Verify ROS 2 and Gazebo installations
4. Check RViz console for error messages

# Troubleshooting Guide - A* Navigation System

## Common Issues and Solutions

### 1. Robot Not Moving

**Symptoms:**
- Gazebo shows robot but it's stationary
- No movement after goal is published
- Console shows "Waiting for sensors..."

**Solutions:**

```bash
# Check if goal was received
ros2 topic echo /goal_pose --once

# Verify cmd_vel is being published
ros2 topic hz /cmd_vel

# Check if planner is active
ros2 node info /astar_path_planner

# Verify sensors are working
ros2 topic hz /scan
ros2 topic hz /odom

# Try sending goal again
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 5.0}}}'
```

**Common Causes:**
- Sensors not initialized yet (wait 20-30 seconds)
- Goal position is inside an obstacle
- Time synchronization issue (check `use_sim_time`)

---

### 2. A* Path Not Found

**Symptoms:**
- Console shows "A* could not find path"
- No green path line in RViz
- Robot receives goal but doesn't move

**Solutions:**

```bash
# Check occupancy grid
ros2 topic echo /local_costmap --once

# Verify start and goal positions
ros2 topic echo /odom --once

# Try closer goal
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 2.0}}}'
```

**Common Causes:**
- Goal is unreachable (blocked by obstacles)
- Goal is outside grid bounds
- `obstacle_inflation` too high

**Fix:**
1. Reduce obstacle inflation:
   - Edit `launch/astar_navigation.launch.py`
   - Change `obstacle_inflation: 0.2` (from 0.4)
2. Increase grid size:
   - Change `grid_size: 150` (from 100)
3. Try different goal position

---

### 3. Gazebo Won't Start

**Symptoms:**
- Script starts but Gazebo window doesn't appear
- Error: "gz: command not found"
- Error: "Failed to load world"

**Solutions:**

```bash
# Kill existing Gazebo processes
pkill -f "gz sim"
pkill -f gazebo
pkill -f ruby

# Check for port conflicts
netstat -tuln | grep 11345

# Clear Gazebo cache
rm -rf ~/.gz/sim/*

# Verify Gazebo installation
gz sim --version

# Try manual launch
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
gz sim -v 4  # Verbose output
```

**Common Causes:**
- Previous Gazebo instance still running
- Port 11345 already in use
- Missing Gazebo installation

---

### 4. RViz Not Showing Map/Path

**Symptoms:**
- RViz opens but displays are empty
- "No map received" message
- TF errors in console

**Solutions:**

```bash
# Check if map is being published
ros2 topic hz /map
ros2 topic echo /map --once

# Verify Fixed Frame is set to "map"
# In RViz: Global Options -> Fixed Frame -> "map"

# Check TF tree
ros2 run tf2_tools view_frames
# Open frames.pdf to see transform tree

# Reset RViz displays
# In RViz: File -> Reset

# Check topic names
ros2 topic list
```

**Common Causes:**
- Fixed Frame not set to "map"
- Map publisher not started
- Wrong topic names in displays

**Fix:**
1. Set Fixed Frame: `map`
2. Add displays manually:
   - Click "Add" button
   - Select "By topic"
   - Add `/map` -> `Map`
   - Add `/planned_path` -> `Path`

---

### 5. Point Clouds Not Visible

**Symptoms:**
- RViz shows robot but no point clouds
- `/scan_pointcloud` topic exists but nothing displays

**Solutions:**

```bash
# Check if point clouds are published
ros2 topic hz /scan_pointcloud
ros2 topic hz /map_pointcloud

# Verify point cloud size
ros2 topic echo /scan_pointcloud --once | head -20

# Check if mapper is running
ros2 node list | grep pointcloud
```

**In RViz:**
1. Add PointCloud2 display
2. Set Topic to `/scan_pointcloud`
3. Set Size (m) to 0.05
4. Set Style to Points
5. Set Color Transformer to Intensity or AxisColor

**Common Causes:**
- LiDAR not receiving data
- Point cloud size too small to see
- Display settings incorrect

---

### 6. Build Errors

**Symptoms:**
- `colcon build` fails
- Import errors
- Missing dependencies

**Solutions:**

```bash
# Install missing dependencies
sudo apt update
sudo apt install -y \
  ros-jazzy-gazebo-ros-pkgs \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-navigation2 \
  python3-numpy \
  python3-opencv

# Clean and rebuild
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install --packages-select ros_humans_ros2

# Source workspace
source install/setup.bash

# Verify package
ros2 pkg prefix ros_humans_ros2
```

---

### 7. "Network is unreachable" Errors

**Symptoms:**
- Console spams "Exception sending a multicast message:Network is unreachable"
- Appears hundreds of times

**Solution:**

**This is NORMAL and can be IGNORED!**

These are harmless Gazebo networking messages. They don't affect functionality.

To reduce spam, you can:
```bash
# Redirect stderr to /dev/null (not recommended)
./run_astar_with_mapping.sh 2>/dev/null
```

Or just ignore them - they don't break anything.

---

### 8. Robot Gets Stuck

**Symptoms:**
- Robot stops in middle of path
- Keeps rotating in place
- Path exists but not following

**Solutions:**

```bash
# Check if path is valid
ros2 topic echo /planned_path --once

# Verify robot position
ros2 topic echo /odom

# Check for obstacles
ros2 topic echo /scan --once

# Send new goal
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0}}}'
```

**Common Causes:**
- Detected obstacle (dynamic replanning disabled)
- Waypoint unreachable due to precision
- Angular velocity too low

**Fix:**
1. Increase angular speed:
   - Edit launch file
   - Change `angular_speed: 0.8` (from 0.5)
2. Reduce goal tolerance:
   - Change `goal_tolerance: 0.2` (from 0.3)

---

### 9. High CPU Usage

**Symptoms:**
- Computer slows down significantly
- Fans running loud
- Gazebo lagging

**Solutions:**

```bash
# Check CPU usage
htop
# Look for gz, rviz2, python3 processes

# Reduce point cloud size
# Edit launch file: max_points: 50000 (from 100000)

# Lower update rates
# In mapper: map_timer period to 2.0 (from 1.0)

# Close unnecessary applications

# Reduce Gazebo graphics quality
# In Gazebo: View -> Graphics Quality -> Low
```

---

### 10. World File Not Found

**Symptoms:**
- Error: "World file not found"
- Lists available worlds but yours isn't there

**Solutions:**

```bash
# Check if file exists
ls -la scan_project/human_models/ros_humans_ros2/worlds/

# Rebuild to copy world files
cd ~/ros2_ws
colcon build --symlink-install --packages-select ros_humans_ros2
source install/setup.bash

# Verify installation
ros2 pkg prefix ros_humans_ros2
ls $(ros2 pkg prefix ros_humans_ros2)/share/ros_humans_ros2/worlds/

# Use absolute path
./run_astar_with_mapping.sh /full/path/to/your.world
```

---

## Diagnostic Commands

### System Status
```bash
# Check ROS2 environment
echo $ROS_DISTRO  # Should show: jazzy
printenv | grep ROS

# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Check topic rates
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /cmd_vel
```

### Node Information
```bash
# A* planner details
ros2 node info /astar_path_planner

# Point cloud mapper details
ros2 node info /pointcloud_mapper

# List parameters
ros2 param list /astar_path_planner
ros2 param get /astar_path_planner linear_speed
```

### Topic Inspection
```bash
# View topic info
ros2 topic info /planned_path

# Echo topic data
ros2 topic echo /goal_pose
ros2 topic echo /odom --once

# Monitor bandwidth
ros2 topic bw /scan_pointcloud
```

### TF Debugging
```bash
# View transform tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo map base_footprint

# List all frames
ros2 run tf2_tools echo_all_transforms
```

---

## Performance Tuning

### For Slower Computers
```yaml
# Reduce point cloud size
max_points: 50000

# Lower update rates
map_timer: 2.0  # Instead of 1.0

# Smaller grid
grid_size: 80

# Coarser resolution
grid_resolution: 0.25
map_resolution: 0.15
```

### For Faster Navigation
```yaml
# Increase speeds
linear_speed: 0.5
angular_speed: 0.8

# Reduce tolerance
goal_tolerance: 0.2

# Fine grid (if computer can handle it)
grid_resolution: 0.15
```

---

## Getting Help

If issues persist:

1. **Check logs**:
   ```bash
   cd ~/ros2_ws
   cat log/latest_build/ros_humans_ros2/stdout_stderr.log
   ```

2. **Enable verbose output**:
   ```bash
   ros2 run ros_humans_ros2 astar_path_planner --ros-args --log-level debug
   ```

3. **Capture diagnostics**:
   ```bash
   ros2 node list > nodes.txt
   ros2 topic list > topics.txt
   ros2 param list /astar_path_planner > params.txt
   ```

4. **Check system requirements**:
   - Ubuntu 22.04 or 24.04
   - ROS2 Jazzy
   - 4GB RAM minimum (8GB recommended)
   - GPU for better Gazebo performance

---

## Quick Fixes Summary

| Problem | Quick Fix |
|---------|-----------|
| Robot not moving | Wait 30s, resend goal |
| No path found | Try closer goal, reduce inflation |
| Gazebo won't start | Kill processes: `pkill -f gazebo` |
| RViz empty | Check Fixed Frame = "map" |
| Build fails | Install dependencies, clean build |
| Network errors | Ignore, they're harmless |
| Robot stuck | Send new goal, increase angular speed |
| High CPU | Reduce max_points, lower update rates |

---

**Still having issues?** Check the complete documentation:
- [README_ASTAR.md](README_ASTAR.md)
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md)
- [SYSTEM_FLOW.md](SYSTEM_FLOW.md)

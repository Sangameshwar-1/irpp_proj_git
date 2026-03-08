# IRPP Social Navigation — Docker Setup

## Quick Start

### 1. Build the image

```bash
cd docker/
./build.sh
```

This builds a Docker image (`irpp-social-nav:latest`) with:
- Ubuntu 24.04 + ROS 2 Jazzy Desktop
- Gazebo Harmonic + ros_gz bridge
- The `ros_humans_ros2` package (pre-built)
- All Python dependencies (numpy, OpenCV, PIL, cv_bridge)

### 2. Run

```bash
# Interactive shell (explore, run commands manually)
./run.sh

# Full navigation stack (5 humans, GT perception)
./run.sh navigation

# Full navigation with camera-based perception
./run.sh navigation camera

# Custom goal
./run.sh navigation no_camera 8.0 3.0

# Isolated test case
./run.sh test case1     # static blocker
./run.sh test case2     # head-on
./run.sh test case3     # crossing
./run.sh test case4a    # ahead same direction
./run.sh test case5     # fast from behind

# RViz only (connect to a running container)
./run.sh rviz

# Stop everything
./run.sh stop
```

### 3. Inside the container (interactive shell)

```bash
# Everything is already sourced. Just run:
ros2 launch ros_humans_ros2 navigation.launch.py

# Or generate a map first:
ros2 run ros_humans_ros2 world_to_map \
    $(ros2 pkg prefix ros_humans_ros2)/share/ros_humans_ros2/worlds/large_messy_room.world \
    -o /root/ros2_maps/large_messy_room_map -r 0.05 --inflate 0.10 --padding 1.5

# Then launch with that map:
ros2 launch ros_humans_ros2 navigation.launch.py \
    map_yaml:=/root/ros2_maps/large_messy_room_map.yaml
```

---

## GPU Support

### Intel / AMD (Mesa)

Works out of the box — `/dev/dri` is mounted by default.

### NVIDIA

1. Install [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
2. Uncomment the NVIDIA section in `docker-compose.yml`:

```yaml
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: all
          capabilities: [gpu]
```

### No GPU (software rendering)

Uncomment in `docker-compose.yml`:
```yaml
- LIBGL_ALWAYS_SOFTWARE=1
```

---

## Display / GUI

The container uses **X11 forwarding** via `/tmp/.X11-unix`.  
The `run.sh` script automatically runs `xhost +local:docker` before starting.

If you get display errors:
```bash
# Allow X11 access manually
xhost +local:docker

# Check DISPLAY is set
echo $DISPLAY
```

---

## Persistent Data

Generated maps are stored in a Docker volume (`irpp-maps`) so they persist
across container restarts. To clear them:

```bash
docker volume rm docker_irpp-maps
```

---

## File Structure

```
docker/
├── Dockerfile              ← Multi-stage build (ROS 2 Jazzy + Gazebo)
├── docker-compose.yml      ← Service definition (X11, GPU, volumes)
├── entrypoint.sh           ← Sources ROS 2 + workspace, generates default map
├── build.sh                ← Build helper script
├── run.sh                  ← Run helper script (navigation/test/rviz/shell)
├── .dockerignore           ← Excludes build artifacts from image
├── README.md               ← This file
├── run_navigation.sh       ← Copied from project root
├── run_test_case.sh        ← Copied from project root
├── run_test_scenario.sh    ← Copied from project root
└── scan_project/           ← Full copy of source code
    ├── rviz_config.rviz
    └── human_models/
        └── ros_humans_ros2/    ← ROS 2 package (Python source + worlds + launch)
```

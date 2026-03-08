# How to Run — IRPP Social Navigation (Docker)

## Prerequisites

- **Docker** and **Docker Compose** installed
- **X11** display server running (standard on Ubuntu desktop)
- A GPU with working OpenGL drivers (Intel/AMD Mesa or NVIDIA)

## Quick Start

```bash
cd docker/

# 1. Build the Docker image (one-time)
docker compose build

# 2. Run (pick a mode below)
./run.sh navigation
```

> Every mode auto-launches **Gazebo GUI + RViz + Camera feed**.  
> No flags needed — visuals are always on.

---

## Run Modes

### Navigation (full stack)

```bash
# Default: camera perception, goal (5.0, 5.0)
./run.sh navigation

# Ground-truth perception
./run.sh navigation no_camera

# Custom goal
./run.sh navigation camera 8.0 3.0
```

### Test Cases (isolated arena)

```bash
./run.sh test case1      # Static Blocker — robot reroutes around stationary human
./run.sh test case2      # Head-On — human walks toward robot, robot slows/stops
./run.sh test case3      # Crossing — collision cone, robot slows to pass behind
./run.sh test case4a     # Same direction ahead — robot matches speed
./run.sh test case4b     # Same direction behind — robot gives way
./run.sh test case5      # Fast from behind — robot replans
./run.sh test case6a     # Diagonal crossing (upper-left)
./run.sh test case6b     # Diagonal crossing (lower-left)
./run.sh test case6c     # Diagonal approach (upper-right)
./run.sh test case6d     # Diagonal approach (lower-right)
```

### Scenarios (custom worlds)

```bash
./run.sh scenario corridor_blocked    # 10m corridor — human blocking centre
./run.sh scenario l_corridor          # L-shaped corridor — human at inner corner
./run.sh scenario open_room           # 8m×8m room — human on diagonal + obstacle
./run.sh scenario doorway             # Two rooms + doorway — human near doorway
```

### Other

```bash
./run.sh shell     # Interactive bash inside the container
./run.sh stop      # Stop all running containers
```

---

## What Happens When You Run

1. **X11 access** is granted to Docker (`xhost +local:docker`)
2. Stale Gazebo / container processes are killed
3. The container starts with GPU passthrough + host networking
4. A **map is auto-generated** from the world file
5. **Gazebo** simulation launches (with the robot + humans)
6. **RViz** opens after ~6 seconds (point cloud, path, markers)
7. The **navigation stack** plans and drives the robot

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Black Gazebo viewport | Uncomment `LIBGL_ALWAYS_SOFTWARE=1` in `docker-compose.yml` |
| `cannot open display` | Run `xhost +local:docker` on host, check `$DISPLAY` is set |
| NVIDIA GPU | Uncomment the `deploy.resources` NVIDIA section in `docker-compose.yml` |
| "9 poles" default world appears | Run `./run.sh stop` first, then retry |
| Permission denied on `run.sh` | `chmod +x run.sh` |
| Old container still running | `./run.sh stop` then retry |

---

## Rebuilding After Code Changes

```bash
cd docker/
docker compose build          # re-builds (uses cache)
docker compose build --no-cache   # full rebuild (if cache is stale)
```

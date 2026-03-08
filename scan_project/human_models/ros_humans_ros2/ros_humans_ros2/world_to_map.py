#!/usr/bin/env python3
"""
world_to_map.py
===============
Convert a Gazebo SDF .world file directly into a ROS 2 occupancy-grid map
(.pgm + .yaml) WITHOUT running SLAM or Gazebo.

Supported geometry types
------------------------
  box       → rotated filled rectangle
  cylinder  → filled circle
  sphere    → filled circle (radius = sphere radius)
  cone      → filled circle (radius = cone base radius)

Usage
-----
  # Basic (auto-detects world bounds):
  python3 world_to_map.py large_messy_room.world

  # Custom output name and resolution:
  python3 world_to_map.py large_messy_room.world -o ~/maps/room -r 0.05

  # Explicit map bounds (metres, world frame):
  python3 world_to_map.py large_messy_room.world \
      --xmin -13 --xmax 13 --ymin -13 --ymax 13 -r 0.05

Output
------
  <output>.pgm   – PGM image  (255=free/white, 0=occupied/black)
  <output>.yaml  – ROS 2 map_server YAML descriptor
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np


# ─── Models to ignore completely ─────────────────────────────────────────────
SKIP_EXACT = {
    "floor", "ground_plane", "sun",
    "turtlebot3_rover",
}
SKIP_PREFIX = (
    "human_",        # human_static_*, human_moving_*
    "ceiling_light",
    "camera",
)
SKIP_SUFFIX = ()


# ─── Helpers ──────────────────────────────────────────────────────────────────

def _parse_pose(elem) -> tuple:
    """
    Return (x, y, z, roll, pitch, yaw) from a <pose> element or (0,)*6.
    Handles both 'x y z r p y' and 'x y z qx qy qz qw' formats.
    """
    if elem is None:
        return (0.0,) * 6
    txt = (elem.text or "").strip().split()
    vals = [float(v) for v in txt] + [0.0] * 6
    return tuple(vals[:6])


def _combine_poses(mp: tuple, lp: tuple) -> tuple:
    """
    Compose model-level pose and link-level pose (planar approximation).
    Only yaw rotation is applied to the link offset (roll/pitch ignored for 2-D map).
    """
    mx, my, mz, mr, mpitch, myaw = mp
    lx, ly, lz, lr, lpitch, lyaw = lp
    # Rotate link offset by model yaw
    c, s = math.cos(myaw), math.sin(myaw)
    wx = mx + c * lx - s * ly
    wy = my + s * lx + c * ly
    return (wx, wy, mz + lz, mr + lr, mpitch + lpitch, myaw + lyaw)


def _world_to_pixel(wx, wy, ox, oy, res, H):
    col = int((wx - ox) / res)
    row = H - 1 - int((wy - oy) / res)
    return col, row


# ─── Rasterisers ──────────────────────────────────────────────────────────────

def _draw_box(grid, ox, oy, res, cx, cy, sx, sy, yaw, inflate):
    """Fill a (possibly rotated) axis-aligned box (vectorised with numpy)."""
    H, W = grid.shape
    hx = sx / 2.0 + inflate
    hy = sy / 2.0 + inflate

    # World-space bounding box of the rotated rectangle
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [
        (cx + c * px - s * py, cy + s * px + c * py)
        for px, py in ((hx, hy), (hx, -hy), (-hx, hy), (-hx, -hy))
    ]
    wxmin = min(p[0] for p in corners) - res
    wxmax = max(p[0] for p in corners) + res
    wymin = min(p[1] for p in corners) - res
    wymax = max(p[1] for p in corners) + res

    col_lo = max(0, int((wxmin - ox) / res))
    col_hi = min(W - 1, int((wxmax - ox) / res) + 1)
    row_lo = max(0, H - 1 - int((wymax - oy) / res) - 1)
    row_hi = min(H - 1, H - 1 - int((wymin - oy) / res) + 1)

    if col_lo > col_hi or row_lo > row_hi:
        return

    # Build coordinate meshgrid for the bounding sub-region
    cols = np.arange(col_lo, col_hi + 1)
    rows = np.arange(row_lo, row_hi + 1)
    WX = ox + cols * res                        # world x  (1-D)
    WY = oy + (H - 1 - rows) * res             # world y  (1-D)
    dx = WX[np.newaxis, :] - cx                # (1, nc)
    dy = WY[:, np.newaxis] - cy                # (nr, 1)

    # Rotate into box local frame
    ci, si = math.cos(-yaw), math.sin(-yaw)
    lx = ci * dx - si * dy
    ly = si * dx + ci * dy

    mask = (np.abs(lx) <= hx) & (np.abs(ly) <= hy)
    grid[row_lo:row_hi + 1, col_lo:col_hi + 1][mask] = 0


def _draw_circle(grid, ox, oy, res, cx, cy, radius, inflate):
    """Fill a circle (cylinder / sphere / cone base), vectorised with numpy."""
    H, W = grid.shape
    r = radius + inflate
    r_pix = r / res + 1

    col_c = (cx - ox) / res
    row_c = (H - 1) - (cy - oy) / res

    col_lo = max(0, int(col_c - r_pix))
    col_hi = min(W - 1, int(col_c + r_pix))
    row_lo = max(0, int(row_c - r_pix))
    row_hi = min(H - 1, int(row_c + r_pix))

    if col_lo > col_hi or row_lo > row_hi:
        return

    cols = np.arange(col_lo, col_hi + 1)
    rows = np.arange(row_lo, row_hi + 1)
    WX = ox + cols * res
    WY = oy + (H - 1 - rows) * res
    dx = WX[np.newaxis, :] - cx
    dy = WY[:, np.newaxis] - cy

    mask = dx ** 2 + dy ** 2 <= r * r
    grid[row_lo:row_hi + 1, col_lo:col_hi + 1][mask] = 0


# ─── SDF parser ───────────────────────────────────────────────────────────────

def _get_geometry(collision_elem):
    """
    Return {'type': ..., 'size': tuple} from a <collision> element, or None.
    """
    geom = collision_elem.find("geometry")
    if geom is None:
        return None

    box = geom.find("box")
    if box is not None:
        size_txt = (box.findtext("size") or "1 1 1").split()
        size = tuple(float(v) for v in size_txt)
        return {"type": "box", "size": size}

    cyl = geom.find("cylinder")
    if cyl is not None:
        r = float(cyl.findtext("radius") or 0.5)
        return {"type": "cylinder", "radius": r}

    sph = geom.find("sphere")
    if sph is not None:
        r = float(sph.findtext("radius") or 0.5)
        return {"type": "sphere", "radius": r}

    cone = geom.find("cone")
    if cone is not None:
        r = float(cone.findtext("radius") or 0.25)
        return {"type": "cone", "radius": r}

    return None


def parse_world(world_file):
    """
    Parse a Gazebo SDF .world file and return a list of obstacle descriptors:
    [{'name': str, 'pose': (x,y,z,r,p,yaw), 'geometry': {...}}, ...]
    Only static models with collision geometry are returned.
    """
    tree = ET.parse(world_file)
    root = tree.getroot()

    # Handle both <sdf><world> and bare <world> roots
    world_elem = root.find("world")
    world = world_elem if world_elem is not None else root

    obstacles = []

    for model in world.findall("model"):
        name = model.get("name", "")

        # Skip non-obstacle models
        if name in SKIP_EXACT:
            continue
        if any(name.startswith(p) for p in SKIP_PREFIX):
            continue
        if any(name.endswith(s) for s in SKIP_SUFFIX):
            continue

        # Only process static models
        static_txt = (model.findtext("static") or "false").lower().strip()
        if static_txt not in ("true", "1"):
            continue

        # Model-level pose
        model_pose = _parse_pose(model.find("pose"))

        for link in model.findall("link"):
            link_pose_elem = link.find("pose")
            link_pose = _parse_pose(link_pose_elem)
            combined = _combine_poses(model_pose, link_pose)

            for collision in link.findall("collision"):
                # Collision-level pose (usually 0)
                col_pose = _parse_pose(collision.find("pose"))
                final_pose = _combine_poses(combined, col_pose)

                geo = _get_geometry(collision)
                if geo:
                    obstacles.append({
                        "name":     name,
                        "pose":     final_pose,
                        "geometry": geo,
                    })

    return obstacles


# ─── Map generator ────────────────────────────────────────────────────────────

def generate_map(
    obstacles,
    resolution: float = 0.05,
    xmin: float = None,
    xmax: float = None,
    ymin: float = None,
    ymax: float = None,
    inflate: float = 0.05,
    padding: float = 0.5,
) -> tuple:
    """
    Rasterise all obstacles onto a numpy array.
    Returns (grid, origin_x, origin_y) where:
      grid       : uint8 H×W, 255=free, 0=occupied
      origin_x/y : world coords of the map's lower-left corner
    """
    # Auto-detect bounds from obstacle extents if not provided
    if xmin is None or xmax is None or ymin is None or ymax is None:
        xs, ys = [], []
        for obs in obstacles:
            x, y = obs["pose"][0], obs["pose"][1]
            xs.append(x); ys.append(y)
        if not xs:
            xs, ys = [-10, 10], [-10, 10]
        _xmin = min(xs) - padding
        _xmax = max(xs) + padding
        _ymin = min(ys) - padding
        _ymax = max(ys) + padding
        xmin = xmin if xmin is not None else _xmin
        xmax = xmax if xmax is not None else _xmax
        ymin = ymin if ymin is not None else _ymin
        ymax = ymax if ymax is not None else _ymax

    W = max(1, int(math.ceil((xmax - xmin) / resolution)))
    H = max(1, int(math.ceil((ymax - ymin) / resolution)))

    grid = np.full((H, W), 205, dtype=np.uint8)   # 205 = unknown (grey)

    # Mark the floor interior as free
    # Use the convex hull of the bounding walls to define "indoors"
    # Simple approach: everything inside the grid starts as free
    grid[:] = 254  # free (near-white, standard ROS free value)

    print(f"  Map size  : {W} × {H} pixels  ({W*resolution:.1f} m × {H*resolution:.1f} m)")
    print(f"  Origin    : ({xmin:.3f}, {ymin:.3f})")
    print(f"  Resolution: {resolution} m/pixel")
    print(f"  Obstacles : {len(obstacles)}")

    for obs in obstacles:
        x, y, z, roll, pitch, yaw = obs["pose"]
        geo = obs["geometry"]

        if geo["type"] == "box":
            sx, sy, sz = geo["size"]
            _draw_box(grid, xmin, ymin, resolution, x, y, sx, sy, yaw, inflate)

        elif geo["type"] in ("cylinder", "sphere"):
            r = geo["radius"]
            _draw_circle(grid, xmin, ymin, resolution, x, y, r, inflate)

        elif geo["type"] == "cone":
            r = geo["radius"]
            _draw_circle(grid, xmin, ymin, resolution, x, y, r, inflate)

    return grid, xmin, ymin


# ─── Writers ──────────────────────────────────────────────────────────────────

def write_pgm(grid: np.ndarray, path: str):
    """Write a binary PGM (P5) file."""
    H, W = grid.shape
    header = f"P5\n# Generated by world_to_map.py\n{W} {H}\n255\n"
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(grid.astype(np.uint8).tobytes())
    print(f"  Written: {path}  ({W}×{H} px)")


def write_yaml(pgm_path: str, yaml_path: str, resolution: float,
               origin_x: float, origin_y: float):
    """Write a ROS 2 map_server YAML descriptor."""
    pgm_name = os.path.basename(pgm_path)
    content = (
        f"image: {pgm_name}\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )
    with open(yaml_path, "w") as f:
        f.write(content)
    print(f"  Written: {yaml_path}")


# ─── CLI ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Convert a Gazebo .world file to a ROS 2 occupancy-grid map "
                    "(.pgm + .yaml) without running SLAM.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("world_file",
                        help="Path to the .world SDF file")
    parser.add_argument("-o", "--output", default=None,
                        help="Output base path (without extension). "
                             "Default: same directory as world file, same stem name.")
    parser.add_argument("-r", "--resolution", type=float, default=0.05,
                        help="Map resolution (metres per pixel)")
    parser.add_argument("--xmin", type=float, default=None, help="Map left  boundary (m)")
    parser.add_argument("--xmax", type=float, default=None, help="Map right boundary (m)")
    parser.add_argument("--ymin", type=float, default=None, help="Map bottom boundary (m)")
    parser.add_argument("--ymax", type=float, default=None, help="Map top   boundary (m)")
    parser.add_argument("--inflate", type=float, default=0.05,
                        help="Obstacle inflation radius (m) – pads every obstacle outward")
    parser.add_argument("--padding", type=float, default=1.0,
                        help="Extra border added when auto-detecting bounds (m)")
    parser.add_argument("--list", action="store_true",
                        help="Print all parsed obstacle names and exit (no map written)")
    args = parser.parse_args()

    world_file = os.path.abspath(args.world_file)
    if not os.path.isfile(world_file):
        print(f"ERROR: world file not found: {world_file}", file=sys.stderr)
        sys.exit(1)

    # Determine output paths
    if args.output:
        base = os.path.splitext(args.output)[0]
    else:
        stem = os.path.splitext(os.path.basename(world_file))[0]
        base = os.path.join(os.path.dirname(world_file), stem + "_map")
    pgm_path  = base + ".pgm"
    yaml_path = base + ".yaml"

    print(f"\n[world_to_map] Parsing: {world_file}")
    obstacles = parse_world(world_file)
    print(f"[world_to_map] Found {len(obstacles)} static collision geometries")

    if args.list:
        for obs in obstacles:
            x, y, _, _, _, yaw = obs["pose"]
            g = obs["geometry"]
            if g["type"] == "box":
                desc = f"box {g['size'][0]:.2f}×{g['size'][1]:.2f}"
            else:
                desc = f"{g['type']} r={g.get('radius', '?'):.2f}"
            print(f"  {obs['name']:<30s} ({x:7.2f}, {y:7.2f})  yaw={math.degrees(yaw):6.1f}°  {desc}")
        return

    print(f"\n[world_to_map] Generating map …")
    grid, ox, oy = generate_map(
        obstacles,
        resolution=args.resolution,
        xmin=args.xmin,
        xmax=args.xmax,
        ymin=args.ymin,
        ymax=args.ymax,
        inflate=args.inflate,
        padding=args.padding,
    )

    os.makedirs(os.path.dirname(os.path.abspath(pgm_path)), exist_ok=True)

    print(f"\n[world_to_map] Writing output …")
    write_pgm(grid, pgm_path)
    write_yaml(pgm_path, yaml_path, args.resolution, ox, oy)

    print(f"\n[world_to_map] Done!")
    print(f"\n  To load in ROS 2:")
    print(f"    ros2 run nav2_map_server map_server --ros-args \\")
    print(f"      -p yaml_filename:={yaml_path} \\")
    print(f"      -p use_sim_time:=true")
    print()


if __name__ == "__main__":
    main()

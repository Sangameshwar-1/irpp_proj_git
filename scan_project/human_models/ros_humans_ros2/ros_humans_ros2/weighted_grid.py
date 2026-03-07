#!/usr/bin/env python3
"""
Weighted Grid Module
====================
Converts an occupancy map into a graph with weighted edges for A* planning.

Initially ALL traversable edges have equal weight (1.0).  The local planner
can dynamically update edge weights to penalise areas with detected humans,
causing A* to route around them on replan.

Conceptually this is an edge-weighted graph, implemented as cell-entry costs
(equivalent for A*):

    edge_cost(A → B) = move_distance(A,B) × base_weight[B] × dynamic_weight[B]

    base_weight   – static safety cost from obstacle proximity (set once)
    dynamic_weight – human / threat cost (updated by local planner, starts at 1.0)

Usage
-----
    grid = WeightedGrid.from_occupancy_grid(map_msg, planner_resolution=0.2)
    path = grid.astar(start, goal)
    grid.update_dynamic_weights([(wx, wy, radius, multiplier), ...])
    new_path = grid.astar(current_pos, goal)   # reroutes around costly zones
"""

import math
import heapq
import numpy as np


class WeightedGrid:
    """2D grid graph with per-cell edge weights for A* path planning."""

    # 8-connected grid directions: (dx, dy)
    DIRECTIONS = [
        (1, 0), (-1, 0), (0, 1), (0, -1),       # cardinal
        (1, 1), (1, -1), (-1, 1), (-1, -1),      # diagonal
    ]

    def __init__(self, width, height, resolution, origin_x, origin_y):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

        # --- All edges start with EQUAL weight (1.0) ---
        self.base_weights = np.ones((height, width), dtype=np.float32)
        self.dynamic_weights = np.ones((height, width), dtype=np.float32)
        self.obstacles = np.zeros((height, width), dtype=bool)

    # ── Factory ───────────────────────────────────────────────────────────

    @classmethod
    def from_occupancy_grid(cls, msg, planner_resolution=0.2,
                            inflation_radius=0.35):
        """Create a WeightedGrid from a nav_msgs/OccupancyGrid.

        The map is downsampled to *planner_resolution* so A* runs on a
        coarser, faster grid.  Obstacle inflation adds a proximity-based
        base weight near walls so A* prefers paths with clearance.

        Args:
            msg:                 OccupancyGrid message (from map_publisher)
            planner_resolution:  metres per planner cell (≥ map resolution)
            inflation_radius:    metres to inflate obstacles
        """
        map_res = msg.info.resolution
        map_w = msg.info.width
        map_h = msg.info.height
        map_ox = msg.info.origin.position.x
        map_oy = msg.info.origin.position.y

        # Planner grid dimensions (covers same metric area)
        metric_w = map_w * map_res
        metric_h = map_h * map_res
        grid_w = max(1, int(metric_w / planner_resolution))
        grid_h = max(1, int(metric_h / planner_resolution))

        grid = cls(grid_w, grid_h, planner_resolution, map_ox, map_oy)

        # ── Vectorised down-sampling ──────────────────────────────────
        raw = np.array(msg.data, dtype=np.int8).reshape(map_h, map_w)

        gx_idx = np.arange(grid_w)
        gy_idx = np.arange(grid_h)
        # World X/Y of each planner cell centre
        wx = map_ox + (gx_idx + 0.5) * planner_resolution
        wy = map_oy + (gy_idx + 0.5) * planner_resolution
        # Nearest map cell for each planner cell
        mx = np.clip(((wx - map_ox) / map_res).astype(int), 0, map_w - 1)
        my = np.clip(((wy - map_oy) / map_res).astype(int), 0, map_h - 1)
        mx_g, my_g = np.meshgrid(mx, my)
        sampled = raw[my_g, mx_g]

        grid.obstacles = (sampled >= 50)

        # ── Obstacle inflation → base_weights ────────────────────────
        inflation_cells = max(1, int(inflation_radius / planner_resolution))
        # Additional lethal zone (cells treated as obstacles) = 0.5m
        lethal_radius = 0.5
        lethal_cells = max(1, int(lethal_radius / planner_resolution))
        obs_ys, obs_xs = np.where(grid.obstacles)

        # For every obstacle cell, penalise nearby free cells
        for oy, ox in zip(obs_ys, obs_xs):
            y_lo = max(0, oy - inflation_cells)
            y_hi = min(grid_h, oy + inflation_cells + 1)
            x_lo = max(0, ox - inflation_cells)
            x_hi = min(grid_w, ox + inflation_cells + 1)
            for ny in range(y_lo, y_hi):
                for nx in range(x_lo, x_hi):
                    if grid.obstacles[ny, nx]:
                        continue
                    dist = math.hypot(nx - ox, ny - oy) * planner_resolution
                    if dist < lethal_radius:
                        # Too close to wall → treat as obstacle (lethal zone)
                        grid.obstacles[ny, nx] = True
                    elif dist <= inflation_radius:
                        # Proximity penalty (exponential falloff for stronger avoidance)
                        ratio = 1.0 - dist / inflation_radius
                        w = 1.0 + 15.0 * (ratio ** 1.5)  # strong exponential penalty
                        if w > grid.base_weights[ny, nx]:
                            grid.base_weights[ny, nx] = w

        return grid

    # ── Coordinate helpers ────────────────────────────────────────────────

    def world_to_grid(self, x, y):
        """World metres → grid indices."""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Grid indices → world metres (cell centre)."""
        x = self.origin_x + (gx + 0.5) * self.resolution
        y = self.origin_y + (gy + 0.5) * self.resolution
        return x, y

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_free(self, gx, gy):
        return self.in_bounds(gx, gy) and not self.obstacles[gy, gx]

    # ── Edge weights ──────────────────────────────────────────────────────

    def get_cost(self, gx, gy):
        """Total cost to enter cell = base × dynamic."""
        return float(self.base_weights[gy, gx] * self.dynamic_weights[gy, gx])

    def update_dynamic_weights(self, zones):
        """Set dynamic weights from a list of human/threat zones.

        Resets dynamic_weights to 1.0 first, then applies each zone.

        Args:
            zones: list of (world_x, world_y, radius_m, weight_multiplier)
        """
        self.dynamic_weights.fill(1.0)
        for wx, wy, radius, weight in zones:
            self._apply_zone(wx, wy, radius, weight)

    def add_dynamic_weight_zone(self, wx, wy, radius, weight):
        """Add a single zone WITHOUT resetting existing dynamic weights."""
        self._apply_zone(wx, wy, radius, weight)

    def _apply_zone(self, wx, wy, radius, weight):
        cx, cy = self.world_to_grid(wx, wy)
        r_cells = max(1, int(radius / self.resolution))
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                nx, ny = cx + dx, cy + dy
                if self.in_bounds(nx, ny) and not self.obstacles[ny, nx]:
                    dist = math.hypot(dx, dy) * self.resolution
                    if dist <= radius:
                        ratio = 1.0 - dist / max(radius, 0.01)
                        cell_w = 1.0 + (weight - 1.0) * ratio
                        if cell_w > self.dynamic_weights[ny, nx]:
                            self.dynamic_weights[ny, nx] = cell_w

    def reset_dynamic_weights(self):
        """Clear all dynamic costs (used before applying fresh zones)."""
        self.dynamic_weights.fill(1.0)

    # ── A* pathfinding ────────────────────────────────────────────────────

    def get_neighbors(self, node):
        """8-connected neighbors with weighted edge costs."""
        gx, gy = node
        neighbors = []
        for dx, dy in self.DIRECTIONS:
            nx, ny = gx + dx, gy + dy
            if self.is_free(nx, ny):
                move_dist = 1.4142 if (dx != 0 and dy != 0) else 1.0
                edge_cost = move_dist * self.get_cost(nx, ny)
                neighbors.append(((nx, ny), edge_cost))
        return neighbors

    @staticmethod
    def heuristic(a, b):
        """Octile distance — tight admissible heuristic for 8-connected grids."""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return min(dx, dy) * 1.4142 + abs(dx - dy)

    def astar(self, start, goal, goal_tolerance=2):
        """Run A* on the weighted grid.

        Returns list of (gx, gy) cells forming the path, or None.
        """
        if not self.is_free(*start):
            # Nudge start to nearest free cell
            start = self._nearest_free(start)
            if start is None:
                return None
        if not self.is_free(*goal):
            goal = self._nearest_free(goal)
            if goal is None:
                return None

        counter = 0
        open_set = [(0.0, counter, start)]
        came_from = {}
        g_score = {start: 0.0}
        visited = set()
        max_iter = self.width * self.height * 2

        for _ in range(max_iter):
            if not open_set:
                break
            _f, _, current = heapq.heappop(open_set)
            if current in visited:
                continue
            visited.add(current)

            if math.hypot(current[0] - goal[0],
                          current[1] - goal[1]) <= goal_tolerance:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for nbr, cost in self.get_neighbors(current):
                if nbr in visited:
                    continue
                tent_g = g_score[current] + cost
                if nbr not in g_score or tent_g < g_score[nbr]:
                    came_from[nbr] = current
                    g_score[nbr] = tent_g
                    f = tent_g + self.heuristic(nbr, goal)
                    counter += 1
                    heapq.heappush(open_set, (f, counter, nbr))

        return None  # no path

    def _nearest_free(self, cell, radius=5):
        """Find nearest free cell within *radius* of *cell*."""
        gx, gy = cell
        for r in range(1, radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) == r or abs(dy) == r:
                        nx, ny = gx + dx, gy + dy
                        if self.is_free(nx, ny):
                            return (nx, ny)
        return None

    # ── Path utilities ────────────────────────────────────────────────────

    def path_distance(self, grid_path):
        """Total Euclidean distance of a grid path in metres."""
        total = 0.0
        for i in range(1, len(grid_path)):
            dx = grid_path[i][0] - grid_path[i - 1][0]
            dy = grid_path[i][1] - grid_path[i - 1][1]
            total += math.hypot(dx, dy) * self.resolution
        return total

    def simplify_path(self, grid_path):
        """Convert grid path → simplified world-coord path.

        1. Grid → world coordinates
        2. Line-of-sight pruning (removes A* staircase)
        3. Corner smoothing (gentle curves at sharp turns)
        """
        if not grid_path:
            return []

        world = [self.grid_to_world(gx, gy) for gx, gy in grid_path]
        if len(world) <= 2:
            return world

        # ── LOS pruning ──
        pruned = [world[0]]
        i = 0
        while i < len(world) - 1:
            best = i + 1
            for j in range(len(world) - 1, i + 1, -1):
                if self._los_grid(grid_path[i], grid_path[j]):
                    best = j
                    break
            pruned.append(world[best])
            i = best

        # ── Corner smoothing ──
        if len(pruned) < 3:
            return pruned
        smoothed = [pruned[0]]
        for k in range(1, len(pruned) - 1):
            prev, curr, nxt = smoothed[-1], pruned[k], pruned[k + 1]
            d1 = math.atan2(curr[1] - prev[1], curr[0] - prev[0])
            d2 = math.atan2(nxt[1] - curr[1], nxt[0] - curr[0])
            angle = abs(d1 - d2)
            if angle > math.pi:
                angle = 2 * math.pi - angle
            if angle > 0.4:  # smooth even gentler turns
                # Approach point — further back for wider arc
                smooth_dist = min(0.6, 0.15 + angle * 0.25)  # scale offset with turn sharpness
                dist_in = math.hypot(curr[0] - prev[0], curr[1] - prev[1])
                if dist_in > 0.4:
                    r = min(0.4, smooth_dist / dist_in)
                    approach = (curr[0] + (prev[0] - curr[0]) * r,
                                curr[1] + (prev[1] - curr[1]) * r)
                    smoothed.append(approach)
                # Add a Bezier-like mid-point for very sharp turns (>60°)
                if angle > 1.05:
                    dist_out = math.hypot(nxt[0] - curr[0], nxt[1] - curr[1])
                    if dist_in > 0.4 and dist_out > 0.4:
                        r_in = min(0.35, smooth_dist / dist_in)
                        r_out = min(0.35, smooth_dist / dist_out)
                        p0 = (curr[0] + (prev[0] - curr[0]) * r_in,
                              curr[1] + (prev[1] - curr[1]) * r_in)
                        p2 = (curr[0] + (nxt[0] - curr[0]) * r_out,
                              curr[1] + (nxt[1] - curr[1]) * r_out)
                        mid = (0.25 * p0[0] + 0.5 * curr[0] + 0.25 * p2[0],
                               0.25 * p0[1] + 0.5 * curr[1] + 0.25 * p2[1])
                        smoothed.append(mid)
                else:
                    smoothed.append(curr)
                # Depart point — push into next segment
                dist_out = math.hypot(nxt[0] - curr[0], nxt[1] - curr[1])
                if dist_out > 0.4:
                    r = min(0.4, smooth_dist / dist_out)
                    depart = (curr[0] + (nxt[0] - curr[0]) * r,
                              curr[1] + (nxt[1] - curr[1]) * r)
                    smoothed.append(depart)
            else:
                smoothed.append(curr)
        smoothed.append(pruned[-1])
        return smoothed

    def _los_grid(self, a, b):
        """Safety-aware line-of-sight check between two grid cells.

        Rejects lines that pass through obstacles, high-cost base cells
        (near walls), OR cells with elevated dynamic weights (human
        zones).  This prevents the path simplifier from collapsing
        A*-detours around humans back into a straight line.
        """
        BASE_COST_THRESHOLD = 3.0   # reject through inflated wall cells
        DYN_COST_THRESHOLD  = 1.5   # reject through human weight zones
        x0, y0 = a
        x1, y1 = b
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            if not self.in_bounds(x, y) or self.obstacles[y, x]:
                return False
            # Block LOS through high-cost cells (near-wall proximity)
            if self.base_weights[y, x] >= BASE_COST_THRESHOLD:
                return False
            # Block LOS through human weight zones (dynamic weights)
            if self.dynamic_weights[y, x] >= DYN_COST_THRESHOLD:
                return False
            # Check 1-cell lateral band for obstacles (safety corridor)
            for ox, oy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + ox, y + oy
                if self.in_bounds(nx, ny) and self.obstacles[ny, nx]:
                    return False
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return True

    # ── Visualisation export ──────────────────────────────────────────────

    def to_occupancy_data(self):
        """Export total cost as a flat list for OccupancyGrid visualisation.

        Maps:  obstacle → 100,  high cost → 1-80,  free → 0
        """
        total = self.base_weights * self.dynamic_weights
        out = np.zeros((self.height, self.width), dtype=np.int8)
        free = ~self.obstacles
        if free.any():
            max_c = max(float(total[free].max()), 1.01)
            normed = (total - 1.0) / (max_c - 1.0) * 80.0
            out[free] = np.clip(normed[free], 0, 80).astype(np.int8)
        out[self.obstacles] = 100
        return out.flatten().tolist()

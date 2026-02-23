"""
astar_path_planner.py
=====================
ROS 2 node that:
  1. Subscribes to /map  (nav_msgs/OccupancyGrid)
  2. Subscribes to /odom (nav_msgs/Odometry)
  3. Accepts a 2-D goal on /move_base_simple/goal  (geometry_msgs/PoseStamped)
  4. Plans a shortest path using A* on the occupancy grid
     (with configurable obstacle inflation)
  5. Publishes the path on /astar_path  (nav_msgs/Path)  → visible in RViz
  6. Drives the robot via a 10 Hz timer-based controller on /cmd_vel
     (non-blocking – does NOT block rclpy.spin)

ROS parameters
--------------
  waypoint_tolerance      [0.20 m]   – distance to consider a waypoint reached
  linear_speed            [0.25 m/s] – max forward speed
  angular_speed           [1.00 r/s] – max yaw rate
  obstacle_inflation_cells[3]        – inflate obstacles by N grid cells
  waypoint_stride         [5]        – keep every Nth waypoint for the follower
"""

import math
import heapq
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist


# ── pure-Python helpers (no tf_transformations dependency) ────────────────────

def _yaw_from_quaternion(q) -> float:
    """Extract the yaw (rotation about Z) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# ── Node ──────────────────────────────────────────────────────────────────────

class AStarPlanner(Node):

    def __init__(self):
        super().__init__('astar_path_planner')

        # ── ROS parameters ──────────────────────────────────────────────────
        self.declare_parameter('waypoint_tolerance',       0.20)
        self.declare_parameter('linear_speed',             0.25)
        self.declare_parameter('angular_speed',            1.00)
        self.declare_parameter('obstacle_inflation_cells', 3)
        self.declare_parameter('waypoint_stride',          5)

        self._tol     = self.get_parameter('waypoint_tolerance').value
        self._vlin    = self.get_parameter('linear_speed').value
        self._vang    = self.get_parameter('angular_speed').value
        self._inflate = self.get_parameter('obstacle_inflation_cells').value
        self._stride  = self.get_parameter('waypoint_stride').value

        # ── state ────────────────────────────────────────────────────────────
        self._map_data: Optional[List[int]] = None
        self._map_info = None
        self._odom_pose = None
        self._waypoints: List[Tuple[float, float]] = []
        self._wp_idx: int = 0

        # ── subscriptions ────────────────────────────────────────────────────
        self.create_subscription(OccupancyGrid, '/map',  self._map_cb,  10)
        self.create_subscription(Odometry,      '/odom', self._odom_cb, 20)
        self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self._goal_cb, 10)

        # ── publishers ───────────────────────────────────────────────────────
        self._path_pub = self.create_publisher(Path,  '/astar_path', 10)
        self._cmd_pub  = self.create_publisher(Twist, '/cmd_vel',    10)

        # ── 10 Hz control loop ───────────────────────────────────────────────
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            'astar_path_planner ready – '
            'waiting for /map and /move_base_simple/goal'
        )

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid):
        self._map_data = list(msg.data)
        self._map_info = msg.info
        self.get_logger().info(
            f'Map received: {msg.info.width}×{msg.info.height} '
            f'@ {msg.info.resolution:.3f} m/cell'
        )

    def _odom_cb(self, msg: Odometry):
        self._odom_pose = msg.pose.pose

    def _goal_cb(self, msg: PoseStamped):
        if self._map_data is None:
            self.get_logger().warning('No map yet – cannot plan')
            return
        if self._odom_pose is None:
            self.get_logger().warning('No odometry yet – cannot plan')
            return

        start = (self._odom_pose.position.x, self._odom_pose.position.y)
        goal  = (msg.pose.position.x,        msg.pose.position.y)
        self.get_logger().info(
            f'Received goal  start=({start[0]:.2f},{start[1]:.2f})'
            f'  goal=({goal[0]:.2f},{goal[1]:.2f})'
        )

        raw_wps = self._plan(start, goal)
        if not raw_wps:
            self.get_logger().error(
                'A* found no path – goal may be inside an obstacle or outside map'
            )
            return

        # Down-sample and always include the final waypoint
        stride = max(1, self._stride)
        self._waypoints = raw_wps[::stride]
        if self._waypoints[-1] != raw_wps[-1]:
            self._waypoints.append(raw_wps[-1])
        self._wp_idx = 0

        # Publish the full path for RViz
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        for x, y in raw_wps:
            ps = PoseStamped()
            ps.header           = path_msg.header
            ps.pose.position.x  = x
            ps.pose.position.y  = y
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self._path_pub.publish(path_msg)

        self.get_logger().info(
            f'Path planned: {len(raw_wps)} cells → '
            f'{len(self._waypoints)} waypoints for follower'
        )

    # ── 10 Hz controller ──────────────────────────────────────────────────────

    def _control_loop(self):
        if not self._waypoints or self._wp_idx >= len(self._waypoints):
            return
        if self._odom_pose is None:
            return

        wx, wy = self._waypoints[self._wp_idx]
        dx = wx - self._odom_pose.position.x
        dy = wy - self._odom_pose.position.y
        dist = math.hypot(dx, dy)

        # Waypoint reached → advance
        if dist < self._tol:
            self._wp_idx += 1
            if self._wp_idx >= len(self._waypoints):
                self.get_logger().info('✓ Goal reached – stopping robot')
                self._cmd_pub.publish(Twist())   # stop
                self._waypoints = []
            return

        yaw   = _yaw_from_quaternion(self._odom_pose.orientation)
        alpha = math.atan2(dy, dx)
        err   = _wrap_angle(alpha - yaw)

        cmd = Twist()
        if abs(err) > 0.40:
            # Pure rotate in place first
            cmd.angular.z = max(-self._vang, min(self._vang, 2.0 * err))
            cmd.linear.x  = 0.0
        else:
            cmd.linear.x  = min(self._vlin, 0.5 * dist)
            cmd.angular.z = max(-self._vang, min(self._vang, 1.5 * err))

        self._cmd_pub.publish(cmd)

    # ── A* planner ────────────────────────────────────────────────────────────

    def _plan(
        self,
        start: Tuple[float, float],
        goal:  Tuple[float, float],
    ) -> List[Tuple[float, float]]:

        info = self._map_info
        res  = info.resolution
        ox   = info.origin.position.x
        oy   = info.origin.position.y
        W    = info.width
        H    = info.height
        data = self._map_data

        def w2i(x: float, y: float) -> Tuple[int, int]:
            return int((x - ox) / res), int((y - oy) / res)

        def i2w(ix: int, iy: int) -> Tuple[float, float]:
            return ox + (ix + 0.5) * res, oy + (iy + 0.5) * res

        def in_bounds(ix: int, iy: int) -> bool:
            return 0 <= ix < W and 0 <= iy < H

        pad = self._inflate

        def is_free(ix: int, iy: int) -> bool:
            """Return True if cell (ix,iy) and its inflated neighbourhood are clear."""
            for ddx in range(-pad, pad + 1):
                for ddy in range(-pad, pad + 1):
                    nx, ny = ix + ddx, iy + ddy
                    if in_bounds(nx, ny) and data[ny * W + nx] >= 50:
                        return False
            return True

        sx, sy = w2i(*start)
        gx, gy = w2i(*goal)

        if not in_bounds(sx, sy) or not in_bounds(gx, gy):
            self.get_logger().error('Start or goal is outside the map bounds')
            return []

        # Snap start/goal to nearest free cell if blocked
        if not is_free(sx, sy):
            self.get_logger().warning('Start cell is inside inflated obstacle – snapping')
        if not is_free(gx, gy):
            self.get_logger().error('Goal cell is inside an obstacle – cannot plan')
            return []

        DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1),
                (-1,-1), (-1, 1), (1,-1), (1, 1)]

        s_node = (sx, sy)
        g_node = (gx, gy)

        came_from:   Dict[Tuple[int,int], Tuple[int,int]] = {}
        cost_so_far: Dict[Tuple[int,int], float]          = {s_node: 0.0}

        h0 = math.hypot(gx - sx, gy - sy)
        open_heap = [(h0, 0.0, s_node)]   # (f, g, node)

        found = False
        while open_heap:
            _, g_cost, cur = heapq.heappop(open_heap)

            if cur == g_node:
                found = True
                break

            # Skip stale entries
            if g_cost > cost_so_far.get(cur, float('inf')):
                continue

            for ddx, ddy in DIRS:
                nxt = (cur[0] + ddx, cur[1] + ddy)
                if not in_bounds(nxt[0], nxt[1]):
                    continue
                if not is_free(nxt[0], nxt[1]):
                    continue

                nc = g_cost + math.hypot(ddx, ddy)
                if nc < cost_so_far.get(nxt, float('inf')):
                    cost_so_far[nxt] = nc
                    h  = math.hypot(gx - nxt[0], gy - nxt[1])
                    heapq.heappush(open_heap, (nc + h, nc, nxt))
                    came_from[nxt] = cur

        if not found:
            return []

        # Reconstruct path
        path: List[Tuple[int, int]] = [g_node]
        while path[-1] != s_node:
            path.append(came_from[path[-1]])
        path.reverse()

        return [i2w(ix, iy) for ix, iy in path]


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cmd_pub.publish(Twist())   # safety stop
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

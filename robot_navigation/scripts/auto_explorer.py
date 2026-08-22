#!/usr/bin/env python3
"""
auto_explorer.py — Autonomous frontier-based exploration untuk ROS 2 Jazzy.

Node ini secara otomatis mengeksplorasi seluruh peta yang belum diketahui dengan
memilih titik frontier (batas antara area bebas dan area yang belum dipetakan)
dan memerintahkan Nav2 untuk bergerak ke sana.

Algoritma:
  1. Subscribe ke /map untuk mendapat OccupancyGrid terbaru.
  2. Temukan semua sel frontier (sel bebas yang bersebelahan dengan sel unknown).
  3. Cluster frontier dan pilih centroid frontier terdekat dari posisi robot.
  4. Kirim tujuan ke Nav2 via NavigateToPose action.
  5. Setelah sampai (atau gagal), ulangi dari langkah 2.
  6. Berhenti jika tidak ada frontier yang tersisa (peta sudah lengkap).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401


UNKNOWN = -1
FREE    = 0
OCCUPIED_THRESHOLD = 50   # sel dengan nilai >= ini dianggap terisi


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        # Params
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('min_frontier_size', 10)      # minimum sel per cluster
        self.declare_parameter('exploration_timeout', 30.0)  # detik per goal
        self.declare_parameter('goal_tolerance', 0.3)        # meter

        self.base_frame  = self.get_parameter('robot_base_frame').value
        self.map_frame   = self.get_parameter('map_frame').value
        self.min_f_size  = self.get_parameter('min_frontier_size').value
        self.timeout_sec = self.get_parameter('exploration_timeout').value

        # TF
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Map subscriber
        self.map_data: OccupancyGrid | None = None
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, 10)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # State
        self._is_navigating  = False
        self._nav2_ready     = False
        self._explored_goals: list[tuple[float, float]] = []

        self.get_logger().info('🚀 Frontier Explorer aktif — menunggu Nav2 aktif dan peta...')

        # Timer cek kesiapan Nav2 (tiap 2 detik) sebelum mulai eksplorasi
        self._readiness_timer = self.create_timer(2.0, self._check_nav2_ready)

    # ------------------------------------------------------------------ #
    # Readiness Check                                                       #
    # ------------------------------------------------------------------ #

    def _check_nav2_ready(self):
        """Tunggu hingga action server navigate_to_pose benar-benar tersedia."""
        if self._nav2_ready:
            return

        server_ready = self._nav_client.wait_for_server(timeout_sec=1.0)
        map_ready    = self.map_data is not None

        if server_ready and map_ready:
            self._nav2_ready = True
            self._readiness_timer.cancel()
            self.get_logger().info('✅ Nav2 aktif dan peta tersedia — eksplorasi dimulai!')
            # Mulai loop eksplorasi utama
            self.create_timer(0.5, self._exploration_loop)
        else:
            status = []
            if not server_ready:
                status.append('Nav2 action server belum ready')
            if not map_ready:
                status.append('/map belum ada')
            self.get_logger().info(f'⏳ Menunggu: {", ".join(status)}...')

    # ------------------------------------------------------------------ #
    # Callbacks                                                             #
    # ------------------------------------------------------------------ #

    def _map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    # ------------------------------------------------------------------ #
    # Main Loop                                                             #
    # ------------------------------------------------------------------ #

    def _exploration_loop(self):
        if self._is_navigating or self.map_data is None:
            return

        robot_pos = self._get_robot_position()
        if robot_pos is None:
            return

        frontiers = self._find_frontiers()
        if not frontiers:
            # Toleransi 5x retry sebelum menyatakan eksplorasi selesai
            self._no_frontier_retry = getattr(self, '_no_frontier_retry', 0) + 1
            if self._no_frontier_retry < 5:
                return
            
            self.get_logger().info('✅ Tidak ada frontier tersisa — eksplorasi selesai!')
            return

        # Reset counter jika frontier ditemukan
        self._no_frontier_retry = 0

        # Pilih frontier terdekat yang belum pernah dikunjungi
        goal = self._select_best_frontier(frontiers, robot_pos)
        if goal is None:
            self._stuck_retry = getattr(self, '_stuck_retry', 0) + 1
            
            # Jika sempat dianggap stuck 3x, reset blacklist agar dicoba ulang setelah costmap Nav2 sync
            if self._stuck_retry == 3:
                if hasattr(self, '_blacklist'):
                    self.get_logger().info('🔄 Mereset blacklist frontier untuk mencoba ulang jalur...')
                    self._blacklist.clear()
                return

            if self._stuck_retry < 6:
                self.get_logger().info(f'⚠️ Menunggu Nav2/Costmap sync... ({self._stuck_retry}/6)', throttle_duration_sec=2.0)
                return

            self.get_logger().warn('⚠️  Semua frontier sudah dicoba — eksplorasi selesai.')
            return

        # Reset counter jika goal valid ditemukan
        self._stuck_retry = 0
        self._send_nav_goal(goal[0], goal[1])

    # ------------------------------------------------------------------ #
    # Frontier Detection                                                    #
    # ------------------------------------------------------------------ #

    def _find_frontiers(self) -> list[tuple[float, float]]:
        """Temukan centroid frontier dari OccupancyGrid."""
        msg    = self.map_data
        width  = msg.info.width
        height = msg.info.height
        data   = msg.data
        res    = msg.info.resolution
        ox     = msg.info.origin.position.x
        oy     = msg.info.origin.position.y

        frontier_cells: list[tuple[int, int]] = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                idx = y * width + x
                if data[idx] != FREE:
                    continue
                # Cek apakah ada sel unknown di 8-tetangga
                is_frontier = False
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        nb = (y + dy) * width + (x + dx)
                        if 0 <= nb < len(data) and data[nb] == UNKNOWN:
                            is_frontier = True
                            break
                    if is_frontier:
                        break
                if is_frontier:
                    frontier_cells.append((x, y))

        if not frontier_cells:
            return []

        # Simple clustering dengan Union-Find sederhana
        clusters = self._cluster_frontiers(frontier_cells, width)

        centroids: list[tuple[float, float]] = []
        for cluster in clusters:
            if len(cluster) < self.min_f_size:
                continue
            cx = sum(c[0] for c in cluster) / len(cluster)
            cy = sum(c[1] for c in cluster) / len(cluster)
            # Konversi ke koordinat world
            wx = ox + (cx + 0.5) * res
            wy = oy + (cy + 0.5) * res
            centroids.append((wx, wy))

        return centroids

    def _cluster_frontiers(
        self,
        cells: list[tuple[int, int]],
        width: int
    ) -> list[list[tuple[int, int]]]:
        """BFS clustering dari sel frontier."""
        cell_set = set(cells)
        visited:  set[tuple[int, int]] = set()
        clusters: list[list[tuple[int, int]]] = []

        for cell in cells:
            if cell in visited:
                continue
            cluster: list[tuple[int, int]] = []
            queue = [cell]
            while queue:
                cur = queue.pop()
                if cur in visited:
                    continue
                visited.add(cur)
                cluster.append(cur)
                x, y = cur
                for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                    nb = (x + dx, y + dy)
                    if nb in cell_set and nb not in visited:
                        queue.append(nb)
            clusters.append(cluster)

        return clusters

    # ------------------------------------------------------------------ #
    # Goal Selection                                                        #
    # ------------------------------------------------------------------ #

    def _select_best_frontier(
        self,
        frontiers: list[tuple[float, float]],
        robot_pos: tuple[float, float]
    ) -> tuple[float, float] | None:
        rx, ry = robot_pos
        tol    = self.get_parameter('goal_tolerance').value

        # Filter yang sudah dikunjungi
        unvisited = [
            f for f in frontiers
            if not any(math.dist(f, g) < tol * 3 for g in self._explored_goals)
        ]
        if not unvisited:
            return None

        # Pilih yang terdekat
        return min(unvisited, key=lambda f: math.dist(f, (rx, ry)))

    # ------------------------------------------------------------------ #
    # Navigation                                                            #
    # ------------------------------------------------------------------ #

    def _send_nav_goal(self, x: float, y: float):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 action server belum ready!')
            return

        goal_msg          = NavigateToPose.Goal()
        pose              = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        goal_msg.pose = pose

        self.get_logger().info(f'📍 Menuju frontier: ({x:.2f}, {y:.2f})')
        self._is_navigating = True
        self._explored_goals.append((x, y))

        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Goal ditolak oleh Nav2.')
            self._is_navigating = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

        # Timeout watchdog
        self.create_timer(
            self.timeout_sec,
            lambda: self._timeout_callback(goal_handle)
        )

    def _result_callback(self, future):
        self._is_navigating = False
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('✅ Sampai di frontier!')
        else:
            self.get_logger().info(f'⚠️  Goal selesai dengan status: {result.status}')

    def _timeout_callback(self, goal_handle):
        if self._is_navigating:
            self.get_logger().warn(f'⏰ Timeout ({self.timeout_sec}s) — membatalkan goal.')
            goal_handle.cancel_goal_async()
            self._is_navigating = False

    # ------------------------------------------------------------------ #
    # TF Utilities                                                          #
    # ------------------------------------------------------------------ #

    def _get_robot_position(self) -> tuple[float, float] | None:
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            return (
                t.transform.translation.x,
                t.transform.translation.y
            )
        except Exception:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

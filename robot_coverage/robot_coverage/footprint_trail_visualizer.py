#!/usr/bin/env python3
"""
footprint_trail_visualizer.py — Real-time Robot Footprint Swept-Area Visualizer & Coverage Metric.

Fitur:
  1. Real-time Swept Area Visualization (RViz TRIANGLE_LIST Marker):
     - Menggambar jejak kaki (footprint/cleaning tool) robot berupa lapisan transparan (hijau stabilo / lime)
       secara mulus di lantai saat robot melaju.
  2. Kalkulasi Metrik Persentase Cakupan Real-Time:
     - Menghitung persentase area lahan yang sudah bersih / tertutup sapuan robot (% Coverage).
     - Menampilkan log statistik sisa area yang belum tersapu.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import PolygonStamped, Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener
import cv2


class FootprintTrailVisualizer(Node):
    def __init__(self):
        super().__init__('footprint_trail_visualizer')

        # ── Parameter ──────────────────────────────────────────────────────────
        self.declare_parameter('tool_width', 0.30)  # Lebar jangkauan sapuan (meter)
        self.declare_parameter('update_dist', 0.04) # Interval minimal pergerakan (meter)
        self.declare_parameter('resolution', 0.03)  # Resolusi kalkulasi metrik area (meter/pixel)

        self.tool_width = self.get_parameter('tool_width').get_parameter_value().double_value
        self.update_dist = self.get_parameter('update_dist').get_parameter_value().double_value
        self.metric_res = self.get_parameter('resolution').get_parameter_value().double_value

        # ── TF Buffer ──────────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── State ─────────────────────────────────────────────────────────────
        self._last_pose = None
        self._triangles = []
        self._field_polygon = []
        self._covered_grid = None
        self._field_mask = None
        self._grid_origin_x = 0.0
        self._grid_origin_y = 0.0
        self._grid_w = 0
        self._grid_h = 0
        self._total_valid_cells = 0
        self._current_map = None

        # ── Publishers & Subscribers ──────────────────────────────────────────
        self.trail_pub = self.create_publisher(Marker, '/coverage_swept_footprint', 10)
        self.percent_pub = self.create_publisher(Float32, '/coverage_progress_percent', 10)
        self.status_pub = self.create_publisher(String, '/coverage_status_text', 10)

        self.poly_sub = self.create_subscription(
            PolygonStamped, '/field_boundary', self._field_boundary_cb, 10)

        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, latch_qos)

        # Timer pelacak jejak pada frekuensi 15 Hz
        self.timer = self.create_timer(1.0 / 15.0, self._track_loop)

        self.get_logger().info(
            f"👣 Footprint Trail Visualizer Siap! Lebar jejak={self.tool_width:.2f}m.")

    def _map_cb(self, msg: OccupancyGrid):
        self._current_map = msg

    def _field_boundary_cb(self, msg: PolygonStamped):
        pts = [(float(p.x), float(p.y)) for p in msg.polygon.points]
        if len(pts) < 3:
            return
        self._field_polygon = pts

        # Inisialisasi grid metrik persentase
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        min_x, max_x = min(xs) - 0.5, max(xs) + 0.5
        min_y, max_y = min(ys) - 0.5, max(ys) + 0.5

        self._grid_origin_x = min_x
        self._grid_origin_y = min_y
        self._grid_w = max(10, int((max_x - min_x) / self.metric_res))
        self._grid_h = max(10, int((max_y - min_y) / self.metric_res))

        self._covered_grid = np.zeros((self._grid_h, self._grid_w), dtype=np.uint8)
        self._field_mask = np.zeros((self._grid_h, self._grid_w), dtype=np.uint8)

        poly_pixels = []
        for x, y in pts:
            c = int((x - min_x) / self.metric_res)
            r = int((y - min_y) / self.metric_res)
            poly_pixels.append([c, r])
        poly_pixels = np.array(poly_pixels, dtype=np.int32)
        cv2.fillPoly(self._field_mask, [poly_pixels], 255)

        # Kurangi sel rintangan/dinding (occupied map) agar target hanya lantai bersih yang dapat dilalui
        if self._current_map is not None:
            info = self._current_map.info
            m_res = info.resolution
            m_orig_x = info.origin.position.x
            m_orig_y = info.origin.position.y
            m_w = info.width
            m_h = info.height
            map_arr = np.array(self._current_map.data, dtype=np.int8).reshape((m_h, m_w))

            for r in range(self._grid_h):
                for c in range(self._grid_w):
                    if self._field_mask[r, c] == 255:
                        wx = self._grid_origin_x + c * self.metric_res
                        wy = self._grid_origin_y + r * self.metric_res
                        mc = int((wx - m_orig_x) / m_res)
                        mr = int((wy - m_orig_y) / m_res)
                        if 0 <= mc < m_w and 0 <= mr < m_h:
                            if map_arr[mr, mc] > 50:
                                self._field_mask[r, c] = 0

        self._total_valid_cells = np.count_nonzero(self._field_mask)

        # Reset jejak visual saat boundary baru dibuat
        self._triangles.clear()
        self._last_pose = None
        self.get_logger().info(
            f"📐 Field boundary terdaftar untuk kalkulasi cakupan: "
            f"Area Target Lantai Bersih ≈ {(self._total_valid_cells * (self.metric_res ** 2)):.2f} m².")

    def _get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (rx, ry, yaw)
        except Exception:
            return None

    def _track_loop(self):
        pose = self._get_robot_pose()
        if pose is None:
            return

        rx, ry, yaw = pose
        half_w = self.tool_width / 2.0

        # Hitung dua titik sayap (kiri & kanan) dari tool/footprint robot saat ini
        left_x = rx - half_w * math.sin(yaw)
        left_y = ry + half_w * math.cos(yaw)
        right_x = rx + half_w * math.sin(yaw)
        right_y = ry - half_w * math.cos(yaw)

        if self._last_pose is None:
            self._last_pose = (rx, ry, yaw, left_x, left_y, right_x, right_y)
            return

        prev_rx, prev_ry, prev_yaw, p_left_x, p_left_y, p_right_x, p_right_y = self._last_pose
        dist_moved = math.hypot(rx - prev_rx, ry - prev_ry)
        yaw_diff = abs(yaw - prev_yaw)

        # Hanya tambahkan segmen jejak jika robot bergerak atau berputar cukup
        if dist_moved >= self.update_dist or yaw_diff >= 0.08:
            def to_pt(x, y):
                p = Point()
                p.x = float(x)
                p.y = float(y)
                p.z = 0.015  # Tepat di atas permukaan lantai
                return p

            p1 = to_pt(p_left_x, p_left_y)
            p2 = to_pt(p_right_x, p_right_y)
            p3 = to_pt(left_x, left_y)
            p4 = to_pt(right_x, right_y)

            # Segitiga 1: P1 -> P2 -> P3
            self._triangles.extend([p1, p2, p3])
            # Segitiga 2: P2 -> P4 -> P3
            self._triangles.extend([p2, p4, p3])

            # Update Grid Metrik
            if self._covered_grid is not None:
                quad_pixels = np.array([
                    [int((p_left_x - self._grid_origin_x) / self.metric_res),
                     int((p_left_y - self._grid_origin_y) / self.metric_res)],
                    [int((p_right_x - self._grid_origin_x) / self.metric_res),
                     int((p_right_y - self._grid_origin_y) / self.metric_res)],
                    [int((right_x - self._grid_origin_x) / self.metric_res),
                     int((right_y - self._grid_origin_y) / self.metric_res)],
                    [int((left_x - self._grid_origin_x) / self.metric_res),
                     int((left_y - self._grid_origin_y) / self.metric_res)]
                ], dtype=np.int32)
                cv2.fillPoly(self._covered_grid, [quad_pixels], 255)

            self._last_pose = (rx, ry, yaw, left_x, left_y, right_x, right_y)
            self._publish_visuals()

    def _publish_visuals(self):
        if not self._triangles:
            return

        # ── 1. Publish Visual Swept Trail (Translucent Lime/Green Carpet) ─────
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'swept_footprint'
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Warna Hijau Stabilo Transparan (Mulus & Elegan di RViz)
        marker.color.r = 0.10
        marker.color.g = 0.95
        marker.color.b = 0.35
        marker.color.a = 0.40  # 40% transparansi

        marker.points = list(self._triangles)
        self.trail_pub.publish(marker)

        # ── 2. Publish Metrik Cakupan Real-Time ─────────────────────────────────
        if self._covered_grid is not None and self._total_valid_cells > 0:
            overlap = cv2.bitwise_and(self._covered_grid, self._field_mask)
            covered_cells = np.count_nonzero(overlap)
            pct = (covered_cells / float(self._total_valid_cells)) * 100.0
            pct = min(100.0, pct)

            msg_pct = Float32()
            msg_pct.data = float(pct)
            self.percent_pub.publish(msg_pct)

            covered_area_m2 = covered_cells * (self.metric_res ** 2)
            total_area_m2 = self._total_valid_cells * (self.metric_res ** 2)

            txt = f"📊 Area Bersih: {covered_area_m2:.2f}m² / {total_area_m2:.2f}m² ({pct:.1f}%)"
            msg_txt = String()
            msg_txt.data = txt
            self.status_pub.publish(msg_txt)

            self.get_logger().info(txt, throttle_duration_sec=4.0)


def main(args=None):
    rclpy.init(args=args)
    node = FootprintTrailVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

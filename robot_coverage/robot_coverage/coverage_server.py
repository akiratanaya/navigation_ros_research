#!/usr/bin/env python3
"""
coverage_server.py — Pure Fields2Cover Coverage Path Generator.

Pipeline resmi Fields2Cover:
  1. Ekstraksi obstacle solid dari /map (kaki meja di-close menjadi 1 blok meja)
  2. Shapely Boolean Difference: Field Boundary minus Obstacle -> Poligon berlubang (Interior Hole)
  3. f2c.Cells (Exterior Ring + Interior Rings)
  4. f2c.HG_Const_gen -> Headland
  5. f2c.SG_BruteForce -> Swaths (otomatis terbagi dua di atas & bawah obstacle)
  6. f2c.RP_RoutePlannerBase.genRoute(cells, swaths) -> Rute mengitari obstacle via headland
  7. f2c.PP_PathPlanning.planPath(robot, route, dubins) -> Path mulus
"""
import os
import sys
import ctypes
import math

f2c_so = '/home/akiratanaya/Fields2Cover/build/libFields2Cover.so'
if os.path.exists(f2c_so):
    try:
        ctypes.CDLL(f2c_so, mode=ctypes.RTLD_GLOBAL)
    except Exception:
        pass
f2c_py = '/home/akiratanaya/Fields2Cover/build/swig/python'
if f2c_py not in sys.path and os.path.exists(f2c_py):
    sys.path.insert(0, f2c_py)

import rclpy
from rclpy.node import Node
import fields2cover as f2c

from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PolygonStamped
from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import Trigger
import numpy as np
import cv2

from robot_coverage.visualizer import Visualizer
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from shapely.geometry import Polygon as ShapelyPolygon, LineString as ShapelyLineString
from tf2_ros import Buffer, TransformListener


class CoverageServer(Node):
    def __init__(self):
        super().__init__('coverage_server')

        self.declare_parameter('auto_compute', True)
        self.declare_parameter('robot_width', 0.15)
        self.declare_parameter('cov_width', 0.24)
        self.declare_parameter('turning_radius', 0.05)
        self.declare_parameter('swath_angle', 0.5 * math.pi)  # 90 deg = vertical swaths
        self.declare_parameter('enable_perimeter_tour', True)  # Pola robot komersial: Perimeter tour dulu, lalu infill
        self.declare_parameter('headland_swaths', 1)

        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.path_pub = self.create_publisher(Path, '/coverage_path', latch_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/coverage_markers', latch_qos)
        self.vis_marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', latch_qos)
        self.perim_count_pub = self.create_publisher(Int32, '/perimeter_waypoint_count', latch_qos)
        self.srv = self.create_service(Trigger, 'compute_coverage_path', self.compute_path_cb)

        self.vis = Visualizer(frame_id='map')
        self.last_path = None
        self.last_markers = None
        self._last_perim_count = None
        self.current_field_polygon = None
        self.current_map: OccupancyGrid | None = None

        # TF untuk membaca posisi robot saat path di-generate
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.boundary_sub = self.create_subscription(
            PolygonStamped, '/field_boundary', self.boundary_cb, latch_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, latch_qos)
        # Subscribe ke global costmap untuk filter waypoint dalam obstacle
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.current_costmap: OccupancyGrid | None = None

        self.create_timer(1.0, self.republish_cb)
        self.get_logger().info("Coverage Server Pipeline siap menerima field boundary & rintangan peta.")

    def _get_robot_pose_from_tf(self):
        """Baca posisi robot (x, y, yaw) dari TF map -> base_footprint."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (rx, ry, yaw)
        except Exception:
            return None

    def map_cb(self, msg: OccupancyGrid):
        self.current_map = msg

    def costmap_cb(self, msg: OccupancyGrid):
        self.current_costmap = msg

    def _filter_obstacle_waypoints(self, ros_path: Path) -> Path:
        """
        Hapus waypoint yang berada di dalam sel obstacle costmap (nilai >= 65).
        Ini menghilangkan waypoint yang dihasilkan Dubins curves yang melewati
        area kaki meja atau inflation zone-nya.
        Nilai costmap Nav2: 0=bebas, 1-252=berbiaya, 253=lethal, 254=inflated_obstacle
        """
        cm = self.current_costmap
        if cm is None or not ros_path.poses:
            self.get_logger().warn("⚠️ Costmap belum tersedia, filter obstacle waypoint dilewati.")
            return ros_path

        info = cm.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        w = info.width
        h = info.height
        data = cm.data  # list of int (0-255)

        filtered_poses = []
        removed = 0

        for pose in ros_path.poses:
            px = pose.pose.position.x
            py = pose.pose.position.y
            gx = int((px - ox) / res)
            gy = int((py - oy) / res)

            if 0 <= gx < w and 0 <= gy < h:
                cost = data[gy * w + gx]
                # Hapus waypoint HANYA jika cell benar-benar rintangan lethal (cost >= 253)
                if cost >= 253:
                    removed += 1
                    continue  # skip waypoint ini

            filtered_poses.append(pose)

        if removed > 0:
            self.get_logger().info(
                f"🗑️  Filter obstacle: {removed} waypoint dalam obstacle zone dihapus "
                f"({len(filtered_poses)} waypoint tersisa dari {len(ros_path.poses)} total).")

        result = Path()
        result.header = ros_path.header
        result.poses = filtered_poses
        return result

    def boundary_cb(self, msg: PolygonStamped):
        self.current_field_polygon = msg.polygon
        self.get_logger().info(f"Menerima field boundary baru dengan {len(msg.polygon.points)} titik.")

        if self.get_parameter('auto_compute').get_parameter_value().bool_value:
            self.get_logger().info("Auto-compute aktif: Menghitung coverage path...")
            success, message = self._generate_coverage_path()
            if success:
                self.get_logger().info(f"✅ {message}")
            else:
                self.get_logger().warn(f"❌ {message}")

    def republish_cb(self):
        if self.last_markers is not None:
            self.marker_pub.publish(self.last_markers)
            self.vis_marker_pub.publish(self.last_markers)
        if self._last_perim_count is not None:
            self.perim_count_pub.publish(Int32(data=int(self._last_perim_count)))

    # ══════════════════════════════════════════════════════════════════════════
    #  OBSTACLE EXTRACTION FROM /map
    # ══════════════════════════════════════════════════════════════════════════

    def _extract_obstacle_polygons(self, poly_pts):
        """
        Ekstrak rintangan interior (kaki-kaki meja) dari peta OccupancyGrid.
        Mengembalikan:
            obs_polygons: list ShapelyPolygon
            obs_circles: list of (center_x, center_y, safe_radius)
        """
        if self.current_map is None:
            return [], []

        info = self.current_map.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        w = info.width
        h = info.height
        map_arr = np.array(self.current_map.data, dtype=np.int8).reshape((h, w))

        # Mask poligon boundary
        pixel_pts = np.array([[int((px - ox) / res), int((py - oy) / res)]
                              for px, py in poly_pts], dtype=np.int32)
        field_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(field_mask, [pixel_pts], 255)

        # Erode 0.15m agar dinding luar ruangan TIDAK ikut terdeteksi sebagai rintangan interior
        erode_k = max(3, int(0.15 / res) * 2 + 1)
        interior_mask = cv2.erode(
            field_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (erode_k, erode_k)))

        occ_interior = np.zeros((h, w), dtype=np.uint8)
        occ_interior[(map_arr > 50) & (interior_mask == 255)] = 255

        if np.count_nonzero(occ_interior) == 0:
            return [], []

        # Dilasi secukupnya untuk menghubungkan pixel rintangan satu tiang
        dilate_k = max(3, int(0.04 / res) * 2 + 1)
        inflated = cv2.dilate(
            occ_interior, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_k, dilate_k)))

        contours, _ = cv2.findContours(inflated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        obs_polygons = []
        obs_circles = []
        for cnt in contours:
            area = cv2.contourArea(cnt) * (res * res)
            if area >= 0.003:  # Filter noise sangat kecil (tiang meja ~0.005m² - 0.05m²)
                hull = cv2.convexHull(cnt)
                rect = cv2.minAreaRect(hull)
                box_pts = cv2.boxPoints(rect)
                world_pts = [(float(pt[0]) * res + ox, float(pt[1]) * res + oy) for pt in box_pts]
                obs_poly = ShapelyPolygon(world_pts).buffer(0)

                cx = rect[0][0] * res + ox
                cy = rect[0][1] * res + oy
                raw_r = max(rect[1][0], rect[1][1]) * 0.5 * res
                # safe_r: robot radius (0.155m) + clearance (0.025m) = 0.18m
                safe_r = max(0.18, raw_r + 0.12)

                if obs_poly.is_valid and not obs_poly.is_empty:
                    self.get_logger().info(
                        f"🛡️ Rintangan tiang interior: luas={area:.3f}m² pada posisi ({cx:.2f}, {cy:.2f}), safe_r={safe_r:.2f}m")
                    obs_polygons.append(obs_poly)
                    obs_circles.append((cx, cy, safe_r))
        return obs_polygons, obs_circles

    def _bypass_segment(self, x1, y1, x2, y2, obs_circles, field_poly, dense_step=0.04):
        """
        Generate titik-titik diskret sepanjang segmen lurus (x1, y1) -> (x2, y2).
        Jika ada rintangan tiang (kaki meja), jalur secara mulus melengkung
        (smooth circular arc) melingkari rintangan pada jarak safe_radius,
        menjaga sapuan swath tetap utuh, rapi, sejajar, tanpa potongan diagonal.
        """
        d = math.hypot(x2 - x1, y2 - y1)
        if d < 1e-4:
            return [(x1, y1)]
        n = max(1, int(d / dense_step))
        ux = (x2 - x1) / d
        uy = (y2 - y1) / d
        nx = -uy
        ny = ux
        pts = []
        for j in range(n + 1):
            t = j / float(n)
            px = x1 + t * (x2 - x1)
            py = y1 + t * (y2 - y1)
            for ox, oy, r_safe in obs_circles:
                vx = px - ox
                vy = py - oy
                s = vx * ux + vy * uy
                if abs(s) < r_safe:
                    r_perp = math.sqrt(max(0.0, r_safe**2 - s**2))
                    d_perp = vx * nx + vy * ny
                    if abs(d_perp) < r_perp:
                        sign = 1.0 if d_perp >= 0 else -1.0
                        shift = r_perp - abs(d_perp)
                        cand_x = px + sign * shift * nx
                        cand_y = py + sign * shift * ny
                        from shapely.geometry import Point as ShapelyPoint
                        if field_poly and not field_poly.contains(ShapelyPoint(cand_x, cand_y)):
                            cand_x = px - sign * shift * nx
                            cand_y = py - sign * shift * ny
                        px, py = cand_x, cand_y
                        break
            pts.append((px, py))
        return pts

    def _pts_to_poses(self, pts):
        """Ubah list of (x, y) menjadi list of PoseStamped dengan orientasi yaw mulus."""
        poses = []
        n = len(pts)
        if n == 0:
            return poses

        last_yaw = 0.0
        for i in range(n):
            if i < n - 1:
                dx = pts[i + 1][0] - pts[i][0]
                dy = pts[i + 1][1] - pts[i][1]
                if math.hypot(dx, dy) > 1e-4:
                    yaw = math.atan2(dy, dx)
                    last_yaw = yaw
                else:
                    yaw = last_yaw
            else:
                yaw = last_yaw

            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = float(pts[i][0])
            p.pose.position.y = float(pts[i][1])
            p.pose.orientation.z = math.sin(yaw / 2.0)
            p.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(p)
        return poses

    # ══════════════════════════════════════════════════════════════════════════
    #  SHAPELY → f2c.Cells CONVERSION
    # ══════════════════════════════════════════════════════════════════════════

    def _shapely_to_f2c_cells(self, shapely_poly) -> f2c.Cells:
        """Konversi ShapelyPolygon ke f2c.Cells."""
        cells = f2c.Cells()
        if shapely_poly.is_empty:
            return cells

        polys = [shapely_poly] if isinstance(shapely_poly, ShapelyPolygon) else list(shapely_poly.geoms)
        for p in polys:
            if p.is_empty or p.area < 0.05:
                continue
            p = p.buffer(0)
            if p.is_empty or p.area < 0.05:
                continue

            c = f2c.Cell()
            ext = f2c.LinearRing()
            for x, y in p.exterior.coords:
                ext.addPoint(f2c.Point(float(x), float(y), 0.0))
            c.addRing(ext)

            for interior in p.interiors:
                ir = f2c.LinearRing()
                for x, y in interior.coords:
                    ir.addPoint(f2c.Point(float(x), float(y), 0.0))
                c.addRing(ir)

            cells.addGeometry(c)
        return cells

    # ══════════════════════════════════════════════════════════════════════════
    #  MAIN PIPELINE: Pure Fields2Cover + Obstacle Bypass
    # ══════════════════════════════════════════════════════════════════════════

    def _generate_coverage_path(self):
        if self.current_field_polygon is None or len(self.current_field_polygon.points) < 3:
            return False, "Belum ada field boundary valid."

        try:
            robot_width = self.get_parameter('robot_width').get_parameter_value().double_value
            cov_width = self.get_parameter('cov_width').get_parameter_value().double_value
            turning_radius = self.get_parameter('turning_radius').get_parameter_value().double_value
            swath_angle = self.get_parameter('swath_angle').get_parameter_value().double_value
            enable_perimeter = self.get_parameter('enable_perimeter_tour').get_parameter_value().bool_value

            # ── 1. Poligon Lahan Shapely ──
            poly_pts = [(float(pt.x), float(pt.y)) for pt in self.current_field_polygon.points]
            field_poly = ShapelyPolygon(poly_pts).buffer(0)
            if field_poly.is_empty:
                return False, "Area lahan kosong."

            obstacles, obs_circles = self._extract_obstacle_polygons(poly_pts)

            # ── 2. Konversi ke f2c.Cells (Batas bersih lahan) ──
            # Kita TIDAK membuat lubang polygon di tiang meja agar Fields2Cover menghasilkan
            # sapuan swath murni, utuh, rapi, dan konsisten (kaki meja dihindari via smooth arc bypass).
            raw_cells = self._shapely_to_f2c_cells(field_poly)
            if raw_cells.size() == 0:
                return False, "f2c.Cells kosong."

            # ── 3. Cellular Decomposition (Untuk Visualisasi & Analisis Sub-Sel) ──
            decomp = f2c.DECOMP_Boustrophedon()
            cells = decomp.decompose(raw_cells)
            self.get_logger().info(
                f"🧩 Cellular Decomposition: Lahan berhasil dipecah menjadi {cells.size()} sub-sel independen!")
            for i in range(cells.size()):
                c_i = cells.getGeometry(i)
                self.get_logger().info(f"   ├─ Sub-Sel {i}: Luas = {c_i.area():.2f}m²")

            # ── 4. Setup Robot ──
            robot = f2c.Robot(robot_width, cov_width)
            robot.setMinTurningRadius(turning_radius)

            dense_step = 0.04
            perim_pts = []
            robot_pose = self._get_robot_pose_from_tf()
            rx = robot_pose[0] if robot_pose else poly_pts[0][0]
            ry = robot_pose[1] if robot_pose else poly_pts[0][1]

            hg = f2c.HG_Const_gen()

            # ── 5. Fase 1: Perimeter Tour (Keliling Dinding Ruangan Bersih) ──
            if enable_perimeter:
                try:
                    hl_swaths = hg.generateHeadlandSwaths(raw_cells, cov_width, 1)
                    if hl_swaths and len(hl_swaths) > 0 and hl_swaths[0].size() > 0:
                        cell_i = hl_swaths[0].getGeometry(0)
                        if cell_i.size() > 0:
                            ring = cell_i.getGeometry(0)
                            if ring.size() >= 3:
                                raw_pts = [(ring.getGeometry(i).getX(), ring.getGeometry(i).getY()) for i in range(ring.size())]
                                if math.hypot(raw_pts[0][0] - raw_pts[-1][0], raw_pts[0][1] - raw_pts[-1][1]) < 0.01:
                                    raw_pts = raw_pts[:-1]

                                best_idx = min(range(len(raw_pts)), key=lambda i: math.hypot(raw_pts[i][0] - rx, raw_pts[i][1] - ry))
                                ordered = raw_pts[best_idx:] + raw_pts[:best_idx]
                                ordered.append(ordered[0])  # Tutup loop 360°

                                for i in range(len(ordered) - 1):
                                    seg_pts = self._bypass_segment(
                                        ordered[i][0], ordered[i][1],
                                        ordered[i+1][0], ordered[i+1][1],
                                        obs_circles, field_poly, dense_step)
                                    if perim_pts and seg_pts:
                                        perim_pts.extend(seg_pts[1:])
                                    else:
                                        perim_pts.extend(seg_pts)
                                self.get_logger().info(
                                    f"🛡️  [FASE 1 - PERIMETER] Tour keliling batas siap: {len(perim_pts)} waypoint.")
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Gagal generate perimeter tour ({e}), fallback ke infill murni.")
                    perim_pts = []

            perim_poses = self._pts_to_poses(perim_pts)

            # ── 6. Fase 2: Infill Sweeping Bersih, Rapi & Teratur (Serpentine/Boustrophedon) ──
            inner_field = None
            if enable_perimeter and len(perim_poses) > 0:
                try:
                    inner_cand = hg.generateHeadlands(raw_cells, cov_width * 0.5)
                    if inner_cand.size() > 0 and inner_cand.area() > 0.15:
                        inner_field = inner_cand
                except Exception:
                    inner_field = None

            target_cells = inner_field if inner_field is not None else raw_cells

            sg = f2c.SG_BruteForce()
            swaths = f2c.Swaths()
            obj_n = f2c.OBJ_NSwath()
            for k in range(target_cells.size()):
                try:
                    if abs(swath_angle) > 1e-4:
                        c_sw = sg.generateSwaths(swath_angle, cov_width, target_cells.getGeometry(k))
                    else:
                        c_sw = sg.generateBestSwaths(obj_n, cov_width, target_cells.getGeometry(k))
                except Exception:
                    c_sw = sg.generateSwaths(swath_angle, cov_width, target_cells.getGeometry(k))
                for s_i in range(c_sw.size()):
                    if c_sw[s_i].length() >= 0.20:
                        swaths.push_back(c_sw[s_i])

            if swaths.size() == 0:
                for k in range(raw_cells.size()):
                    try:
                        c_sw = sg.generateBestSwaths(obj_n, cov_width, raw_cells.getGeometry(k))
                    except Exception:
                        c_sw = sg.generateSwaths(swath_angle, cov_width, raw_cells.getGeometry(k))
                    for s_i in range(c_sw.size()):
                        if c_sw[s_i].length() >= 0.20:
                            swaths.push_back(c_sw[s_i])

            boustro = f2c.RP_Boustrophedon()
            ref_x = perim_poses[-1].pose.position.x if perim_poses else rx
            ref_y = perim_poses[-1].pose.position.y if perim_poses else ry

            best_v = 0
            min_d = float('inf')
            for v in range(4):
                test_sw = boustro.genSortedSwaths(swaths, v)
                if test_sw.size() > 0:
                    sp = test_sw[0].startPoint()
                    d = math.hypot(sp.getX() - ref_x, sp.getY() - ref_y)
                    if d < min_d:
                        min_d = d
                        best_v = v

            sorted_sw = boustro.genSortedSwaths(swaths, best_v)

            # Generate jalur infill teratur dengan smooth circular bypass jika berpapasan tiang meja
            infill_pts = []
            for i in range(sorted_sw.size()):
                s = sorted_sw[i]
                if s.length() < 0.20:
                    continue
                x1, y1 = s.startPoint().getX(), s.startPoint().getY()
                x2, y2 = s.endPoint().getX(), s.endPoint().getY()

                swath_seg = self._bypass_segment(x1, y1, x2, y2, obs_circles, field_poly, dense_step)
                if infill_pts and swath_seg:
                    infill_pts.extend(swath_seg[1:])
                else:
                    infill_pts.extend(swath_seg)

                # Sambungan U-turn langsung ke baris berikutnya (Langkah ortogonal rapi tanpa garis diagonal)
                if i < sorted_sw.size() - 1:
                    next_s = sorted_sw[i + 1]
                    nx, ny = next_s.startPoint().getX(), next_s.startPoint().getY()
                    turn_seg = self._bypass_segment(
                        infill_pts[-1][0], infill_pts[-1][1],
                        nx, ny,
                        obs_circles, field_poly, dense_step)
                    if len(turn_seg) > 1:
                        infill_pts.extend(turn_seg[1:])

            infill_poses = self._pts_to_poses(infill_pts)
            self.get_logger().info(
                f"🚜 [FASE 2 - INFILL] Swaths Paralel Rapi: {sorted_sw.size()} baris terurut rapi, {len(infill_poses)} waypoint.")

            # ── 7. Filter obstacle costmap per fase agar index dan perim_count akurat ──
            perim_path = Path()
            perim_path.header.frame_id = 'map'
            perim_path.header.stamp = self.get_clock().now().to_msg()
            perim_path.poses = perim_poses
            perim_path = self._filter_obstacle_waypoints(perim_path)
            perim_poses = perim_path.poses
            perim_count = len(perim_poses)

            infill_path = Path()
            infill_path.header.frame_id = 'map'
            infill_path.header.stamp = self.get_clock().now().to_msg()
            infill_path.poses = infill_poses
            infill_path = self._filter_obstacle_waypoints(infill_path)
            infill_poses = infill_path.poses

            combined_poses = perim_poses + infill_poses

            ros_path = Path()
            ros_path.header.frame_id = 'map'
            ros_path.header.stamp = self.get_clock().now().to_msg()
            ros_path.poses = combined_poses

            # ── 8. Filter waypoint dalam obstacle costmap (jika ada) ──
            ros_path = self._filter_obstacle_waypoints(ros_path)

            # ── 9. Visualisasi Markers & Publish ──
            path_markers = self.vis.create_path_markers(ros_path, perim_count, obs_circles)
            cell_markers = self.vis.create_cells_markers(cells)
            all_markers = MarkerArray()
            all_markers.markers.extend(path_markers.markers)
            all_markers.markers.extend(cell_markers.markers)

            self.last_path = ros_path
            self.last_markers = all_markers
            self._last_perim_count = perim_count
            self.path_pub.publish(ros_path)
            self.marker_pub.publish(all_markers)
            self.vis_marker_pub.publish(all_markers)
            self.perim_count_pub.publish(Int32(data=int(perim_count)))

            return True, (f"Commercial Coverage Path Siap: {len(perim_poses)} waypoint Perimeter (Fase 1) "
                          f"+ {len(infill_poses)} waypoint Infill (Fase 2) | Total: {len(ros_path.poses)} poses")

        except Exception as e:
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False, f"Error: {str(e)}"

    def _f2c_path_to_ros(self, f2c_path) -> Path:
        """Convert f2c.Path ke ROS 2 nav_msgs/Path dengan interpolasi dense 5cm dalam urutan maju alami."""
        if f2c_path.size() == 0:
            return Path()

        ros_path = Path()
        ros_path.header.frame_id = 'map'
        ros_path.header.stamp = self.get_clock().now().to_msg()
        now = ros_path.header.stamp

        dense_step = 0.05
        poses = []

        for i in range(f2c_path.size()):
            s = f2c_path.getState(i)
            cx, cy = s.point.getX(), s.point.getY()

            if i < f2c_path.size() - 1:
                sn = f2c_path.getState(i + 1)
                nx, ny = sn.point.getX(), sn.point.getY()
                dist = math.hypot(nx - cx, ny - cy)
                yaw = math.atan2(ny - cy, nx - cx)
                n_sub = max(1, int(dist / dense_step))
                for j in range(n_sub):
                    t = j / float(n_sub)
                    p = PoseStamped()
                    p.header.frame_id = 'map'
                    p.header.stamp = now
                    p.pose.position.x = cx + t * (nx - cx)
                    p.pose.position.y = cy + t * (ny - cy)
                    p.pose.orientation.z = math.sin(yaw / 2.0)
                    p.pose.orientation.w = math.cos(yaw / 2.0)
                    poses.append(p)
            else:
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.header.stamp = now
                p.pose.position.x = cx
                p.pose.position.y = cy
                if len(poses) > 0:
                    p.pose.orientation = poses[-1].pose.orientation
                else:
                    p.pose.orientation.z = math.sin(s.angle / 2.0)
                    p.pose.orientation.w = math.cos(s.angle / 2.0)
                poses.append(p)

        ros_path.poses = poses
        return ros_path

    def compute_path_cb(self, request, response):
        success, message = self._generate_coverage_path()
        response.success = success
        response.message = message
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CoverageServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

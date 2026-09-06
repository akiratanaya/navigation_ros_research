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
        self.srv = self.create_service(Trigger, 'compute_coverage_path', self.compute_path_cb)

        self.vis = Visualizer(frame_id='map')
        self.last_path = None
        self.last_markers = None
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

    # ══════════════════════════════════════════════════════════════════════════
    #  OBSTACLE EXTRACTION FROM /map
    # ══════════════════════════════════════════════════════════════════════════

    def _extract_obstacle_polygons(self, poly_pts):
        """
        Ekstrak rintangan interior (kaki-kaki samping meja) secara realistis.
        Menghubungkan kaki depan & belakang (searah Y / 0.95m) menjadi 2 panel samping meja,
        sementara koridor tengah (kolong meja 1.40m) tetap 100% terbuka bebas untuk disapu.
        """
        if self.current_map is None:
            return []

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
            return []

        # Inflasi clearance kaki meja secukupnya (0.08m agar bodi robot bebas tabrakan tapi kolong meja tetap terbuka luas)
        dilate_k = max(3, int(0.08 / res) * 2 + 1)
        inflated = cv2.dilate(
            occ_interior, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_k, dilate_k)))
        
        contours, _ = cv2.findContours(inflated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        obs_list = []
        for cnt in contours:
            area = cv2.contourArea(cnt) * (res * res)
            if area >= 0.005:  # Filter noise sangat kecil (kaki meja ~0.01m² - 0.03m²)
                hull = cv2.convexHull(cnt)
                rect = cv2.minAreaRect(hull)
                box_pts = cv2.boxPoints(rect)
                world_pts = [(float(pt[0]) * res + ox, float(pt[1]) * res + oy) for pt in box_pts]
                obs_poly = ShapelyPolygon(world_pts).buffer(0)
                if obs_poly.is_valid and not obs_poly.is_empty:
                    self.get_logger().info(
                        f"🛡️ Rintangan tiang interior: luas={area:.3f}m² pada posisi ({rect[0][0]*res+ox:.2f}, {rect[0][1]*res+oy:.2f})")
                    obs_list.append(obs_poly)
        return obs_list
    
    # ══════════════════════════════════════════════════════════════════════════
    #  SHAPELY → f2c.Cells CONVERSION
    # ══════════════════════════════════════════════════════════════════════════

    def _shapely_to_f2c_cells(self, shapely_poly) -> f2c.Cells:
        """Konversi ShapelyPolygon (dengan interior holes) ke f2c.Cells."""
        cells = f2c.Cells()
        if shapely_poly.is_empty:
            return cells

        polys = [shapely_poly] if isinstance(shapely_poly, ShapelyPolygon) else list(shapely_poly.geoms)
        for p in polys:
            if p.is_empty or p.area < 0.05:
                continue
            p = p.buffer(0)  # Pastikan geometri valid
            if p.is_empty or p.area < 0.05:
                continue

            c = f2c.Cell()
            # Exterior ring (batas luar lahan)
            ext = f2c.LinearRing()
            for x, y in p.exterior.coords:
                ext.addPoint(f2c.Point(float(x), float(y), 0.0))
            c.addRing(ext)

            # Interior rings (lubang rintangan meja)
            for interior in p.interiors:
                ir = f2c.LinearRing()
                for x, y in interior.coords:
                    ir.addPoint(f2c.Point(float(x), float(y), 0.0))
                c.addRing(ir)

            cells.addGeometry(c)
        return cells

    # ══════════════════════════════════════════════════════════════════════════
    #  MAIN PIPELINE: Pure Fields2Cover
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

            # ── 1. Poligon Lahan Shapely dikurangi Rintangan Meja ──
            poly_pts = [(float(pt.x), float(pt.y)) for pt in self.current_field_polygon.points]
            field_poly = ShapelyPolygon(poly_pts).buffer(0)

            obstacles = self._extract_obstacle_polygons(poly_pts)
            for obs in obstacles:
                field_poly = field_poly.difference(obs)

            if field_poly.is_empty:
                return False, "Area lahan kosong setelah dikurangi rintangan."

            # ── 2. Konversi ke f2c.Cells (Exterior + Interior Hole Rintangan) ──
            raw_cells = self._shapely_to_f2c_cells(field_poly)
            if raw_cells.size() == 0:
                return False, "f2c.Cells kosong."

            # ── 3. Boustrophedon Cellular Decomposition (Untuk Visualisasi & Analisis Sub-Sel) ──
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

            dense_step = 0.05
            perim_poses = []
            robot_pose = self._get_robot_pose_from_tf()
            rx = robot_pose[0] if robot_pose else poly_pts[0][0]
            ry = robot_pose[1] if robot_pose else poly_pts[0][1]

            hg = f2c.HG_Const_gen()

            # ── 5. Fase 1: Perimeter / Headland Tour (Standar Robot Komersial) ──
            # Mengitari tepi dinding luar ruangan dan mengitari rintangan meja
            if enable_perimeter:
                try:
                    hl_swaths = hg.generateHeadlandSwaths(raw_cells, cov_width, 1)
                    if hl_swaths and len(hl_swaths) > 0 and hl_swaths[0].size() > 0:
                        for ci in range(hl_swaths[0].size()):
                            cell_i = hl_swaths[0].getGeometry(ci)
                            # HANYA proses ring 0 (Batas luar ruangan / perimeter dinding).
                            # ABAIKAN ring >= 1 (lubang kaki meja) agar TIDAK ADA jalur belah ketupat di sekeliling tiang!
                            if cell_i.size() == 0:
                                continue
                            ring = cell_i.getGeometry(0)
                            if ring.size() < 3:
                                continue
                            raw_pts = [(ring.getGeometry(i).getX(), ring.getGeometry(i).getY()) for i in range(ring.size())]
                            if math.hypot(raw_pts[0][0] - raw_pts[-1][0], raw_pts[0][1] - raw_pts[-1][1]) < 0.01:
                                raw_pts = raw_pts[:-1]

                                # Urutkan titik loop perimeter dimulai dari titik terdekat dengan robot
                                best_idx = min(range(len(raw_pts)), key=lambda i: math.hypot(raw_pts[i][0] - rx, raw_pts[i][1] - ry))
                                ordered = raw_pts[best_idx:] + raw_pts[:best_idx]
                                ordered.append(ordered[0])  # Tutup loop 360°

                                for i in range(len(ordered) - 1):
                                    x1, y1 = ordered[i]
                                    x2, y2 = ordered[i+1]
                                    d = math.hypot(x2 - x1, y2 - y1)
                                    n = max(1, int(d / dense_step))
                                    yaw = math.atan2(y2 - y1, x2 - x1)
                                    for j in range(n):
                                        t = j / float(n)
                                        p = PoseStamped()
                                        p.header.frame_id = 'map'
                                        p.header.stamp = self.get_clock().now().to_msg()
                                        p.pose.position.x = x1 + t * (x2 - x1)
                                        p.pose.position.y = y1 + t * (y2 - y1)
                                        p.pose.orientation.z = math.sin(yaw / 2.0)
                                        p.pose.orientation.w = math.cos(yaw / 2.0)
                                        perim_poses.append(p)
                        self.get_logger().info(
                            f"🛡️  [FASE 1 - PERIMETER] Tour keliling batas & rintangan siap: {len(perim_poses)} waypoint.")
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Gagal generate perimeter tour ({e}), fallback ke infill murni.")
                    perim_poses = []

            # ── 6. Fase 2: Infill Sweeping di Area Tengah yang Bersih ──
            # Menghitung inner_field yang sudah dikurangi headland (margin cov_width * 0.5 dari dinding & rintangan)
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
                    if c_sw[s_i].length() >= 0.20:  # Filter micro-swaths
                        swaths.push_back(c_sw[s_i])

            if swaths.size() == 0:
                # Fallback ke raw cells jika inner_field terlalu sempit
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

            # Perencanaan Jalur Infill Bersih untuk Robot Differential Drive (Tanpa Omega Loop Traktor)
            infill_poses = []
            for i in range(sorted_sw.size()):
                s = sorted_sw[i]
                if s.length() < 0.20:
                    continue
                x1, y1 = s.startPoint().getX(), s.startPoint().getY()
                x2, y2 = s.endPoint().getX(), s.endPoint().getY()
                d = math.hypot(x2 - x1, y2 - y1)
                n = max(1, int(d / dense_step))
                yaw = math.atan2(y2 - y1, x2 - x1)
                for j in range(n + 1):
                    t = j / float(n)
                    p = PoseStamped()
                    p.header.frame_id = 'map'
                    p.header.stamp = self.get_clock().now().to_msg()
                    p.pose.position.x = x1 + t * (x2 - x1)
                    p.pose.position.y = y1 + t * (y2 - y1)
                    p.pose.orientation.z = math.sin(yaw / 2.0)
                    p.pose.orientation.w = math.cos(yaw / 2.0)
                    infill_poses.append(p)

                # Sambungan lurus bersih ke baris berikutnya (U-turn langsung tanpa omega loops)
                if i < sorted_sw.size() - 1:
                    next_s = sorted_sw[i + 1]
                    nx, ny = next_s.startPoint().getX(), next_s.startPoint().getY()
                    td = math.hypot(nx - x2, ny - y2)
                    tn = max(1, int(td / dense_step))
                    tyaw = math.atan2(ny - y2, nx - x2)
                    for j in range(1, tn):
                        t = j / float(tn)
                        p = PoseStamped()
                        p.header.frame_id = 'map'
                        p.header.stamp = self.get_clock().now().to_msg()
                        p.pose.position.x = x2 + t * (nx - x2)
                        p.pose.position.y = y2 + t * (ny - y2)
                        p.pose.orientation.z = math.sin(tyaw / 2.0)
                        p.pose.orientation.w = math.cos(tyaw / 2.0)
                        infill_poses.append(p)

            infill_ros = Path()
            infill_ros.header.frame_id = 'map'
            infill_ros.header.stamp = self.get_clock().now().to_msg()
            infill_ros.poses = infill_poses

            self.get_logger().info(
                f"🚜 [FASE 2 - INFILL] Swaths Paralel Bersih: {sorted_sw.size()} baris terurut, {len(infill_poses)} waypoint.")

            # ── 7. Sambungkan Fase 1 (Perimeter) ke Fase 2 (Infill) Secara Mulus ──
            trans_poses = []
            if perim_poses and infill_ros.poses:
                p_last = perim_poses[-1].pose.position
                p_first = infill_ros.poses[0].pose.position
                d_trans = math.hypot(p_first.x - p_last.x, p_first.y - p_last.y)
                n_trans = max(1, int(d_trans / dense_step))
                yaw_trans = math.atan2(p_first.y - p_last.y, p_first.x - p_last.x)
                for j in range(n_trans):
                    t = j / float(n_trans)
                    p = PoseStamped()
                    p.header.frame_id = 'map'
                    p.header.stamp = self.get_clock().now().to_msg()
                    p.pose.position.x = p_last.x + t * (p_first.x - p_last.x)
                    p.pose.position.y = p_last.y + t * (p_first.y - p_last.y)
                    p.pose.orientation.z = math.sin(yaw_trans / 2.0)
                    p.pose.orientation.w = math.cos(yaw_trans / 2.0)
                    trans_poses.append(p)

            combined_poses = perim_poses + trans_poses + infill_ros.poses
            perim_count = len(perim_poses)

            ros_path = Path()
            ros_path.header.frame_id = 'map'
            ros_path.header.stamp = self.get_clock().now().to_msg()
            ros_path.poses = combined_poses

            # ── 8. Filter waypoint dalam polygon rintangan (kaki meja + clearance) ──
            if obstacles:
                valid_poses = []
                num_dropped = 0
                for p in ros_path.poses:
                    from shapely.geometry import Point as ShapelyPoint
                    pt = ShapelyPoint(p.pose.position.x, p.pose.position.y)
                    inside = False
                    for obs in obstacles:
                        if obs.contains(pt):
                            inside = True
                            break
                    if inside:
                        num_dropped += 1
                    else:
                        valid_poses.append(p)
                if num_dropped > 0:
                    self.get_logger().info(
                        f"🛡️  Filter polygon: {num_dropped} waypoint di dalam obstacle zone dihapus "
                        f"({len(valid_poses)} tersisa).")
                    ros_path.poses = valid_poses

            # ── 9. Filter waypoint dalam obstacle costmap (jika ada) ──
            ros_path = self._filter_obstacle_waypoints(ros_path)

            # ── 10. Visualisasi Markers & Publish ──
            path_markers = self.vis.create_path_markers(ros_path, perim_count)
            cell_markers = self.vis.create_cells_markers(cells)
            all_markers = MarkerArray()
            all_markers.markers.extend(path_markers.markers)
            all_markers.markers.extend(cell_markers.markers)

            self.last_path = ros_path
            self.last_markers = all_markers
            self.path_pub.publish(ros_path)
            self.marker_pub.publish(all_markers)

            return True, (f"Commercial Coverage Path Siap: {len(perim_poses)} waypoint Perimeter (Fase 1) "
                          f"+ {len(infill_ros.poses)} waypoint Infill (Fase 2) | Total: {len(ros_path.poses)} poses")

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

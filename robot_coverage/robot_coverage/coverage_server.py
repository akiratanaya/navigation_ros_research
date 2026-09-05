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
                # Hapus waypoint jika cell berbiaya tinggi (lethal = 253/254, atau inflated >= 65)
                if cost >= 65:
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

        # Morphological Close vertikal (tinggi 0.95m, lebar 0.10m) untuk menyatukan kaki depan-belakang meja
        close_ky = max(3, int(0.95 / res))
        close_kx = max(1, int(0.10 / res))
        side_panels = cv2.morphologyEx(
            occ_interior, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (close_kx, close_ky)))

        # Inflasi clearance kaki meja (0.22m agar bodi robot 0.34m bebas tabrakan dari kaki meja)
        dilate_k = max(3, int(0.22 / res) * 2 + 1)
        inflated = cv2.dilate(
            side_panels, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_k, dilate_k)))
        
        contours, _ = cv2.findContours(inflated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        obs_list = []
        for cnt in contours:
            area = cv2.contourArea(cnt) * (res * res)
            if area >= 0.02:  # Filter noise kecil
                hull = cv2.convexHull(cnt)
                rect = cv2.minAreaRect(hull)
                box_pts = cv2.boxPoints(rect)
                world_pts = [(float(pt[0]) * res + ox, float(pt[1]) * res + oy) for pt in box_pts]
                obs_poly = ShapelyPolygon(world_pts).buffer(0)
                if obs_poly.is_valid and not obs_poly.is_empty:
                    self.get_logger().info(
                        f"🛡️ Rintangan panel samping meja: luas={area:.3f}m² pada posisi ({rect[0][0]*res+ox:.2f}, {rect[0][1]*res+oy:.2f})")
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

            # ── 3. Boustrophedon Cellular Decomposition (Untuk Analisis & Visualisasi Sub-Sel) ──
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

            # ── 5. Swaths Paralel Menghindari Rintangan Meja ──
            # Menggunakan raw_cells yang sudah memiliki lubang rintangan meja (interior rings)
            # agar Fields2Cover memotong swath di depan rintangan secara alami
            sg = f2c.SG_BruteForce()
            swaths = f2c.Swaths()
            for k in range(raw_cells.size()):
                c_sw = sg.generateSwaths(swath_angle, cov_width, raw_cells.getGeometry(k))
                for s_i in range(c_sw.size()):
                    swaths.push_back(c_sw[s_i])

            if swaths.size() == 0:
                ext_ring = f2c.LinearRing()
                for px, py in poly_pts:
                    ext_ring.addPoint(f2c.Point(float(px), float(py), 0.0))
                ext_ring.addPoint(f2c.Point(float(poly_pts[0][0]), float(poly_pts[0][1]), 0.0))
                field_cell = f2c.Cell()
                field_cell.addRing(ext_ring)
                swaths = sg.generateSwaths(swath_angle, cov_width, field_cell)

            boustro = f2c.RP_Boustrophedon()
            robot_pose = self._get_robot_pose_from_tf()
            rx = robot_pose[0] if robot_pose else poly_pts[0][0]
            ry = robot_pose[1] if robot_pose else poly_pts[0][1]

            best_v = 0
            min_d = float('inf')
            for v in range(4):
                test_sw = boustro.genSortedSwaths(swaths, v)
                if test_sw.size() > 0:
                    sp = test_sw[0].startPoint()
                    d = math.hypot(sp.getX() - rx, sp.getY() - ry)
                    if d < min_d:
                        min_d = d
                        best_v = v

            sorted_sw = boustro.genSortedSwaths(swaths, best_v)
            p_start = sorted_sw[0].startPoint()
            p_end = sorted_sw[sorted_sw.size() - 1].endPoint()
            self.get_logger().info(
                f"🚜 Swaths Paralel Bebas-Persilangan: {sorted_sw.size()} baris terurut "
                f"| Titik Pangkal Terluar: ({p_start.getX():.2f}, {p_start.getY():.2f}) "
                f"| End: ({p_end.getX():.2f}, {p_end.getY():.2f}).")

            # ── 6. Path Planning (Dubins Curves Kontinu Bebas Persilangan) ──
            pp = f2c.PP_PathPlanning()
            dubins = f2c.PP_DubinsCurves()
            f2c_path = pp.planPath(robot, sorted_sw, dubins)
            self.get_logger().info(f"📍 Path Dubins: {f2c_path.size()} titik (panjang {f2c_path.length():.2f}m)")

            # ── 7. Konversi ke ROS 2 Path ──
            ros_path = self._f2c_path_to_ros(f2c_path)

            # ── 9b. Filter waypoint dalam polygon rintangan (kaki meja + clearance) ──
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

            # ── 9c. Filter waypoint dalam obstacle costmap (jika ada) ──
            ros_path = self._filter_obstacle_waypoints(ros_path)

            # ── 10. Visualisasi Markers & Publish (Jalur + Batas Tiap Sub-Sel) ──
            path_markers = self.vis.create_path_markers(f2c_path)
            cell_markers = self.vis.create_cells_markers(cells)
            all_markers = MarkerArray()
            all_markers.markers.extend(path_markers.markers)
            all_markers.markers.extend(cell_markers.markers)

            self.last_path = ros_path
            self.last_markers = all_markers
            self.path_pub.publish(ros_path)
            self.marker_pub.publish(all_markers)

            return True, f"Coverage Path Paralel Bebas-Persilangan ({cells.size()} sub-sel, {sorted_sw.size()} swaths, {len(ros_path.poses)} waypoints)"

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import fields2cover as f2c

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PolygonStamped
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import Trigger

from robot_coverage.decomp_generator import DecompGenerator
from robot_coverage.headland_generator import HeadlandGenerator
from robot_coverage.swath_generator import SwathGenerator
from robot_coverage.route_generator import RouteGenerator
from robot_coverage.path_generator import PathGenerator
from robot_coverage.visualizer import Visualizer

from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy


class CoverageServer(Node):
    def __init__(self):
        super().__init__('coverage_server')

        # Declare parameters agar mudah di-tweak lewat launch file / YAML
        self.declare_parameter('auto_compute', True)
        self.declare_parameter('robot_width', 0.1)
        self.declare_parameter('cov_width', 0.3)
        self.declare_parameter('headland_swaths', 1)
        self.declare_parameter('turning_radius', 0.0)
        self.declare_parameter('route_pattern', 'or_tools')
        self.declare_parameter('curve_type', 'dubins')
        self.declare_parameter('swath_angle', 0.5 * math.pi)
        self.declare_parameter('decomp_method', 'boustrophedon')
        self.declare_parameter('split_angle', 0.5 * math.pi)
        self.declare_parameter('spiral_width', 4)

        # QoS Transient Local agar data tersimpan (latch) untuk subscriber baru (seperti RViz)
        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Publishers & Service
        self.path_pub = self.create_publisher(Path, '/coverage_path', latch_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/coverage_markers', latch_qos)
        self.srv = self.create_service(Trigger, 'compute_coverage_path', self.compute_path_cb)

        # Visualizer Helper
        self.vis = Visualizer(frame_id='map')

        self.last_path = None
        self.last_markers = None
        self.current_field_polygon = None

        # Subscriber boundary lahan dari field_boundary_collector
        self.boundary_sub = self.create_subscription(
            PolygonStamped, '/field_boundary', self.boundary_cb, latch_qos)

        # Timer untuk republish periodik (1.0 detik) agar visualisasi tidak hilang di RViz
        self.create_timer(1.0, self.republish_cb)

        self.get_logger().info("Coverage Server Pipeline siap menerima field boundary.")

    def boundary_cb(self, msg: PolygonStamped):
        self.current_field_polygon = msg.polygon
        self.get_logger().info(
            f"Menerima field boundary baru dengan {len(msg.polygon.points)} titik."
        )

        auto_compute = self.get_parameter('auto_compute').get_parameter_value().bool_value
        if auto_compute:
            self.get_logger().info("Auto-compute aktif: Menghitung coverage path secara otomatis...")
            success, message = self._generate_coverage_path()
            if success:
                self.get_logger().info(f"Auto-compute berhasil: {message}")
            else:
                self.get_logger().warn(f"Auto-compute gagal: {message}")

    def polygon_to_f2c_cells(self, polygon) -> f2c.Cells:
        """Konversi geometry_msgs/Polygon menjadi f2c.Cells (single cell / boundary)."""
        ring = f2c.LinearRing()
        for pt in polygon.points:
            ring.addPoint(f2c.Point(float(pt.x), float(pt.y), 0.0))
        # tutup ring (titik pertama = titik terakhir), Fields2Cover butuh closed ring
        first = polygon.points[0]
        ring.addPoint(f2c.Point(float(first.x), float(first.y), 0.0))

        cell = f2c.Cell()
        cell.addRing(ring)

        cells = f2c.Cells()
        cells.addGeometry(cell)
        return cells

    def republish_cb(self):
        if self.last_path is not None:
            self.last_path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.last_path)
        if self.last_markers is not None:
            self.marker_pub.publish(self.last_markers)

    def _generate_coverage_path(self):
        """Pipeline utama perhitungan coverage path menggunakan Fields2Cover."""
        if self.current_field_polygon is None or len(self.current_field_polygon.points) < 3:
            return False, (
                "Belum ada field boundary valid. Klik titik-titik di RViz dengan "
                "'Publish Point' lalu panggil /finish_field_boundary dulu."
            )

        try:
            robot_width = self.get_parameter('robot_width').get_parameter_value().double_value
            cov_width = self.get_parameter('cov_width').get_parameter_value().double_value
            headland_swaths = self.get_parameter('headland_swaths').get_parameter_value().integer_value
            turning_radius = self.get_parameter('turning_radius').get_parameter_value().double_value
            route_pattern = self.get_parameter('route_pattern').get_parameter_value().string_value
            curve_type = self.get_parameter('curve_type').get_parameter_value().string_value
            swath_angle = self.get_parameter('swath_angle').get_parameter_value().double_value
            decomp_method = self.get_parameter('decomp_method').get_parameter_value().string_value
            split_angle = self.get_parameter('split_angle').get_parameter_value().double_value
            spiral_width = self.get_parameter('spiral_width').get_parameter_value().integer_value

            # 1. Setup Robot & Lahan dari Polygon
            robot = f2c.Robot(robot_width, cov_width)
            raw_cells = self.polygon_to_f2c_cells(self.current_field_polygon)

            # 2. Cell Decomposition (Boustrophedon / Trapezoidal / None)
            decomp_gen = DecompGenerator(method=decomp_method, split_angle=split_angle)
            decomp_cells = decomp_gen.decompose(raw_cells)

            # 3. Headland Generation
            hl_gen = HeadlandGenerator(robot.getWidth())
            no_hl = hl_gen.generate(decomp_cells, headland_swaths=headland_swaths)

            # 4. Swath Generation
            swath_gen = SwathGenerator(robot.getCovWidth())
            swaths = swath_gen.generate(no_hl, angle=swath_angle)
            if swaths.size() == 0:
                return False, (
                    "Swath generation menghasilkan 0 swath. Kemungkinan luas area terlalu kecil "
                    "dibanding lebar coverage robot, atau polygon boundary tidak sesuai."
                )

            # 5. Route Planning (OR-Tools / Snake / Boustrophedon / Spiral)
            # Jika spiral_width <= 0 (Auto), hitung dari diameter putar robot: ceil(2 * R_min / W_cov)
            eff_spiral_width = spiral_width
            if eff_spiral_width <= 0:
                if turning_radius > 0.0 and cov_width > 0.0:
                    eff_spiral_width = max(3, math.ceil((2.0 * turning_radius) / cov_width))
                else:
                    eff_spiral_width = 4

            route_gen = RouteGenerator(pattern=route_pattern, spiral_width=eff_spiral_width)
            route = route_gen.generate(decomp_cells, swaths)

            # 6. Path Planning (Dubins / Reeds-Shepp Curves)
            path_gen = PathGenerator(turning_radius=turning_radius, curve_type=curve_type)
            f2c_path = path_gen.generate(robot, route)

            # 7. Convert to ROS 2 Path
            ros_path = Path()
            ros_path.header.frame_id = 'map'
            ros_path.header.stamp = self.get_clock().now().to_msg()

            for i in range(f2c_path.size()):
                state = f2c_path.getState(i)
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = ros_path.header.stamp
                pose.pose.position.x = state.point.getX()
                pose.pose.position.y = state.point.getY()
                pose.pose.position.z = 0.05

                # Orientasi Yaw
                yaw = state.angle
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)

                ros_path.poses.append(pose)

            # 8. Buat Markers
            markers = self.vis.create_path_markers(f2c_path)

            # Simpan cache untuk republish periodik & publish sekarang
            self.last_path = ros_path
            self.last_markers = markers

            self.path_pub.publish(ros_path)
            self.marker_pub.publish(markers)

            msg = f"Path berhasil digenerate ({decomp_method} decomp, {len(ros_path.poses)} waypoints, {route_pattern})."
            return True, msg

        except Exception as e:
            return False, f"Error saat kalkulasi path: {str(e)}"

    def compute_path_cb(self, request, response):
        success, message = self._generate_coverage_path()
        response.success = success
        response.message = message
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        return response

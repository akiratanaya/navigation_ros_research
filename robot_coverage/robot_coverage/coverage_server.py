#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import fields2cover as f2c

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PolygonStamped
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
        
        # QoS Transient Local agar data tersimpan (latch) untuk subscriber baru (seperti RViz / topic echo)
        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publishers & Service
        self.path_pub = self.create_publisher(Path, '/coverage_path', latch_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/coverage_markers', latch_qos)
        self.srv = self.create_service(Trigger, 'compute_coverage_path', self.compute_path_cb)
        
        # Visualizer Helper
        self.vis = Visualizer(frame_id='map')
        self.get_logger().info("Coverage Server Pipeline initialized.")
        self.last_path = None
        self.last_markers = None
        self.create_timer(1.0, self.republish_cb)  # republish tiap 1 detik
                # State untuk boundary lahan asli (dari field_boundary_collector)
        self.current_field_polygon = None  # akan diisi geometry_msgs/Polygon

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.boundary_sub = self.create_subscription(
            PolygonStamped, '/field_boundary', self.boundary_cb, latched_qos)

    def boundary_cb(self, msg: PolygonStamped):
        self.current_field_polygon = msg.polygon
        self.get_logger().info(
            f"Menerima field boundary baru dengan {len(msg.polygon.points)} titik."
        )

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

    def compute_path_cb(self, request, response):
        try:
            # # 1. Setup Robot & Dummy Field (Lahan Sintetis)
            # robot = f2c.Robot(2.0, 6.0)
            # rand = f2c.Random(100)
            # field = rand.generateRandField(1e4, 9)

            # # 2. Cell Decomposition (Menangani lahan kompleks/L/U)
            # decomp_gen = DecompGenerator(method="boustrophedon", split_angle=0.5 * math.pi)
            # decomp_cells = decomp_gen.decompose(field.getField())

                        # 1. Cek boundary lahan asli sudah ada atau belum
            if self.current_field_polygon is None or len(self.current_field_polygon.points) < 3:
                response.success = False
                response.message = (
                    "Belum ada field boundary. Klik titik-titik di RViz dengan "
                    "'Publish Point' lalu panggil /finish_field_boundary dulu."
                )
                self.get_logger().warn(response.message)
                return response

            robot = f2c.Robot(0.1, 0.3)
            raw_cells = self.polygon_to_f2c_cells(self.current_field_polygon)

            # 2. Cell Decomposition (Menangani lahan kompleks/L/U)
            decomp_gen = DecompGenerator(method="boustrophedon", split_angle=0.5 * math.pi)
            decomp_cells = decomp_gen.decompose(raw_cells)

            # 3. Headland Generation
            hl_gen = HeadlandGenerator(robot.getWidth())
            no_hl = hl_gen.generate(decomp_cells, headland_swaths=1)

            # 4. Swath Generation
            swath_gen = SwathGenerator(robot.getCovWidth())
            swaths = swath_gen.generate(no_hl, angle=math.pi / 2.0)
            if swaths.size() == 0:
                response.success = False
                response.message = (
                    "Swath generation menghasilkan 0 swath. Kemungkinan field terlalu kecil "
                    "dibanding lebar coverage robot, atau polygon boundary tidak valid."
                )
                self.get_logger().error(response.message)
                return response

            # 5. Route Planning (Menggunakan OR-Tools / TSP Solver)
            route_gen = RouteGenerator(pattern="or_tools")
            route = route_gen.generate(decomp_cells, swaths)

            # 6. Path Planning (Dubins / Reeds-Shepp Curves)
            path_gen = PathGenerator(turning_radius=0.0, curve_type="dubins")
            f2c_path = path_gen.generate(robot, route)

            # 7. Convert & Publish ROS 2 Path
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
                
                # Orientasi Yaw
                yaw = state.angle
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                
                ros_path.poses.append(pose)

            # Publish Topic Path & Visualizer Marker
            self.path_pub.publish(ros_path)
            markers = self.vis.create_path_markers(f2c_path)
            self.marker_pub.publish(markers)

            response.success = True
            response.message = f"Path generated with {len(ros_path.poses)} waypoints using OR-Tools."
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(response.message)
            self.last_path = ros_path
            self.last_markers = markers

        return response


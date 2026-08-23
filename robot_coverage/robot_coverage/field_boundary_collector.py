#!/usr/bin/env python3
"""
Node untuk mengumpulkan titik-titik boundary lahan dari klik "Publish Point" di RViz.

Cara pakai:
1. Jalankan node ini.
2. Di RViz, klik tool "Publish Point" di toolbar, lalu klik satu-satu di atas map
   mengelilingi batas lahan (urutan klik = urutan titik polygon).
3. Setelah semua titik selesai diklik (minimal 3 titik), panggil:
       ros2 service call /finish_field_boundary std_srvs/srv/Trigger "{}"
   Ini akan publish polygon ke topic /field_boundary dan mereset titik untuk polygon berikutnya.
4. Kalau salah klik dan mau mulai ulang tanpa publish, panggil:
       ros2 service call /reset_field_boundary std_srvs/srv/Trigger "{}"
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PointStamped, PolygonStamped, Point32, Point
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

try:
    from shapely.geometry import Polygon as ShapelyPolygon
    SHAPELY_AVAILABLE = True
except ImportError:
    SHAPELY_AVAILABLE = False


class FieldBoundaryCollector(Node):
    def __init__(self):
        super().__init__('field_boundary_collector')

        self.points = []  # list of (x, y, z) dari klik-an

        # Subscribe ke titik yang diklik lewat "Publish Point" di RViz
        self.clicked_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_cb, 10)

        # Publisher polygon hasil akhir.
        # TRANSIENT_LOCAL supaya subscriber yang connect belakangan (misal coverage_server
        # yang baru start setelah ini) tetap dapat polygon terakhir yang pernah dipublish.
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.boundary_pub = self.create_publisher(
            PolygonStamped, '/field_boundary', latched_qos)

        # Marker untuk preview titik & garis boundary saat masih diklik-klik (opsional tapi membantu)
        self.marker_pub = self.create_publisher(Marker, '/field_boundary_preview', 10)

        # Service untuk finalize polygon
        self.finish_srv = self.create_service(
            Trigger, 'finish_field_boundary', self.finish_cb)

        # Service untuk reset kalau salah klik
        self.reset_srv = self.create_service(
            Trigger, 'reset_field_boundary', self.reset_cb)

        self.get_logger().info(
            "Field Boundary Collector siap. Klik 'Publish Point' di RViz untuk menambah titik, "
            "lalu panggil /finish_field_boundary setelah selesai."
        )

    def clicked_point_cb(self, msg: PointStamped):
        self.points.append((msg.point.x, msg.point.y, msg.point.z))
        self.get_logger().info(
            f"Titik #{len(self.points)} ditambahkan: "
            f"({msg.point.x:.3f}, {msg.point.y:.3f}) [frame: {msg.header.frame_id}]"
        )
        self.publish_preview_marker(msg.header.frame_id)

    def publish_preview_marker(self, frame_id: str):
        marker = Marker()
        marker.header.frame_id = frame_id if frame_id else 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'field_boundary_preview'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.15
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0

        marker.points = [Point(x=x, y=y, z=z) for (x, y, z) in self.points]

        # tutup loop biar keliatan sebagai polygon preview (kalau titik >= 3)
        if len(self.points) >= 3:
            x0, y0, z0 = self.points[0]
            marker.points.append(Point(x=x0, y=y0, z=z0))

        self.marker_pub.publish(marker)

    def finish_cb(self, request, response):
        if len(self.points) < 3:
            response.success = False
            response.message = (
                f"Butuh minimal 3 titik untuk membentuk polygon, baru ada {len(self.points)}. "
                "Klik lagi di RViz sebelum panggil service ini."
            )
            self.get_logger().warn(response.message)
            return response

        # Validasi polygon: cek self-intersection (bentuk "jam pasir"/bowtie) sebelum publish.
        # Polygon yang sisi-sisinya saling silang akan bikin Fields2Cover/GEOS crash fatal
        # di tahap decomposition, jadi harus ditolak di sini dulu.
        if SHAPELY_AVAILABLE:
            xy_points = [(x, y) for (x, y, z) in self.points]
            shapely_poly = ShapelyPolygon(xy_points)
            if not shapely_poly.is_valid:
                response.success = False
                response.message = (
                    "Polygon tidak valid (kemungkinan sisi-sisinya saling silang / bentuk "
                    "'jam pasir'). Pastikan klik titik SEARAH mengelilingi area (jangan "
                    "melompat menyilang), lalu panggil /reset_field_boundary dan ulangi."
                )
                self.get_logger().error(response.message)
                return response
        else:
            self.get_logger().warn(
                "shapely tidak terinstall — validasi self-intersection dilewati. "
                "Pastikan titik diklik SEARAH mengelilingi area untuk menghindari crash."
            )

        polygon_msg = PolygonStamped()
        polygon_msg.header.frame_id = 'map'
        polygon_msg.header.stamp = self.get_clock().now().to_msg()
        polygon_msg.polygon.points = [
            Point32(x=float(x), y=float(y), z=float(z)) for (x, y, z) in self.points
        ]

        self.boundary_pub.publish(polygon_msg)

        response.success = True
        response.message = f"Boundary dipublish dengan {len(self.points)} titik ke /field_boundary."
        self.get_logger().info(response.message)

        # reset untuk sesi berikutnya
        self.points = []

        return response

    def reset_cb(self, request, response):
        n = len(self.points)
        self.points = []
        response.success = True
        response.message = f"Direset. {n} titik dihapus."
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FieldBoundaryCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
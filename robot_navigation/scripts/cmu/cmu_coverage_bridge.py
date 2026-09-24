#!/usr/bin/env python3
"""
cmu_coverage_bridge.py — Bridge Between Fields2Cover and CMU Autonomy Stack
=============================================================================
Menghubungkan jalur sapuan global (coverage swaths) dari Fields2Cover dengan
CMU Base Autonomy Stack (terrainAnalysis, localPlanner, pathFollower) untuk
menjelajah dan menyisir medan 3D berbukit/berkontur (outdoor park).

Fungsi Utama:
1. Menerima /coverage_path dari Fields2Cover (nav_msgs/Path).
2. Memantau posisi 3D robot dari /state_estimation (LIO-SAM).
3. Monotonic Lookahead Waypoint Tracking:
   - Memproyeksikan target waypoint di depan robot (lookahead) ke topic /way_point
     (geometry_msgs/PointStamped) untuk dikonsumsi oleh CMU localPlanner.
   - Menggeser target maju saat robot berada dalam radius reach_threshold.
   - Mencegah osilasi dan hentakan dengan lookahead target yang mulus.
4. Auto-Boundary Publisher:
   - Otomatis mempublikasikan poligon batas area padang rumput berkontur (/field_boundary)
     sehingga Fields2Cover langsung menghitung jalur tanpa perlu klik manual.
   - Tetap mendukung klik manual di RViz kapan saja via /field_boundary.
5. Visualisasi RViz Terpadu:
   - Target waypoint marker aktif (/cmu_coverage/target_marker)
   - Sisa jalur aktif yang belum tersapu (/cmu_coverage/active_path)
   - Metrik status progress misi real-time (/cmu_coverage/status & /cmu_coverage/progress)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PointStamped, PolygonStamped, Point32, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import Trigger


class CmuCoverageBridge(Node):
    def __init__(self):
        super().__init__('cmu_coverage_bridge')

        # ── Parameter ──────────────────────────────────────────────────────────
        self.declare_parameter('reach_threshold', 0.65)      # Radius pencapaian waypoint (meter)
        self.declare_parameter('lookahead_dist', 1.20)       # Jarak lookahead CMU local planner (meter)
        self.declare_parameter('waypoint_rate', 10.0)        # Frekuensi kirim /way_point (Hz)
        self.declare_parameter('auto_boundary', True)        # Otomatis trigger batas padang rumput
        self.declare_parameter('auto_boundary_delay', 7.5)   # Detik tunda sebelum auto-boundary
        self.declare_parameter('boundary_x_min', -7.0)       # Batas area padang rumput outdoor park
        self.declare_parameter('boundary_x_max', 7.0)
        self.declare_parameter('boundary_y_min', -5.0)
        self.declare_parameter('boundary_y_max', 5.0)

        self.reach_threshold = self.get_parameter('reach_threshold').get_parameter_value().double_value
        self.lookahead_dist = self.get_parameter('lookahead_dist').get_parameter_value().double_value
        self.waypoint_rate = self.get_parameter('waypoint_rate').get_parameter_value().double_value
        self.auto_boundary = self.get_parameter('auto_boundary').get_parameter_value().bool_value
        self.auto_boundary_delay = self.get_parameter('auto_boundary_delay').get_parameter_value().double_value

        self.bx_min = self.get_parameter('boundary_x_min').get_parameter_value().double_value
        self.bx_max = self.get_parameter('boundary_x_max').get_parameter_value().double_value
        self.by_min = self.get_parameter('boundary_y_min').get_parameter_value().double_value
        self.by_max = self.get_parameter('boundary_y_max').get_parameter_value().double_value

        # ── QoS Profiles ───────────────────────────────────────────────────────
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        standard_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ── Publishers ─────────────────────────────────────────────────────────
        # Output utama ke CMU Local Planner
        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', standard_qos)

        # Field boundary publisher (untuk memicu Fields2Cover secara otomatis)
        self.boundary_pub = self.create_publisher(PolygonStamped, '/field_boundary', latched_qos)
        self.boundary_preview_pub = self.create_publisher(Marker, '/field_boundary_preview', 10)

        # Visualisasi & Telemetri Coverage Bridge
        self.target_marker_pub = self.create_publisher(Marker, '/cmu_coverage/target_marker', 10)
        self.active_path_pub = self.create_publisher(Path, '/cmu_coverage/active_path', 10)
        self.status_pub = self.create_publisher(String, '/cmu_coverage/status', 10)
        self.progress_pub = self.create_publisher(Float32, '/cmu_coverage/progress', 10)

        # ── Subscribers ────────────────────────────────────────────────────────
        # 1. Jalur Coverage dari Fields2Cover
        self.coverage_path_sub = self.create_subscription(
            Path, '/coverage_path', self.coverage_path_callback, latched_qos)

        # 2. Odometri 6-DoF dari LIO-SAM CMU Bridge
        self.odom_sub = self.create_subscription(
            Odometry, '/state_estimation', self.odometry_callback, standard_qos)

        # 3. Pantau jika ada boundary manual yang dipublish
        self.boundary_sub = self.create_subscription(
            PolygonStamped, '/field_boundary', self.boundary_callback, latched_qos)

        # ── Services ───────────────────────────────────────────────────────────
        self.srv_start = self.create_service(Trigger, '/start_coverage', self.srv_start_cb)
        self.srv_pause = self.create_service(Trigger, '/pause_coverage', self.srv_pause_cb)
        self.srv_reset = self.create_service(Trigger, '/reset_coverage', self.srv_reset_cb)
        self.srv_boundary = self.create_service(Trigger, '/trigger_park_boundary', self.srv_boundary_cb)

        # ── State Variables ────────────────────────────────────────────────────
        self.state = 'IDLE'   # 'IDLE', 'TRACKING', 'PAUSED', 'COMPLETED'
        self.waypoints = []   # list of (x, y, z)
        self.current_idx = 0
        self.robot_x = None
        self.robot_y = None
        self.robot_z = None
        self.total_waypoints = 0
        self.start_time = None
        self.boundary_published = False

        # ── Timers ─────────────────────────────────────────────────────────────
        # Control Loop (10 Hz)
        self.timer_period = 1.0 / max(1.0, self.waypoint_rate)
        self.control_timer = self.create_timer(self.timer_period, self.control_loop)

        # Auto-boundary Timer (Trigger satu kali setelah sistem stabil)
        if self.auto_boundary:
            self.auto_boundary_timer = self.create_timer(1.0, self.auto_boundary_check)
            self.node_start_time = self.get_clock().now()

        self.get_logger().info(
            f"CMU Coverage Bridge aktif. Menghubungkan Fields2Cover -> CMU 3D Local Planner."
        )

    # ══════════════════════════════════════════════════════════════════════════
    #  AUTO-BOUNDARY PUBLISHER (Outdoor Meadow Clearing)
    # ══════════════════════════════════════════════════════════════════════════

    def auto_boundary_check(self):
        """Memicu pengiriman field boundary otomatis untuk outdoor park clearing."""
        if self.boundary_published:
            return

        elapsed = (self.get_clock().now() - self.node_start_time).nanoseconds / 1e9
        if elapsed >= self.auto_boundary_delay:
            self.publish_park_boundary()
            self.boundary_published = True
            if hasattr(self, 'auto_boundary_timer') and self.auto_boundary_timer is not None:
                self.auto_boundary_timer.cancel()

    def publish_park_boundary(self):
        """Kirim poligon area padang rumput berbukit outdoor park ke /field_boundary."""
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Buat batas persegi empat mengelilingi meadow terbuka bebas pohon besar
        corners = [
            (self.bx_min, self.by_min),
            (self.bx_max, self.by_min),
            (self.bx_max, self.by_max),
            (self.bx_min, self.by_max),
        ]

        for x, y in corners:
            pt = Point32()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0
            msg.polygon.points.append(pt)

        self.boundary_pub.publish(msg)
        self.get_logger().info(
            f"🏞️  Batas lahan otomatis dipublikasikan: X[{self.bx_min:.1f} .. {self.bx_max:.1f}], "
            f"Y[{self.by_min:.1f} .. {self.by_max:.1f}] ({len(corners)} titik)."
        )

        # Publikasikan marker preview boundary agar terlihat rapi di RViz
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = msg.header.stamp
        marker.ns = 'field_boundary_preview'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.15
        marker.color.r = 1.0
        marker.color.g = 0.65
        marker.color.b = 0.0
        marker.color.a = 0.9
        marker.pose.orientation.w = 1.0

        for x, y in corners:
            marker.points.append(Point(x=float(x), y=float(y), z=0.05))
        # Tutup loop
        marker.points.append(Point(x=float(corners[0][0]), y=float(corners[0][1]), z=0.05))

        self.boundary_preview_pub.publish(marker)

    # ══════════════════════════════════════════════════════════════════════════
    #  CALLBACKS
    # ══════════════════════════════════════════════════════════════════════════

    def boundary_callback(self, msg: PolygonStamped):
        """Catat jika ada boundary baru dari pengguna / sistem."""
        self.boundary_published = True

    def odometry_callback(self, msg: Odometry):
        """Perbarui estimasi pose robot dari LIO-SAM."""
        pos = msg.pose.pose.position
        self.robot_x = pos.x
        self.robot_y = pos.y
        self.robot_z = pos.z

    def coverage_path_callback(self, msg: Path):
        """Terima jalur sapuan dari Fields2Cover dan inisialisasi tracking."""
        if not msg.poses:
            self.get_logger().warn("Menerima /coverage_path kosong.")
            return

        new_waypoints = []
        for p in msg.poses:
            pos = p.pose.position
            new_waypoints.append((pos.x, pos.y, pos.z))

        self.waypoints = new_waypoints
        self.total_waypoints = len(new_waypoints)
        self.current_idx = 0
        self.state = 'TRACKING'
        self.start_time = self.get_clock().now()

        self.get_logger().info(
            f"🚜 Jalur Coverage Baru Diterima: {self.total_waypoints} waypoints! "
            f"Memulai tracking menuju CMU Local Planner..."
        )

    # ══════════════════════════════════════════════════════════════════════════
    #  CONTROL LOOP: LOOKAHEAD WAYPOINT TRACKING
    # ══════════════════════════════════════════════════════════════════════════

    def control_loop(self):
        """Loop kendali 10 Hz untuk mengirim target /way_point ke CMU localPlanner."""
        if self.state != 'TRACKING' or not self.waypoints or self.robot_x is None:
            return

        rx, ry, rz = self.robot_x, self.robot_y, self.robot_z
        n_pts = len(self.waypoints)

        # ── 1. Monotonic Advance Waypoint Index ──
        # Majukan indeks progres saat robot mendekati waypoint saat ini
        while self.current_idx < n_pts - 1:
            wx, wy, wz = self.waypoints[self.current_idx]
            dist = math.hypot(rx - wx, ry - wy)

            if dist < self.reach_threshold:
                self.current_idx += 1
            else:
                # Periksa apakah robot sudah melewati waypoint ini (vektor proyeksi)
                if self.current_idx < n_pts - 1:
                    next_wx, next_wy, _ = self.waypoints[self.current_idx + 1]
                    seg_dx = next_wx - wx
                    seg_dy = next_wy - wy
                    seg_len = math.hypot(seg_dx, seg_dy)
                    if seg_len > 1e-3:
                        rob_dx = rx - wx
                        rob_dy = ry - wy
                        proj = (rob_dx * seg_dx + rob_dy * seg_dy) / (seg_len * seg_len)
                        # Jika proyeksi > 1.0 dan jarak transversal dekat, robot sudah melampaui waypoint
                        if proj > 1.0 and dist < (self.reach_threshold * 1.5):
                            self.current_idx += 1
                            continue
                break

        # ── 2. Cek Selesai Misi ──
        if self.current_idx >= n_pts - 1:
            last_wx, last_wy, _ = self.waypoints[-1]
            dist_final = math.hypot(rx - last_wx, ry - last_wy)
            if dist_final < max(0.4, self.reach_threshold * 0.8):
                self.state = 'COMPLETED'
                self.get_logger().info("🏆 MISI COVERAGE 3D SELESAI! Seluruh medan telah berhasil disisir.")
                self.publish_status_telemetry(100.0, 0.0)
                return

        # ── 3. Cari Target Waypoint dengan Lookahead Distance ──
        # CMU localPlanner bekerja optimal ketika diberi target lookahead 1.0 - 1.5m di depannya
        target_idx = self.current_idx
        for k in range(self.current_idx, n_pts):
            kwx, kwy, _ = self.waypoints[k]
            dist_k = math.hypot(kwx - rx, kwy - ry)
            if dist_k >= self.lookahead_dist or k == n_pts - 1:
                target_idx = k
                break

        tx, ty, tz = self.waypoints[target_idx]
        dist_to_target = math.hypot(tx - rx, ty - ry)

        # ── 4. Publish ke /way_point (Konsumsi CMU localPlanner) ──
        wp_msg = PointStamped()
        wp_msg.header.stamp = self.get_clock().now().to_msg()
        wp_msg.header.frame_id = 'map'
        wp_msg.point.x = float(tx)
        wp_msg.point.y = float(ty)
        # CMU localPlanner akan memproyeksikan X & Y ke elevasi terrainCloud secara otomatis
        wp_msg.point.z = float(rz) if rz is not None else float(tz)
        self.waypoint_pub.publish(wp_msg)

        # ── 5. Telemetri & Visualisasi Marker ──
        progress = (float(self.current_idx) / float(max(1, n_pts - 1))) * 100.0
        self.publish_status_telemetry(progress, dist_to_target)
        self.publish_target_marker(tx, ty, rz if rz is not None else tz)
        self.publish_active_path()

    def publish_status_telemetry(self, progress: float, dist_to_target: float):
        """Kirim string status dan persentase numerik."""
        status_text = (
            f"[{self.state}] Wp: {self.current_idx}/{self.total_waypoints} "
            f"({progress:.1f}%) | D_target: {dist_to_target:.2f}m"
        )
        self.status_pub.publish(String(data=status_text))
        self.progress_pub.publish(Float32(data=float(progress)))

    def publish_target_marker(self, x: float, y: float, z: float):
        """Marker bola terang di RViz menandai target waypoint aktif CMU localPlanner."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'cmu_coverage_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z + 0.35)  # Melayang sedikit di atas tanah agar mudah dilihat
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.40
        marker.scale.y = 0.40
        marker.scale.z = 0.40
        # Warna kuning emas terang
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.1
        marker.color.a = 0.95
        self.target_marker_pub.publish(marker)

    def publish_active_path(self):
        """Publikasikan sisa jalur yang belum dilewati ke RViz."""
        if not self.waypoints or self.current_idx >= len(self.waypoints):
            return

        # Ambil subset waypoints dari current_idx ke akhir
        active_pts = self.waypoints[self.current_idx:]
        if len(active_pts) < 2:
            return

        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        for pt in active_pts:
            from geometry_msgs.msg import PoseStamped
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = float(pt[0])
            p.pose.position.y = float(pt[1])
            p.pose.position.z = float(pt[2])
            p.pose.orientation.w = 1.0
            msg.poses.append(p)

        self.active_path_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════════════════
    #  SERVICE HANDLERS
    # ══════════════════════════════════════════════════════════════════════════

    def srv_start_cb(self, request, response):
        if not self.waypoints:
            response.success = False
            response.message = "Belum ada jalur coverage. Kirim field boundary terlebih dahulu."
            return response

        self.state = 'TRACKING'
        response.success = True
        response.message = f"Coverage dilanjutkan pada waypoint {self.current_idx}/{self.total_waypoints}."
        self.get_logger().info(f"▶️  {response.message}")
        return response

    def srv_pause_cb(self, request, response):
        self.state = 'PAUSED'
        response.success = True
        response.message = f"Coverage dihentikan sementara (PAUSED) pada waypoint {self.current_idx}."
        self.get_logger().info(f"⏸️  {response.message}")
        return response

    def srv_reset_cb(self, request, response):
        self.current_idx = 0
        self.state = 'TRACKING' if self.waypoints else 'IDLE'
        response.success = True
        response.message = "Indeks waypoint di-reset ke awal (0)."
        self.get_logger().info(f"🔄 {response.message}")
        return response

    def srv_boundary_cb(self, request, response):
        self.publish_park_boundary()
        response.success = True
        response.message = "Batas outdoor park clearing berhasil dipublikasikan."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CmuCoverageBridge()
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

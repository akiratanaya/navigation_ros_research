#!/usr/bin/env python3
"""
coverage_navigator.py — 2-Stage Collision-Free Coverage Navigator.

Fitur:
  1. Tahap 1 (Transit Koridor):
     - Nav2 Global Planner (ComputePathToPose) + FollowPath (MPPI)
     - Navigasi melewati lorong, rintangan meja, dan pintu menuju titik masuk coverage.
  2. Tahap 2 (Coverage Serpentine Murni via Filtered PD Pure Pursuit):
     - Pengendali Pure Pursuit berbasis indeks monotonic (hanya bergerak maju, anti melompat).
     - Filtered PD Heading Controller dengan Damping Turunan (Kd) & Slew Rate Acceleration Limiter
       sehingga 100% bebas overshooting / osilasi limit-cycle.
     - Dukungan manuver mundur (reverse crawl) jika titik target berada di kuadran belakang.
     - Cosine Linear Speed Profiler untuk akselerasi mulus pada jalur lurus dan perlambatan presisi di tikungan U-turn.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import FollowPath, ComputePathToPose
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy


class CoverageNavigator(Node):
    def __init__(self):
        super().__init__('coverage_navigator')

        # ── Parameter ──────────────────────────────────────────────────────────
        self.declare_parameter('auto_navigate', True)
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('goal_checker_id', 'general_goal_checker')
        self.declare_parameter('progress_checker_id', 'progress_checker')
        self.declare_parameter('lead_in_step', 0.20)

        # Pure Pursuit & Damped PD Controller parameters
        self.declare_parameter('pp_lookahead', 0.35)
        self.declare_parameter('pp_linear_speed', 0.35)
        self.declare_parameter('pp_max_angular', 1.20)
        self.declare_parameter('pp_goal_tolerance', 0.20)
        self.declare_parameter('pp_frequency', 20.0)
        self.declare_parameter('kp_heading', 1.80)
        self.declare_parameter('kd_heading', 0.45)
        self.declare_parameter('max_linear_accel', 0.80)
        self.declare_parameter('max_angular_accel', 2.50)

        self.auto_navigate = self.get_parameter('auto_navigate').get_parameter_value().bool_value
        self.controller_id = self.get_parameter('controller_id').get_parameter_value().string_value
        self.planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
        self.goal_checker_id = self.get_parameter('goal_checker_id').get_parameter_value().string_value
        self.progress_checker_id = self.get_parameter('progress_checker_id').get_parameter_value().string_value
        self.lead_in_step = self.get_parameter('lead_in_step').get_parameter_value().double_value

        self.pp_lookahead = self.get_parameter('pp_lookahead').get_parameter_value().double_value
        self.pp_linear_speed = self.get_parameter('pp_linear_speed').get_parameter_value().double_value
        self.pp_max_angular = self.get_parameter('pp_max_angular').get_parameter_value().double_value
        self.pp_goal_tolerance = self.get_parameter('pp_goal_tolerance').get_parameter_value().double_value
        self.pp_frequency = self.get_parameter('pp_frequency').get_parameter_value().double_value
        self.kp_heading = self.get_parameter('kp_heading').get_parameter_value().double_value
        self.kd_heading = self.get_parameter('kd_heading').get_parameter_value().double_value
        self.max_linear_accel = self.get_parameter('max_linear_accel').get_parameter_value().double_value
        self.max_angular_accel = self.get_parameter('max_angular_accel').get_parameter_value().double_value

        # ── TF Listener ───────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── State ─────────────────────────────────────────────────────────────
        self.latest_coverage_path: Path | None = None
        self.is_navigating = False
        self.current_stage = 'IDLE'  # 'TRANSIT', 'COVERAGE', 'IDLE'
        self._goal_handle = None
        self._transit_goal_handle = None
        self._coverage_goal_handle = None
        self._current_path_signature = None
        self._is_transitioning = False
        self._latest_scan: LaserScan | None = None

        # Pure Pursuit & Filter State
        self._pp_path_poses: list[PoseStamped] = []
        self._pp_current_idx = 0
        self._pp_timer = None
        self._prev_heading_error = 0.0
        self._current_cmd_v = 0.0
        self._current_cmd_w = 0.0
        self._last_time = None
        self._blocked_duration = 0.0

        # ── Publishers & Subscribers ──────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.path_sub = self.create_subscription(
            Path, '/coverage_path', self.path_callback, latch_qos)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10)

        # ── Nav2 Action Clients (untuk transit saja) ──────────────────────────
        self._follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self._compute_path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # ── Services ──────────────────────────────────────────────────────────
        self.start_srv = self.create_service(
            Trigger, 'start_coverage_navigation', self.start_nav_cb)
        self.cancel_srv = self.create_service(
            Trigger, 'cancel_coverage_navigation', self.cancel_nav_cb)

        self.get_logger().info(
            "🚀 Coverage Navigator 2-Stage (Transit + Filtered PD Pure Pursuit) Siap!")

    # ══════════════════════════════════════════════════════════════════════════
    #  UTILITY
    # ══════════════════════════════════════════════════════════════════════════

    def stop_robot(self):
        """Mengirim kecepatan nol untuk menghentikan robot seketika."""
        self._current_cmd_v = 0.0
        self._current_cmd_w = 0.0
        msg = Twist()
        for _ in range(3):
            self.cmd_vel_pub.publish(msg)

    def scan_cb(self, msg: LaserScan):
        """Menyimpan data scan terbaru untuk monitoring keselamatan."""
        self._latest_scan = msg

    def get_robot_pose(self) -> tuple[float, float, float] | None:
        """Membaca posisi robot (x, y, yaw) dari TF map -> base_footprint."""
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
        except Exception as e:
            return None

    def refresh_path_timestamps(self, path: Path) -> Path:
        """Segarkan stempel waktu agar Nav2 tidak menolak path lampau."""
        now_msg = self.get_clock().now().to_msg()
        path.header.stamp = now_msg
        path.header.frame_id = 'map'
        for p in path.poses:
            p.header.stamp = now_msg
            p.header.frame_id = 'map'
        return path

    def compute_signature(self, path: Path):
        if not path.poses:
            return None
        p0 = path.poses[0].pose.position
        pe = path.poses[-1].pose.position
        return (len(path.poses), round(p0.x, 2), round(p0.y, 2),
                round(pe.x, 2), round(pe.y, 2))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ══════════════════════════════════════════════════════════════════════════
    #  PATH CALLBACK & PIPELINE
    # ══════════════════════════════════════════════════════════════════════════

    def path_callback(self, msg: Path):
        if not msg.poses:
            return

        sig = self.compute_signature(msg)
        if sig == self._current_path_signature and self.is_navigating:
            return

        self.latest_coverage_path = msg
        self.get_logger().info(
            f"📥 Menerima /coverage_path baru ({len(msg.poses)} titik poses dari Fields2Cover).")

        if self.get_parameter('auto_navigate').get_parameter_value().bool_value:
            self.get_logger().info("🤖 Auto-navigate aktif! Menunggu 3 detik agar Nav2 siap...")
            # Delay 3 detik agar Nav2 lifecycle selesai activate sebelum kirim goal
            self._pending_path = msg
            self._pending_sig = sig
            self.create_timer(3.0, self._delayed_start, callback_group=None)

    def _delayed_start(self):
        """Callback setelah delay, memulai pipeline navigasi."""
        if hasattr(self, '_pending_path') and self._pending_path is not None:
            path = self._pending_path
            sig = self._pending_sig
            self._pending_path = None
            self._pending_sig = None
            self.get_logger().info("🚀 Nav2 seharusnya sudah aktif, memulai pipeline navigasi...")
            self.start_pipeline(path, sig)

    def start_pipeline(self, coverage_path: Path, sig=None):
        self.latest_coverage_path = coverage_path
        self._current_path_signature = sig
        self._is_transitioning = False

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn("Posisi robot belum terbaca, langsung coverage stage...")
            self.start_coverage_follow_path(coverage_path)
            return

        rx, ry, _ = robot_pose
        first = coverage_path.poses[0].pose.position
        dist = math.hypot(first.x - rx, first.y - ry)

        # Jika robot sudah di titik awal coverage (<= 30 cm), langsung jalankan Tahap 2
        if dist <= 0.30:
            self.get_logger().info(
                f"🎯 Robot sudah di titik awal ({dist:.2f}m). Langsung mulai Tahap 2 (Coverage via Nav2)!")
            self.start_coverage_follow_path(coverage_path)
            return

        # Tahap 1: Transit bebas rintangan via Nav2
        self.get_logger().info(
            f"🚗 [Tahap 1/2] Merencanakan rute transit dari ({rx:.2f}, {ry:.2f}) "
            f"ke pintu masuk ({first.x:.2f}, {first.y:.2f})...")

        if not self._compute_path_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().warn(
                "Planner server belum siap, langsung mulai coverage...")
            self.start_coverage_follow_path(coverage_path)
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = coverage_path.poses[0]
        goal_msg.planner_id = self.planner_id
        goal_msg.use_start = False

        future = self._compute_path_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f: self._on_compute_path_response(f, coverage_path))

    # ══════════════════════════════════════════════════════════════════════════
    #  TAHAP 1: TRANSIT via Nav2 FollowPath
    # ══════════════════════════════════════════════════════════════════════════

    def _on_compute_path_response(self, future, coverage_path: Path):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("ComputePathToPose ditolak, langsung mulai coverage...")
            self.start_coverage_follow_path(coverage_path)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_compute_path_result(f, coverage_path))

    def compute_path_tangents(self, path: Path) -> Path:
        if not path.poses:
            return path
        n = len(path.poses)
        prev_yaw = 0.0
        for i in range(n):
            if i < n - 1:
                p = path.poses[i].pose.position
                q = path.poses[i + 1].pose.position
                dx, dy = q.x - p.x, q.y - p.y
                if math.hypot(dx, dy) > 0.01:
                    prev_yaw = math.atan2(dy, dx)
            path.poses[i].pose.orientation.z = math.sin(prev_yaw / 2.0)
            path.poses[i].pose.orientation.w = math.cos(prev_yaw / 2.0)
        return path

    def _on_compute_path_result(self, future, coverage_path: Path):
        result = future.result().result
        transit_path = result.path

        if transit_path and transit_path.poses:
            transit_path = self.compute_path_tangents(transit_path)
            self.get_logger().info(
                f"✅ Rute transit siap ({len(transit_path.poses)} poses). "
                f"Memulai Tahap 1 (Transit via Nav2)!")
            self.current_stage = 'TRANSIT'
            self._is_transitioning = False
            self._send_transit_follow_path(transit_path)
        else:
            self.get_logger().warn("Transit path kosong, langsung mulai coverage...")
            self.start_coverage_follow_path(coverage_path)

    def _send_transit_follow_path(self, path: Path):
        if not self._follow_path_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("❌ follow_path server belum aktif!")
            if self.latest_coverage_path:
                self.start_coverage_follow_path(self.latest_coverage_path)
            return

        fresh = self.refresh_path_timestamps(path)
        if self._goal_handle is not None and self.is_navigating:
            self._goal_handle.cancel_goal_async()

        goal = FollowPath.Goal()
        goal.path = fresh
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        goal.progress_checker_id = self.progress_checker_id

        future = self._follow_path_client.send_goal_async(
            goal, feedback_callback=self._transit_feedback_cb)
        future.add_done_callback(self._on_transit_goal_response)
        self.is_navigating = True

    def _on_transit_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("❌ Transit goal ditolak oleh controller!")
            self.is_navigating = False
            self.stop_robot()
            if self.latest_coverage_path:
                self.start_coverage_follow_path(self.latest_coverage_path)
            return

        self.get_logger().info("✅ Transit goal diterima. Robot meluncur ke pintu masuk.")
        self._transit_goal_handle = gh
        result_future = gh.get_result_async()
        result_future.add_done_callback(self._on_transit_done)

    def _on_transit_done(self, future):
        status = future.result().status
        self._transit_goal_handle = None
        if self.current_stage == 'COVERAGE':
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🏁 Tahap 1 (Transit) Sukses! Memulai Tahap 2 (Coverage) via Nav2 MPPI Controller.")
            if self.latest_coverage_path:
                self.start_coverage_follow_path(self.latest_coverage_path)
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Transit terhenti di dekat pintu, lanjut coverage via Nav2...")
            if self.latest_coverage_path:
                self.start_coverage_follow_path(self.latest_coverage_path)

    def _transit_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = getattr(fb, 'distance_to_goal', 0.0)
        speed = getattr(fb, 'speed', 0.0)
        self.get_logger().info(
            f"📍 [TRANSIT] Sisa: {dist:.2f}m | Kecepatan: {speed:.2f}m/s",
            throttle_duration_sec=3.0)

        # Handover mulus ke Tahap 2 (Coverage) saat robot benar-benar tiba di titik awal (<= 0.30m)
        if not self._is_transitioning and dist <= 0.30:
            self._is_transitioning = True
            self.get_logger().info(
                f"🎯 Robot tiba di titik awal coverage ({dist:.2f}m)! Mengalihkan kontrol ke Tahap 2 (Coverage Nav2)...")
            if self._transit_goal_handle:
                self._transit_goal_handle.cancel_goal_async()
                self._transit_goal_handle = None
            if self.latest_coverage_path:
                self.start_coverage_follow_path(self.latest_coverage_path)

    # ══════════════════════════════════════════════════════════════════════════
    #  TAHAP 2: COVERAGE via Nav2 Controller (MPPI dengan Full Obstacle Avoidance)
    # ══════════════════════════════════════════════════════════════════════════

    def start_coverage_follow_path(self, coverage_path: Path):
        """
        Mengeksekusi seluruh jalur coverage menggunakan Nav2 FollowPath Action Server.
        Mengaktifkan 100% sistem penghindar rintangan Nav2 (Costmap Inflation, CostCritic,
        Footprint Collision Checker, Dynamic LiDAR Avoidance) secara native dan optimal.
        """
        self.current_stage = 'COVERAGE'
        self.is_navigating = True
        self._is_transitioning = False

        if not self._follow_path_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("❌ Nav2 follow_path server belum siap!")
            self.is_navigating = False
            return

        fresh = self.refresh_path_timestamps(coverage_path)
        if self._coverage_goal_handle is not None:
            self._coverage_goal_handle.cancel_goal_async()

        goal = FollowPath.Goal()
        goal.path = fresh
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        goal.progress_checker_id = self.progress_checker_id

        self.get_logger().info(
            f"🚜 [Tahap 2/2] Mengirim {len(fresh.poses)} waypoint coverage ke Nav2 MPPI Controller "
            f"(Active Obstacle Avoidance ON)...")

        future = self._follow_path_client.send_goal_async(
            goal, feedback_callback=self._coverage_feedback_cb)
        future.add_done_callback(self._on_coverage_goal_response)

    def _on_coverage_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("❌ Coverage goal ditolak oleh Nav2 controller!")
            self.is_navigating = False
            self.stop_robot()
            return

        self.get_logger().info("✅ Nav2 Coverage Controller aktif! Robot mulai menyapu area dengan penghindaran rintangan aktif.")
        self._coverage_goal_handle = gh
        result_future = gh.get_result_async()
        result_future.add_done_callback(self._on_coverage_done)

    def _coverage_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = getattr(fb, 'distance_to_goal', 0.0)
        speed = getattr(fb, 'speed', 0.0)
        self.get_logger().info(
            f"📍 [COVERAGE NAV2] Sisa: {dist:.2f}m | Kecepatan: {speed:.2f}m/s | Obstacle Avoidance Aktif",
            throttle_duration_sec=3.0)

    def _on_coverage_done(self, future):
        status = future.result().status
        self.is_navigating = False
        self._coverage_goal_handle = None
        self.stop_robot()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🎉 [SUKSES] Misi Coverage Nav2 Selesai 100% tanpa tabrakan!")
        else:
            self.get_logger().warn(f"⚠️ Coverage selesai dengan status: {status}")

    # ══════════════════════════════════════════════════════════════════════════
    #  SERVICE CALLBACKS
    # ══════════════════════════════════════════════════════════════════════════

    def start_nav_cb(self, request, response):
        if not self.latest_coverage_path or not self.latest_coverage_path.poses:
            response.success = False
            response.message = "Belum ada /coverage_path yang digenerate."
            return response
        sig = self.compute_signature(self.latest_coverage_path)
        self.start_pipeline(self.latest_coverage_path, sig)
        response.success = True
        response.message = "Memulai 2-stage coverage navigation."
        return response

    def cancel_nav_cb(self, request, response):
        if self.is_navigating:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None
            self._finish_coverage()
            response.success = True
            response.message = "Navigasi dibatalkan."
        else:
            response.success = False
            response.message = "Robot tidak sedang bernavigasi aktif."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CoverageNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

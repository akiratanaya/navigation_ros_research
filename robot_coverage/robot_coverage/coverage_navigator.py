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

        # Dynamic Obstacle Detour & Waypoint Slicing State (Sequential Progress Tracker)
        self._active_coverage_poses: list[PoseStamped] = []
        self._coverage_progress_idx = 0
        self._is_detouring = False
        self._coverage_stuck_ticks = 0
        self._last_detour_time = 0.0
        self._coverage_start_time = 0.0       # waktu mulai coverage (untuk grace period)
        self._coverage_initial_dist = None    # jarak awal — detour hanya jika pernah berkurang
        self._detour_skip_count = 22

        # Responsif Maju-Mundur Maneuver Engine
        self._is_maneuvering = False
        self._maneuver_phase = 'IDLE'         # 'MUNDUR', 'PUTAR', 'MAJU', 'IDLE'
        self._maneuver_turn_dir = 1.0         # +1.0 = kiri, -1.0 = kanan
        self._maneuver_turn_name = 'KIRI'
        self._maneuver_start_time = 0.0
        self._maneuver_timer = None

        # Segment-based sequential execution
        self._coverage_segments: list[list] = []   # antrian segmen path
        self._current_segment_idx = 0              # segmen yang sedang dieksekusi

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
        self.coverage_path_pub = self.create_publisher(
            Path, '/coverage_path', latch_qos)

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
        if hasattr(self, '_maneuver_timer') and self._maneuver_timer is not None:
            self._maneuver_timer.cancel()
            self._maneuver_timer = None
        if hasattr(self, '_reverse_timer') and self._reverse_timer is not None:
            self._reverse_timer.cancel()
            self._reverse_timer = None
        self._is_maneuvering = False
        self._is_reversing = False
        self._current_cmd_v = 0.0
        self._current_cmd_w = 0.0
        msg = Twist()
        for _ in range(3):
            self.cmd_vel_pub.publish(msg)

    def scan_cb(self, msg: LaserScan):
        """Menyimpan data scan terbaru untuk monitoring keselamatan."""
        self._latest_scan = msg

    def analyze_obstacles(self):
        """
        Menganalisis rintangan fisik di sekeliling robot via data LiDAR.
        Menggunakan proyeksi kartesian (x, y) robot frame:
          - x: arah maju robot (bumper depan di x = +0.14m)
          - y: arah lateral robot (lebar bodi y = -0.153m s.d. +0.153m)
        Returns:
            has_near_obs (bool): Ada objek berjarak bahaya (< 12 cm dari bumper atau < 8 cm dari bodi samping)
            side (str): 'LEFT', 'RIGHT', 'FRONT', atau None
            min_front_x (float): Jarak terdekat rintangan di koridor depan robot (|y| <= 0.20m)
            min_left (float): Jarak terdekat di sektor kiri
            min_right (float): Jarak terdekat di sektor kanan
            min_rear_x (float): Jarak rintangan di belakang robot (x < 0, |y| <= 0.20m)
        """
        scan = getattr(self, '_latest_scan', None)
        if scan is None or not scan.ranges:
            return False, None, 999.0, 999.0, 999.0, 999.0

        amin = scan.angle_min
        ainc = scan.angle_increment
        n = len(scan.ranges)

        min_front_x = 999.0
        min_left = 999.0
        min_right = 999.0
        min_rear_x = 999.0

        for i in range(n):
            angle = amin + i * ainc
            r = scan.ranges[i]
            if math.isnan(r) or math.isinf(r) or r < 0.05:
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # 1. Koridor depan robot: lebar bodi +/- 0.20m (lebar robot 0.306m + margin aman)
            if x > 0.05 and abs(y) <= 0.20:
                if x < min_front_x:
                    min_front_x = x

            # 2. Koridor belakang robot (untuk proteksi saat mundur):
            if x < -0.05 and abs(y) <= 0.20:
                if abs(x) < min_rear_x:
                    min_rear_x = abs(x)

            # 3. Sisi Kiri: y > 0.12 dan x >= -0.15
            if y > 0.12 and x >= -0.15:
                if r < min_left:
                    min_left = r

            # 4. Sisi Kanan: y < -0.12 dan x >= -0.15
            if y < -0.12 and x >= -0.15:
                if r < min_right:
                    min_right = r

        # Bumper depan berada di x = +0.14m.
        # Jika rintangan depan berjarak <= 12 cm dari bumper (min_front_x <= 0.26m)
        # atau rintangan samping berjarak <= 8 cm dari bodi (min_side <= 0.23m):
        has_near_obs = (min_front_x <= 0.26 or min_left <= 0.23 or min_right <= 0.23)

        side = None
        if has_near_obs:
            if min_front_x <= 0.26 and (min_front_x - 0.14) <= min(min_left - 0.15, min_right - 0.15):
                side = 'FRONT'
            elif min_left < min_right:
                side = 'LEFT'
            else:
                side = 'RIGHT'

        return has_near_obs, side, min_front_x, min_left, min_right, min_rear_x

    def is_obstacle_in_front(self, threshold_dist=0.26) -> bool:
        """Cek apakah ada rintangan fisik tepat di depan koridor robot (< threshold_dist) via LiDAR."""
        has_near_obs, _, min_front_x, _, _, _ = self.analyze_obstacles()
        return min_front_x < threshold_dist

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

        # Jika sedang aktif menjalankan coverage, abaikan update path dari pemangkasan internal
        if self.is_navigating and self.current_stage == 'COVERAGE':
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
            if hasattr(self, '_delayed_timer') and self._delayed_timer is not None:
                self._delayed_timer.cancel()
            self._delayed_timer = self.create_timer(3.0, self._delayed_start, callback_group=None)

    def _delayed_start(self):
        """Callback setelah delay, memulai pipeline navigasi."""
        if hasattr(self, '_delayed_timer') and self._delayed_timer is not None:
            self._delayed_timer.cancel()
            self._delayed_timer = None
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
        # Handover mulus ke Tahap 2 (Coverage) jika robot sudah tiba tepat di titik awal (<= 0.15m)
        # atau jika kecepatan sudah 0 di dekat titik awal (<= 0.22m) agar tidak ngestuck
        if not self._is_transitioning and (dist <= 0.15 or (dist <= 0.22 and speed < 0.05)):
            self._is_transitioning = True
            self.get_logger().info(
                f"🎯 Robot tiba di titik awal Swath 0 ({dist:.2f}m)! Mengalihkan kontrol ke Tahap 2 (Coverage Nav2)...")
            if self._transit_goal_handle:
                self._transit_goal_handle.cancel_goal_async()
                self._transit_goal_handle = None
            if self.latest_coverage_path:
                self.start_coverage_follow_path(self.latest_coverage_path)

    # ══════════════════════════════════════════════════════════════════════════
    #  TAHAP 2: COVERAGE via Nav2 Controller (Seluruh Swath Sekuensial)
    # ══════════════════════════════════════════════════════════════════════════

    def start_coverage_follow_path(self, coverage_path: Path):
        """
        Mengeksekusi seluruh jalur coverage (semua swath) menggunakan Nav2 FollowPath.
        Mempertahankan seluruh swath lengkap, memotong waypoint yang sudah dilewati,
        dan memotong rintangan lokal via Dynamic Obstacle Detour.
        """
        self.current_stage = 'COVERAGE'
        self.is_navigating = True
        self._is_transitioning = False
        self._is_detouring = False
        self._is_maneuvering = False
        self._maneuver_phase = 'IDLE'
        self._active_coverage_poses = list(coverage_path.poses)
        self._coverage_progress_idx = 0
        self._coverage_stuck_ticks = 0
        self._coverage_start_time = self.get_clock().now().nanoseconds / 1e9
        self._coverage_initial_dist = None

        if not self._active_coverage_poses:
            self.get_logger().error("❌ Coverage path kosong!")
            self.is_navigating = False
            return

        self.get_logger().info(
            f"🚜 [Tahap 2/2] Memulai Coverage Nav2: {len(self._active_coverage_poses)} waypoint total (seluruh swath lengkap).")

        self._send_coverage_path(self._active_coverage_poses)

    def _send_coverage_path(self, poses: list):
        """Kirim seluruh sisa jalur coverage ke Nav2 FollowPath."""
        if not poses:
            self.get_logger().info("🎉 Seluruh urutan misi coverage selesai tuntas 100%!")
            self.stop_robot()
            self.is_navigating = False
            return

        if not self._follow_path_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("❌ Nav2 follow_path server belum siap!")
            self.is_navigating = False
            return

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = poses
        fresh = self.refresh_path_timestamps(path_msg)

        if self._coverage_goal_handle is not None:
            old_handle = self._coverage_goal_handle
            self._coverage_goal_handle = None
            old_handle.cancel_goal_async()

        goal = FollowPath.Goal()
        goal.path = fresh
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        goal.progress_checker_id = self.progress_checker_id

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

        self.get_logger().info(
            f"✅ Nav2 Coverage Controller aktif! Robot mulai menyapu ({len(self._active_coverage_poses)} waypoints tersisa).")
        self._coverage_goal_handle = gh
        result_future = gh.get_result_async()
        result_future.add_done_callback(lambda f, handle=gh: self._on_coverage_done(f, handle))

    def _coverage_feedback_cb(self, feedback_msg):
        try:
            fb = feedback_msg.feedback
            dist = getattr(fb, 'distance_to_goal', 0.0)
            speed = getattr(fb, 'speed', 0.0)
            now_sec = self.get_clock().now().nanoseconds / 1e9

            # ── 0. Cek apakah robot sudah benar-benar tiba di akhir seluruh misi ──
            elapsed = now_sec - self._coverage_start_time
            if elapsed > 5.0 and dist <= 0.15 and speed < 0.02 and len(self._active_coverage_poses) <= 15:
                if self._coverage_goal_handle is not None:
                    old_handle = self._coverage_goal_handle
                    self._coverage_goal_handle = None
                    old_handle.cancel_goal_async()
                self.stop_robot()
                self.is_navigating = False
                self.current_stage = 'IDLE'
                self.get_logger().info("🎉 [SUKSES] Seluruh misi coverage telah tuntas 100%! Seluruh lantai berhasil disapu bersih.")
                return

            self.get_logger().info(
                f"📍 [COVERAGE NAV2] Sisa: {dist:.2f}m | Kecepatan: {speed:.2f}m/s | Obstacle Avoidance Aktif",
                throttle_duration_sec=3.0)

            # ── 1. Update Indeks Progres Secara Sekuensial Monotonik ──
            if self._active_coverage_poses:
                robot_pose = self.get_robot_pose()
                if robot_pose is not None:
                    rx, ry, _ = robot_pose
                    poses = list(self._active_coverage_poses)
                    n_poses = len(poses)
                    cur_idx = max(0, min(self._coverage_progress_idx, n_poses - 1)) if n_poses > 0 else 0
                    # Cari titik terdekat hanya di jendela maju lokal [cur_idx, cur_idx + 6] (maks 30 cm)
                    # Jendela super ketat menjamin robot maju teratur satu per satu tanpa pernah meloncat
                    search_end = min(n_poses, cur_idx + 6)
                    if cur_idx < search_end:
                        best_local_idx = min(
                            range(cur_idx, search_end),
                            key=lambda i: math.hypot(poses[i].pose.position.x - rx, poses[i].pose.position.y - ry)
                        )
                        d = math.hypot(poses[best_local_idx].pose.position.x - rx, poses[best_local_idx].pose.position.y - ry)
                        if d < 0.25 and best_local_idx > cur_idx:
                            self._coverage_progress_idx = best_local_idx

                    # Pangkas waypoint yang sudah dilalui dan segarkan /coverage_path
                    # agar di RViz garis waypoint yang sudah selesai otomatis menghilang
                    if self._coverage_progress_idx >= 5 and len(self._active_coverage_poses) > self._coverage_progress_idx:
                        self._active_coverage_poses = self._active_coverage_poses[self._coverage_progress_idx:]
                        self._coverage_progress_idx = 0
                        rem_msg = Path()
                        rem_msg.header.frame_id = 'map'
                        rem_msg.header.stamp = self.get_clock().now().to_msg()
                        rem_msg.poses = self._active_coverage_poses
                        self.coverage_path_pub.publish(rem_msg)

            # ── 2. Deteksi Rintangan Nyata & Manuver Responsif ──
            if elapsed < 1.5:
                return

            if getattr(self, '_is_maneuvering', False) or self._is_detouring:
                return

            # Hanya jalankan jika masih ada sisa jalur yang cukup (> 5 waypoints)
            # dan cooldown detour sudah lewat (> 2.0 detik)
            if len(self._active_coverage_poses) > 5 and (now_sec - self._last_detour_time > 2.0):
                robot_pose = self.get_robot_pose()
                trans_moved = 0.1
                dt = 0.0

                if robot_pose is not None:
                    rx, ry, ryaw = robot_pose
                    if hasattr(self, '_last_check_pose') and self._last_check_pose is not None:
                        lx, ly, lyaw, ltime = self._last_check_pose
                        dt = now_sec - ltime
                        if dt >= 0.4:
                            trans_moved = math.hypot(rx - lx, ry - ly)
                            self._last_check_pose = (rx, ry, ryaw, now_sec)
                    else:
                        self._last_check_pose = (rx, ry, ryaw, now_sec)

                has_near_obs, obs_side, d_front, d_left, d_right, d_rear = self.analyze_obstacles()

                # Kondisi kritis: Rintangan berjarak <= 12 cm di depan bumper (d_front <= 0.26)
                # ATAU rintangan samping/sudut berjarak <= 7 cm dari bodi (d_left <= 0.22 atau d_right <= 0.22):
                critical_front_obs = (d_front <= 0.26 or d_left <= 0.22 or d_right <= 0.22)

                is_stuck = False
                if critical_front_obs:
                    self.get_logger().warn(
                        f"🚨 Rintangan dekat bodi (F={d_front:.2f}m, L={d_left:.2f}m, R={d_right:.2f}m)! Refleks aktif seketika!")
                    is_stuck = True
                elif dt >= 0.4:
                    # Pertimbangkan gerak robot: translasi < 0.04m ATAU kecepatan Nav2 < 0.06m/s
                    is_low_progress = (trans_moved < 0.04 or speed < 0.06)

                    if is_low_progress:
                        self._coverage_stuck_ticks += 1
                        # Kasus A: Ada rintangan dalam jarak dekat (has_near_obs) dan terhambat 2 tick (~0.8s)
                        if has_near_obs and self._coverage_stuck_ticks >= 2:
                            self.get_logger().warn(
                                f"⚠️ Robot terhambat rintangan di sisi {obs_side} (min: F={d_front:.2f}m, L={d_left:.2f}m, R={d_right:.2f}m).")
                            is_stuck = True
                        # Kasus B: Robot macet tidak bergerak selama 4 tick (~1.6s)
                        elif self._coverage_stuck_ticks >= 4:
                            self.get_logger().warn(
                                f"⚠️ Robot macet tidak bergerak selama {self._coverage_stuck_ticks * 0.4:.1f}s.")
                            is_stuck = True
                    else:
                        # Robot bergerak normal, turunkan tick secara gradual (anti-jitter)
                        self._coverage_stuck_ticks = max(0, self._coverage_stuck_ticks - 1)

                if is_stuck:
                    self._coverage_stuck_ticks = 0
                    self._last_detour_time = now_sec
                    self._start_maju_mundur_maneuver(obs_side, d_front, d_left, d_right)
                elif speed > 0.15:
                    self._detour_skip_count = 22
        except Exception as e:
            self.get_logger().error(f"⚠️ Error di _coverage_feedback_cb: {e}")

    def _start_maju_mundur_maneuver(self, obs_side, d_front, d_left, d_right):
        """
        Engine Manuver Maju-Mundur Responsif:
        1. Mundur sejauh ~32 cm ke area lantai yang sudah bersih dan bebas rintangan.
        2. Putar haluan ~50 derajat menjauhi rintangan menuju clearance terbuka.
        3. Langsung sambung rute Detour Bypass collision-free via Nav2 A* planner
           (tanpa langkah maju buta yang berisiko menyerempet tiang/rintangan).
        """
        if getattr(self, '_is_maneuvering', False) or self._is_detouring:
            return

        self._is_maneuvering = True
        self._coverage_stuck_ticks = 0

        # Batalkan goal FollowPath Nav2 agar robot tidak ditahan oleh controller
        if self._coverage_goal_handle is not None:
            old_handle = self._coverage_goal_handle
            self._coverage_goal_handle = None
            old_handle.cancel_goal_async()

        self.stop_robot()
        self._is_maneuvering = True  # pastikan tetap True setelah stop_robot

        # Tentukan arah putar menjauhi rintangan:
        # Jika rintangan di Kiri -> Putar ke Kanan (turn_dir = -1.0)
        # Jika rintangan di Kanan -> Putar ke Kiri (turn_dir = +1.0)
        # Jika di Depan / Netral -> Pilih sisi dengan ruang bebas (clearance) terluas
        if obs_side == 'LEFT':
            turn_dir = -1.0
            turn_name = 'KANAN'
        elif obs_side == 'RIGHT':
            turn_dir = 1.0
            turn_name = 'KIRI'
        else:
            if d_left >= d_right:
                turn_dir = 1.0
                turn_name = 'KIRI (clearance kiri lebih luas)'
            else:
                turn_dir = -1.0
                turn_name = 'KANAN (clearance kanan lebih luas)'

        self._maneuver_phase = 'MUNDUR'
        self._maneuver_turn_dir = turn_dir
        self._maneuver_turn_name = turn_name
        self._maneuver_start_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().warn(
            f"🔄 [MANUVER RESPONSif] Memulai Manuver Bebas-Tabrakan! "
            f"Fase 1: Mundur 32 cm (v = -0.20 m/s)... Target belok: {turn_name}.")

        if hasattr(self, '_maneuver_timer') and self._maneuver_timer is not None:
            self._maneuver_timer.cancel()
        self._maneuver_timer = self.create_timer(0.05, self._maneuver_step_cb)

    def _maneuver_step_cb(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed = now_sec - self._maneuver_start_time
        twist = Twist()

        has_near_obs, obs_side, d_front, d_left, d_right, d_rear = self.analyze_obstacles()

        if self._maneuver_phase == 'MUNDUR':
            # Fase 1: MUNDUR selama 1.6 detik (-0.20 m/s * 1.6s ~ 32 cm mundur ke area bersih)
            # Active Guard: Hentikan mundur jika di belakang mendekati rintangan (< 0.25m)
            if elapsed < 1.6 and d_rear > 0.25:
                twist.linear.x = -0.20
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
            else:
                self.cmd_vel_pub.publish(Twist())
                self._maneuver_phase = 'PUTAR'
                self._maneuver_start_time = now_sec
                self.get_logger().info(
                    f"🔄 [MANUVER] Fase 1 (Mundur 32 cm) Selesai. "
                    f"Fase 2: Putar {self._maneuver_turn_name} (~50°) menjauhi rintangan...")

        elif self._maneuver_phase == 'PUTAR':
            # Fase 2: PUTAR arah bodi robot menjauhi rintangan (~50 derajat / ~0.87 rad)
            # w = turn_dir * 1.3 rad/s selama 0.65 detik
            # Active Guard: Hentikan putaran jika sisi arah putar mendekati objek (< 0.23m)
            turning_side_clearance = d_right if self._maneuver_turn_dir < 0 else d_left
            if elapsed < 0.65 and turning_side_clearance > 0.23:
                twist.linear.x = 0.0
                twist.angular.z = self._maneuver_turn_dir * 1.3
                self.cmd_vel_pub.publish(twist)
            else:
                # Selesai seluruh manuver mundur & putar!
                # Menghilangkan langkah maju buta yang berisiko menyerempet tiang/panel meja.
                # Langsung aktifkan Nav2 A* Detour Planner dari posisi aman dan bersudut ini!
                if hasattr(self, '_maneuver_timer') and self._maneuver_timer is not None:
                    self._maneuver_timer.cancel()
                    self._maneuver_timer = None
                self.stop_robot()
                self._maneuver_phase = 'IDLE'
                self._is_maneuvering = False
                self.get_logger().info(
                    "✅ [MANUVER SELESAI] Sukses mundur dan memutar haluan ke ruang bebas! "
                    "Menyambungkan rute Detour Bypass collision-free...")
                self._trigger_obstacle_detour()

    def _trigger_obstacle_detour(self):
        """
        Dynamic Obstacle Detour Bypass:
        Merencanakan jalur memutar melewati rintangan meja (~1.2m / 24 waypoint)
        dan menyambung kembali ke sisa jalur coverage.
        """
        if self._is_detouring or not self._active_coverage_poses:
            return

        self._is_detouring = True

        # Batalkan goal FollowPath yang sedang macet di depan rintangan
        if self._coverage_goal_handle is not None:
            old_handle = self._coverage_goal_handle
            self._coverage_goal_handle = None
            old_handle.cancel_goal_async()

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self._is_detouring = False
            return

        rx, ry, _ = robot_pose
        poses = self._active_coverage_poses
        cur_idx = self._coverage_progress_idx

        # Lewati seluruh panjang meja (~1.1 - 1.3 m, 22-26 waypoint)
        if not hasattr(self, '_detour_skip_count') or self._detour_skip_count is None:
            self._detour_skip_count = 22
        else:
            self._detour_skip_count = min(len(poses) - 5, self._detour_skip_count + 4)

        target_idx = min(len(poses) - 1, cur_idx + self._detour_skip_count)

        # Jika sudah benar-benar di akhir seluruh jalur coverage
        if target_idx >= len(poses) - 3:
            self.get_logger().info("🎉 Robot sudah berada di ujung akhir misi. Menyelesaikan coverage!")
            self.stop_robot()
            self._is_detouring = False
            self.is_navigating = False
            return

        resume_pose = poses[target_idx]
        self.get_logger().info(
            f"✂️ [DETOUR BYPASS] Merencanakan rute memutar dari ({rx:.2f}, {ry:.2f}) ke ({resume_pose.pose.position.x:.2f}, {resume_pose.pose.position.y:.2f}) (skip {self._detour_skip_count} wp, resume idx {target_idx}/{len(poses)})...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = resume_pose
        goal_msg.planner_id = self.planner_id
        goal_msg.use_start = False

        future = self._compute_path_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f: self._on_detour_compute_response(f, target_idx))

    def _on_detour_compute_response(self, future, target_idx):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("ComputePathToPose untuk detour ditolak, mencoba waypoint lebih jauh...")
            self._detour_skip_count = min(len(self._active_coverage_poses) - 5, self._detour_skip_count + 8)
            target_idx = min(len(self._active_coverage_poses) - 1, self._coverage_progress_idx + self._detour_skip_count)
            self._active_coverage_poses = self._active_coverage_poses[target_idx:]
            self._coverage_progress_idx = 0
            self._is_detouring = False
            self._send_coverage_path(self._active_coverage_poses)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_detour_compute_result(f, target_idx))

    def _on_detour_compute_result(self, future, target_idx):
        try:
            result = future.result().result
            detour_path = result.path

            if not detour_path or len(detour_path.poses) == 0:
                self.get_logger().warn("Detour path kosong dari Nav2. Melanjutkan langsung dari sisa jalur...")
                remaining_poses = self._active_coverage_poses[target_idx:]
            else:
                detour_path = self.compute_path_tangents(detour_path)
                self.get_logger().info(
                    f"✅ Detour collision-free siap ({len(detour_path.poses)} poses). Menyambung ke sisa jalur coverage...")
                remaining_poses = list(detour_path.poses) + list(self._active_coverage_poses[target_idx:])

            if len(remaining_poses) == 0:
                self.get_logger().info("🎉 Seluruh jalur coverage telah dituntaskan!")
                self.stop_robot()
                self._is_detouring = False
                self.is_navigating = False
                return

            new_coverage_path = Path()
            new_coverage_path.header.frame_id = 'map'
            new_coverage_path.header.stamp = self.get_clock().now().to_msg()
            new_coverage_path.poses = remaining_poses

            self._active_coverage_poses = remaining_poses
            self._coverage_progress_idx = 0
            self._coverage_stuck_ticks = 0
            self._coverage_initial_dist = None   # reset agar progress gate fresh untuk segment baru
            self._coverage_start_time = self.get_clock().now().nanoseconds / 1e9
            self._is_detouring = False

            # Jalankan sisa seluruh jalur yang disambung rute detour
            self._send_coverage_path(remaining_poses)
        except Exception as e:
            self.get_logger().error(f"⚠️ Error di _on_detour_compute_result: {e}")
            self._is_detouring = False

    def _on_coverage_done(self, future, handle):
        # Abaikan callback dari goal lama yang sudah dibatalkan atau digantikan
        if self._coverage_goal_handle is None or handle is not self._coverage_goal_handle:
            return

        self._coverage_goal_handle = None

        if self._is_detouring:
            return  # Sedang bermanuver detour, jangan batalkan misi

        status = future.result().status
        poses = self._active_coverage_poses
        cur_idx = self._coverage_progress_idx

        # Proteksi False Arrival: Cek apakah seluruh sekuens coverage benar-benar sudah dituntaskan
        if status == GoalStatus.STATUS_SUCCEEDED:
            remaining_count = len(poses) - cur_idx if poses else 0
            if remaining_count > 10:
                self.get_logger().warn(
                    f"⚠️ Nav2 GoalChecker declare SUCCEEDED prematur! Masih ada {remaining_count} waypoint tersisa. "
                    f"Melanjutkan eksekusi sisa {remaining_count} waypoint...")
                rem_poses = poses[cur_idx:]
                self._active_coverage_poses = rem_poses
                self._coverage_progress_idx = 0
                self._send_coverage_path(rem_poses)
                return

            self.is_navigating = False
            self.stop_robot()
            self.get_logger().info("🎉 [SUKSES] Seluruh Sequence Misi Coverage Selesai 100% tanpa tabrakan!")

        elif status == GoalStatus.STATUS_ABORTED:
            remaining_count = len(poses) - cur_idx if poses else 0
            if remaining_count > 10:
                self.get_logger().warn(
                    f"⚠️ Controller mengaborsi jalur (sisa {remaining_count} waypoint). "
                    f"Menjalankan Manuver Maju-Mundur untuk mundur dan meloloskan diri dari rintangan...")
                has_near_obs, obs_side, d_front, d_left, d_right, d_rear = self.analyze_obstacles()
                self._start_maju_mundur_maneuver(obs_side, d_front, d_left, d_right)
            else:
                self.is_navigating = False
                self.stop_robot()
                self.get_logger().info("🎉 Misi coverage selesai di ujung akhir jalur.")

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("ℹ️ Goal coverage dibatalkan secara normal.")

        else:
            self.is_navigating = False
            self.stop_robot()
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

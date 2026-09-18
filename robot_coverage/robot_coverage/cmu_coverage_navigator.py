#!/usr/bin/env python3
"""
CMU Coverage Navigator Node
---------------------------
Menghubungkan coverage path dari Fields2Cover (/coverage_path) ke CMU Local Planner (/way_point).
1. Menerima nav_msgs/Path dari coverage planner.
2. Secara sekuensial mengirimkan setiap waypoint ke CMU /way_point (geometry_msgs/PointStamped).
3. Memantau posisi robot via /state_estimation dan memajukan waypoint saat robot sudah dekat (< waypoint_tolerance).
4. Menyediakan feedback progres eksekusi coverage.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Float32, Int8


class CmuCoverageNavigator(Node):
    def __init__(self):
        super().__init__('cmu_coverage_navigator')

        self.declare_parameter('waypoint_tolerance', 0.35)
        self.declare_parameter('auto_navigate', True)
        self.declare_parameter('world_frame', 'map')

        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.auto_navigate = self.get_parameter('auto_navigate').get_parameter_value().bool_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value

        self.coverage_poses = []
        self.current_wp_idx = 0
        self.is_navigating = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.has_odom = False

        # QoS
        latch_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers to CMU stack
        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', qos_reliable)
        self.stop_pub = self.create_publisher(Int8, '/stop', qos_reliable)

        # Subscriptions
        self.path_sub = self.create_subscription(
            Path, '/coverage_path', self.coverage_path_callback, latch_qos)
        self.state_sub = self.create_subscription(
            Odometry, '/state_estimation', self.state_callback, qos_reliable)

        # Periodic navigation loop (10 Hz)
        self.timer = self.create_timer(0.1, self.navigation_loop)

        self.get_logger().info(
            f"🚜 CmuCoverageNavigator siap. Menunggu /coverage_path... (tolerance: {self.waypoint_tolerance}m)")

    def coverage_path_callback(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("⚠️ /coverage_path kosong!")
            return

        self.coverage_poses = list(msg.poses)
        self.current_wp_idx = 0
        total = len(self.coverage_poses)
        self.get_logger().info(f"📥 Menerima /coverage_path baru dengan {total} titik waypoint dari Fields2Cover.")

        if self.auto_navigate:
            self.start_navigation()

    def start_navigation(self):
        if not self.coverage_poses:
            return
        self.is_navigating = True
        self.current_wp_idx = 0
        self._publish_current_waypoint()
        self.get_logger().info(f"🚀 Memulai navigasi coverage dengan CMU stack ({len(self.coverage_poses)} waypoints)...")

    def _publish_current_waypoint(self):
        if self.current_wp_idx >= len(self.coverage_poses):
            return

        pose = self.coverage_poses[self.current_wp_idx]
        pt_msg = PointStamped()
        pt_msg.header.stamp = self.get_clock().now().to_msg()
        pt_msg.header.frame_id = self.world_frame
        pt_msg.point.x = pose.pose.position.x
        pt_msg.point.y = pose.pose.position.y
        pt_msg.point.z = pose.pose.position.z

        self.waypoint_pub.publish(pt_msg)

    def state_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z
        self.has_odom = True

    def navigation_loop(self):
        if not self.is_navigating or not self.has_odom or not self.coverage_poses:
            return

        total = len(self.coverage_poses)
        if self.current_wp_idx >= total:
            # Selesai!
            self.is_navigating = False
            self.get_logger().info(f"🏁 ✅ Misi Coverage selesai! Seluruh {total} waypoint berhasil dikunjungi.")
            stop_msg = Int8()
            stop_msg.data = 1
            self.stop_pub.publish(stop_msg)
            return

        target_pose = self.coverage_poses[self.current_wp_idx]
        tx = target_pose.pose.position.x
        ty = target_pose.pose.position.y

        dist = math.hypot(self.robot_x - tx, self.robot_y - ty)

        if dist <= self.waypoint_tolerance:
            self.current_wp_idx += 1
            progress = (self.current_wp_idx / total) * 100.0

            if self.current_wp_idx < total:
                self._publish_current_waypoint()
                next_pose = self.coverage_poses[self.current_wp_idx]
                if self.current_wp_idx % 5 == 0 or self.current_wp_idx == total - 1:
                    self.get_logger().info(
                        f"📍 [CMU Coverage] Progres: {self.current_wp_idx}/{total} ({progress:.1f}%) | "
                        f"Target: ({next_pose.pose.position.x:.2f}, {next_pose.pose.position.y:.2f})")
            else:
                self.is_navigating = False
                self.get_logger().info(f"🏁 ✅ Misi Coverage selesai! Seluruh {total} waypoint berhasil dikunjungi.")
                stop_msg = Int8()
                stop_msg.data = 1
                self.stop_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmuCoverageNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

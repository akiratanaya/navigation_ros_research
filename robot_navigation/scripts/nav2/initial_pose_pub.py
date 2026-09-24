#!/usr/bin/env python3
"""
initial_pose_pub.py — Otomatis mengirim Initial Pose ke AMCL saat simulasi dimulai.
Menggunakan sinkronisasi timestamp dari buffer TF odom -> base_footprint
untuk menghindari error ekstrapolasi waktu pada Nav2 AMCL.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('x', -2.0)
        self.declare_parameter('y', 1.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('repeat_count', 5)

        self.x = self.get_parameter('x').get_parameter_value().double_value
        self.y = self.get_parameter('y').get_parameter_value().double_value
        self.yaw = self.get_parameter('yaw').get_parameter_value().double_value
        self.max_repeats = self.get_parameter('repeat_count').get_parameter_value().integer_value
        self.count = 0

        # TF Buffer & Listener untuk mendapatkan stempel waktu aktual dari odom
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)
        # Delay timer 1 detik sebelum mulai mengirim agar TF odom -> base_footprint sudah terisi
        self.timer = self.create_timer(1.0, self.publish_initial_pose)

        self.get_logger().info(
            f"📍 InitialPosePublisher aktif. Menunggu TF odom lalu mengirim ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f}) ke AMCL..."
        )

    def publish_initial_pose(self):
        # Ambil stempel waktu dari data TF terbaru di buffer agar AMCL tidak gagal ekstrapolasi
        try:
            t = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            stamp = t.header.stamp
        except Exception as e:
            self.get_logger().info(f"⏳ Menunggu TF odom -> base_footprint tersedia: {e}")
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = stamp

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        # Matriks kovariansi awal (ketidakpastian sangat kecil agar partikel AMCL langsung mengunci)
        cov = [0.0] * 36
        cov[0] = 0.02   # Var(x)
        cov[7] = 0.02   # Var(y)
        cov[35] = 0.02  # Var(yaw)
        msg.pose.covariance = cov

        self.publisher.publish(msg)
        self.count += 1

        self.get_logger().info(
            f"✅ Initial Pose terkirim presisi ke /initialpose pada waktu {stamp.sec}.{stamp.nanosec} [{self.count}/{self.max_repeats}]"
        )

        if self.count >= self.max_repeats:
            self.timer.cancel()
            self.get_logger().info("🎯 Inisialisasi posisi AMCL sukses dan selesai.")


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

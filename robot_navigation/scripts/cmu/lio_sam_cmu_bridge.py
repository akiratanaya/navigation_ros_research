#!/usr/bin/env python3
"""
LIO-SAM to CMU Autonomy Bridge Node
-----------------------------------
Menjembatani LIO-SAM 3D SLAM dengan CMU Autonomy Stack (Terrain Analysis & TARE Planner):
1. Mengubah /odometry/imu dari LIO-SAM menjadi /state_estimation (frame: map, child: vehicle).
2. Mentransformasikan /points (LiDAR 3D) menjadi /registered_scan (frame: map) menggunakan TF LIO-SAM.
3. Memfilter self-reflection robot (r <= 0.28m) tanpa memotong batas taman terbuka (outdoor park).
4. Mempublikasikan polygon batas taman (/navigation_boundary & /sensor_coverage_planner/coverage_boundary).
5. Mengirim sinyal otomatis /start_exploration (true) setelah sistem stabil.
6. Mempublikasikan static TF: base_footprint -> vehicle.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import TransformStamped, PointStamped, PoseStamped, PolygonStamped, Point32
import tf2_ros
from tf2_ros import Buffer, TransformListener
from sensor_msgs_py import point_cloud2 as pc2


class LioSamCmuBridge(Node):
    def __init__(self):
        super().__init__('lio_sam_cmu_bridge')

        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('lidar_frame', 'lidar_3d_link')
        self.declare_parameter('robot_radius', 0.28)
        self.declare_parameter('boundary_size', 42.0)  # +/- 42 meter untuk outdoor park 84x84m
        self.declare_parameter('enable_clicked_point', True)

        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self.lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.boundary_size = self.get_parameter('boundary_size').get_parameter_value().double_value
        self.enable_clicked_point = self.get_parameter('enable_clicked_point').get_parameter_value().bool_value

        # TF Buffer & Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS Profiles
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers untuk CMU Stack
        self.state_est_pub = self.create_publisher(Odometry, '/state_estimation', qos_reliable)
        self.reg_scan_pub = self.create_publisher(PointCloud2, '/registered_scan', qos_reliable)
        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', qos_reliable)
        self.nav_boundary_pub = self.create_publisher(PolygonStamped, '/navigation_boundary', qos_reliable)
        self.cov_boundary_pub = self.create_publisher(PolygonStamped, '/sensor_coverage_planner/coverage_boundary', qos_reliable)
        self.start_expl_pub = self.create_publisher(Bool, '/start_exploration', qos_reliable)

        # Timer untuk publikasi batas area eksplorasi dan static TF (1 Hz)
        self.boundary_timer = self.create_timer(1.0, self.publish_boundaries)

        # Timer untuk memicu start_exploration otomatis (mulai setelah 6 detik)
        self.start_timer = self.create_timer(2.0, self.trigger_auto_start)
        self.start_count = 0
        self.start_time_init = self.get_clock().now()

        # Subscriptions dari LIO-SAM dan Sensor
        # 1. Odometri presisi tinggi dari IMU Preintegration LIO-SAM (dipublikasikan BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/imu', self.odom_callback, qos_sensor)

        # 2. Raw 3D point cloud dari Velodyne LiDAR
        self.points_sub = self.create_subscription(
            PointCloud2, '/points', self.points_callback, qos_sensor)

        # 3. RViz Waypoint manual (opsional jika pengguna ingin klik tujuan manual)
        self.clicked_point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_callback, qos_reliable)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, qos_reliable)

        # PointField layout untuk PointXYZI
        self.point_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Publish static transform: base_footprint -> vehicle
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_transforms()

        self.get_logger().info(
            f"✅ LioSamCmuBridge aktif: /odometry/imu -> /state_estimation, /points -> /registered_scan (frame: {self.world_frame})")

    def _publish_static_transforms(self):
        transforms = []
        now = self.get_clock().now().to_msg()

        # base_footprint -> vehicle (identity)
        t_base_vehicle = TransformStamped()
        t_base_vehicle.header.stamp = now
        t_base_vehicle.header.frame_id = self.robot_base_frame
        t_base_vehicle.child_frame_id = 'vehicle'
        t_base_vehicle.transform.rotation.w = 1.0
        transforms.append(t_base_vehicle)

        self.static_broadcaster.sendTransform(transforms)

    def trigger_auto_start(self):
        elapsed = (self.get_clock().now() - self.start_time_init).nanoseconds / 1e9
        if elapsed < 6.0:
            return

        if self.start_count < 8:
            msg = Bool()
            msg.data = True
            self.start_expl_pub.publish(msg)
            if self.start_count == 0:
                self.get_logger().info("🚀 Auto-Exploration TARE Planner DIPICU: /start_exploration -> True")
            self.start_count += 1

    def odom_callback(self, msg: Odometry):
        # CMU nodes memerlukan state_estimation dalam frame 'map' dengan child 'vehicle'
        state_msg = Odometry()
        state_msg.header = msg.header
        state_msg.header.frame_id = self.world_frame
        state_msg.child_frame_id = 'vehicle'
        state_msg.pose = msg.pose
        state_msg.twist = msg.twist
        self.state_est_pub.publish(state_msg)

    def clicked_point_callback(self, msg: PointStamped):
        if not self.enable_clicked_point:
            return
        pt = PointStamped()
        pt.header = msg.header
        pt.header.frame_id = self.world_frame
        pt.point.x = msg.point.x
        pt.point.y = msg.point.y
        pt.point.z = msg.point.z
        self.get_logger().info(f"🎯 Waypoint manual dari /clicked_point: ({pt.point.x:.2f}, {pt.point.y:.2f}, {pt.point.z:.2f})")
        self.waypoint_pub.publish(pt)

    def goal_pose_callback(self, msg: PoseStamped):
        pt = PointStamped()
        pt.header = msg.header
        pt.header.frame_id = self.world_frame
        pt.point.x = msg.pose.position.x
        pt.point.y = msg.pose.position.y
        pt.point.z = msg.pose.position.z
        self.get_logger().info(f"🎯 Waypoint manual dari /goal_pose: ({pt.point.x:.2f}, {pt.point.y:.2f}, {pt.point.z:.2f})")
        self.waypoint_pub.publish(pt)

    def publish_boundaries(self):
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        b = self.boundary_size
        coords = [
            (-b, -b),
            (b, -b),
            (b, b),
            (-b, b),
            (-b, -b)
        ]
        for x, y in coords:
            pt = Point32()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0
            msg.polygon.points.append(pt)
        self.nav_boundary_pub.publish(msg)
        self.cov_boundary_pub.publish(msg)
        self._publish_static_transforms()

    def points_callback(self, msg: PointCloud2):
        # Lookup transform dari map ke frame LiDAR robot (lidar_3d_link)
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.lidar_frame,
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed for {self.lidar_frame}: {e}", throttle_duration_sec=2.0)
            return

        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation

        # Quaternion ke rotation matrix 3x3
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        r_mat = np.array([
            [1.0 - 2.0 * (qy*qy + qz*qz), 2.0 * (qx*qy - qz*qw),       2.0 * (qx*qz + qy*qw)],
            [2.0 * (qx*qy + qz*qw),       1.0 - 2.0 * (qx*qx + qz*qz), 2.0 * (qy*qz - qx*qw)],
            [2.0 * (qx*qz - qy*qw),       2.0 * (qy*qz + qx*qw),       1.0 - 2.0 * (qx*qx + qy*qy)]
        ], dtype=np.float32)

        trans_vec = np.array([t.x, t.y, t.z], dtype=np.float32)

        # Baca titik XYZ
        try:
            pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        except Exception as e:
            self.get_logger().warn(f"read_points failed: {e}", throttle_duration_sec=2.0)
            return

        if len(pts) == 0:
            return

        # Filter NaN dan Inf
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) == 0:
            return

        # Filter pantulan ke bodi robot sendiri (r_xy <= robot_radius)
        r_xy = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[r_xy > self.robot_radius]
        if len(pts) == 0:
            return

        # Transformasikan ke world frame (map): P_world = (P @ R^T) + trans
        pts_world = (pts @ r_mat.T) + trans_vec

        # Tambahkan kolom intensity (0.0)
        cloud_data = np.hstack([pts_world, np.zeros((len(pts_world), 1), dtype=np.float32)])

        out_header = Header()
        out_header.stamp = msg.header.stamp
        out_header.frame_id = self.world_frame

        reg_cloud = pc2.create_cloud(out_header, self.point_fields, cloud_data)
        self.reg_scan_pub.publish(reg_cloud)


def main(args=None):
    rclpy.init(args=args)
    node = LioSamCmuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

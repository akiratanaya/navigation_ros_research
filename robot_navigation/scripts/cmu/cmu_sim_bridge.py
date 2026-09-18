#!/usr/bin/env python3
"""
CMU Simulation Bridge Node
--------------------------
Menghubungkan robot simulasi Gazebo (differential drive) dengan CMU Autonomy Stack:
1. Menerjemahkan /odom -> /state_estimation (frame_id: 'map', child_frame_id: 'vehicle').
2. Mentransformasikan pointcloud 3D /points (lidar_link) menjadi /registered_scan (frame_id: 'map').
3. Memfilter nilai NaN / Inf agar PCL KD-Tree C++ tidak crash.
4. Mempublikasikan static TF yang dibutuhkan CMU stack:
   - map -> odom
   - base_footprint -> vehicle
   - vehicle -> sensor
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, PointStamped, PoseStamped
import tf2_ros
from tf2_ros import Buffer, TransformListener
from sensor_msgs_py import point_cloud2 as pc2


class CmuSimBridge(Node):
    def __init__(self):
        super().__init__('cmu_sim_bridge')

        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('pub_static_tf', True)
        self.declare_parameter('robot_radius', 0.28)

        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self.lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value
        self.pub_static_tf = self.get_parameter('pub_static_tf').get_parameter_value().bool_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value

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

        # Publishers for CMU Autonomy Stack
        self.state_est_pub = self.create_publisher(Odometry, '/state_estimation', qos_reliable)
        self.reg_scan_pub = self.create_publisher(PointCloud2, '/registered_scan', qos_reliable)
        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', qos_reliable)

        # Subscriptions from Gazebo
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_reliable)
        self.points_sub = self.create_subscription(
            PointCloud2, '/points', self.points_callback, qos_sensor)

        # RViz Waypoint Adapter Subscriptions (2D Goal Pose & Publish Point)
        self.clicked_point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_callback, qos_reliable)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, qos_reliable)

        # PointField layout for PointXYZI
        self.point_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Publish static transforms if enabled
        if self.pub_static_tf:
            self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
            self._publish_static_transforms()

        self.get_logger().info(
            f"✅ CmuSimBridge aktif: /odom -> /state_estimation, /points -> /registered_scan (frame: {self.world_frame})")

    def _publish_static_transforms(self):
        transforms = []
        now = self.get_clock().now().to_msg()

        # 1. map -> odom (identity)
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = self.world_frame
        t_map_odom.child_frame_id = self.odom_frame
        t_map_odom.transform.rotation.w = 1.0
        transforms.append(t_map_odom)

        # 2. base_footprint -> vehicle (identity)
        t_base_vehicle = TransformStamped()
        t_base_vehicle.header.stamp = now
        t_base_vehicle.header.frame_id = self.robot_base_frame
        t_base_vehicle.child_frame_id = 'vehicle'
        t_base_vehicle.transform.rotation.w = 1.0
        transforms.append(t_base_vehicle)

        # 3. vehicle -> sensor
        t_vehicle_sensor = TransformStamped()
        t_vehicle_sensor.header.stamp = now
        t_vehicle_sensor.header.frame_id = 'vehicle'
        t_vehicle_sensor.child_frame_id = 'sensor'
        t_vehicle_sensor.transform.translation.z = 0.165
        t_vehicle_sensor.transform.rotation.w = 1.0
        transforms.append(t_vehicle_sensor)

        self.static_broadcaster.sendTransform(transforms)

    def odom_callback(self, msg: Odometry):
        # CMU nodes expect state_estimation in map frame with child vehicle
        state_msg = Odometry()
        state_msg.header = msg.header
        state_msg.header.frame_id = self.world_frame
        state_msg.child_frame_id = 'vehicle'
        state_msg.pose = msg.pose
        state_msg.twist = msg.twist
        self.state_est_pub.publish(state_msg)

    def clicked_point_callback(self, msg: PointStamped):
        pt = PointStamped()
        pt.header = msg.header
        pt.header.frame_id = self.world_frame
        pt.point.x = msg.point.x
        pt.point.y = msg.point.y
        pt.point.z = 0.0  # Lock to ground level
        self.get_logger().info(f"🎯 Waypoint dari /clicked_point diterima: ({pt.point.x:.2f}, {pt.point.y:.2f}, z=0.0)")
        self.waypoint_pub.publish(pt)

    def goal_pose_callback(self, msg: PoseStamped):
        pt = PointStamped()
        pt.header = msg.header
        pt.header.frame_id = self.world_frame
        pt.point.x = msg.pose.position.x
        pt.point.y = msg.pose.position.y
        pt.point.z = 0.0  # Lock to ground level
        self.get_logger().info(f"🎯 Waypoint dari /goal_pose diterima: ({pt.point.x:.2f}, {pt.point.y:.2f}, z=0.0)")
        self.waypoint_pub.publish(pt)

    def points_callback(self, msg: PointCloud2):
        # Lookup transform from world_frame to lidar frame
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.world_frame,
                msg.header.frame_id,
                rclpy.time.Time())
        except Exception:
            return

        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation

        # Convert quaternion to 3x3 rotation matrix
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        r_mat = np.array([
            [1.0 - 2.0 * (qy*qy + qz*qz), 2.0 * (qx*qy - qz*qw),       2.0 * (qx*qz + qy*qw)],
            [2.0 * (qx*qy + qz*qw),       1.0 - 2.0 * (qx*qx + qz*qz), 2.0 * (qy*qz - qx*qw)],
            [2.0 * (qx*qz - qy*qw),       2.0 * (qy*qz + qx*qw),       1.0 - 2.0 * (qx*qx + qy*qy)]
        ], dtype=np.float32)

        trans_vec = np.array([t.x, t.y, t.z], dtype=np.float32)

        # Read point cloud points
        try:
            pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        except Exception:
            return

        if len(pts) == 0:
            return

        # Filter out NaN and Inf points from sensor rays into open space
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) == 0:
            return

        # Filter self-reflection points hitting robot body/wheels (r_xy <= robot_radius)
        r_xy = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[r_xy > self.robot_radius]
        if len(pts) == 0:
            return

        # Vectorized transform to world frame: P_world = (P @ R^T) + trans
        pts_world = (pts @ r_mat.T) + trans_vec

        # Add intensity column (0.0)
        cloud_data = np.hstack([pts_world, np.zeros((len(pts_world), 1), dtype=np.float32)])

        out_header = Header()
        out_header.stamp = msg.header.stamp
        out_header.frame_id = self.world_frame

        reg_cloud = pc2.create_cloud(out_header, self.point_fields, cloud_data)
        self.reg_scan_pub.publish(reg_cloud)


def main(args=None):
    rclpy.init(args=args)
    node = CmuSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

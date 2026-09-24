#!/usr/bin/env python3
"""
send_custom_polygon.py — Kirim Batas Poligon Buatan Sendiri ke Fields2Cover
=============================================================================
Memudahkan pengiriman poligon lahan dengan bentuk bebas:
1. Dari file YAML (misal: polygon_l_shape.yaml)
2. Dari preset bentuk bawaan (hexagon, l_shape, triangle, trapezoid, rectangle)
3. Dari string koordinat langsung di terminal: "x1,y1; x2,y2; x3,y3; ..."

Cara Pakai:
  # Contoh 1: Pakai preset bentuk
  ros2 run robot_coverage send_custom_polygon --preset l_shape
  ros2 run robot_coverage send_custom_polygon --preset hexagon

  # Contoh 2: Pakai file YAML buatan sendiri
  ros2 run robot_coverage send_custom_polygon --file ~/my_field.yaml

  # Contoh 3: Masukkan koordinat manual di terminal
  ros2 run robot_coverage send_custom_polygon --coords "-4,-3; 4,-3; 5,2; 0,4; -4,2"
"""

import os
import sys
import argparse
import yaml
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PolygonStamped, Point32, Point
from visualization_msgs.msg import Marker


# ── Preset Bentuk Poligon Unik di Outdoor Park ────────────────────────────────
PRESETS = {
    # 1. Bentuk L (Melengkung mengitari kontur bukit tengah)
    'l_shape': [
        (-6.0, -4.0),
        (4.0, -4.0),
        (4.0, 0.0),
        (-1.0, 0.0),
        (-1.0, 4.0),
        (-6.0, 4.0),
    ],
    # 2. Segi Enam (Hexagon simetris di tengah meadow)
    'hexagon': [
        (-4.0, 0.0),
        (-2.0, 3.5),
        (2.0, 3.5),
        (4.0, 0.0),
        (2.0, -3.5),
        (-2.0, -3.5),
    ],
    # 3. Trapesium / Segi Empat Asimetris
    'trapezoid': [
        (-5.0, -4.0),
        (5.0, -3.0),
        (3.0, 4.0),
        (-4.0, 3.5),
    ],
    # 4. Segitiga Besar
    'triangle': [
        (-5.0, -4.0),
        (5.0, -4.0),
        (0.0, 5.0),
    ],
    # 5. Persegi Panjang Padang Rumput (Default Meadow)
    'rectangle': [
        (-6.0, -4.5),
        (6.0, -4.5),
        (6.0, 4.5),
        (-6.0, 4.5),
    ],
}


class PolygonSender(Node):
    def __init__(self, points, frame_id='map'):
        super().__init__('custom_polygon_sender')
        self.points = points
        self.frame_id = frame_id

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.poly_pub = self.create_publisher(PolygonStamped, '/field_boundary', latched_qos)
        self.preview_pub = self.create_publisher(Marker, '/field_boundary_preview', latched_qos)

    def publish_polygon(self):
        now = self.get_clock().now().to_msg()

        # 1. PolygonStamped untuk Fields2Cover
        poly_msg = PolygonStamped()
        poly_msg.header.stamp = now
        poly_msg.header.frame_id = self.frame_id

        for x, y in self.points:
            pt = Point32()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0
            poly_msg.polygon.points.append(pt)

        # 2. Marker LINE_STRIP untuk visualisasi di RViz
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = self.frame_id
        marker.ns = 'custom_field_boundary'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.15
        marker.color.r = 1.0
        marker.color.g = 0.55
        marker.color.b = 0.0
        marker.color.a = 0.95
        marker.pose.orientation.w = 1.0

        for x, y in self.points:
            marker.points.append(Point(x=float(x), y=float(y), z=0.05))
        # Tutup garis loop kembali ke titik awal
        if len(self.points) > 0:
            marker.points.append(Point(x=float(self.points[0][0]), y=float(self.points[0][1]), z=0.05))

        self.poly_pub.publish(poly_msg)
        self.preview_pub.publish(marker)

        # Hitung luas area sederhana (Shoelace formula)
        area = 0.0
        n = len(self.points)
        for i in range(n):
            j = (i + 1) % n
            area += self.points[i][0] * self.points[j][1]
            area -= self.points[j][0] * self.points[i][1]
        area = abs(area) * 0.5

        self.get_logger().info(f"✅ Poligon kustom berhasil dipublikasikan ke /field_boundary!")
        self.get_logger().info(f"   ├─ Jumlah titik sudut: {len(self.points)}")
        self.get_logger().info(f"   ├─ Luas area terlingkupi: ≈ {area:.2f} m²")
        self.get_logger().info(f"   └─ Frame ID: '{self.frame_id}'")
        for i, (x, y) in enumerate(self.points):
            self.get_logger().info(f"      Pt #{i+1}: ({x:+.2f}, {y:+.2f})")


def parse_coords_string(s: str):
    """Parse format: 'x1,y1; x2,y2; x3,y3' atau 'x1,y1 x2,y2'."""
    pts = []
    # ganti koma spasi atau titik-koma
    tokens = [t.strip() for t in s.replace(';', ' ').split() if t.strip()]
    for token in tokens:
        parts = token.split(',')
        if len(parts) >= 2:
            pts.append((float(parts[0]), float(parts[1])))
    return pts


def parse_yaml_file(filepath: str):
    """Baca list titik dari file YAML."""
    if not os.path.exists(filepath):
        print(f"Error: File '{filepath}' tidak ditemukan.")
        sys.exit(1)
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)
    
    # Format bisa:
    # 1. points: [[x1, y1], [x2, y2], ...]
    # 2. [[x1, y1], [x2, y2], ...]
    if isinstance(data, dict):
        raw_pts = data.get('points', data.get('polygon', []))
    elif isinstance(data, list):
        raw_pts = data
    else:
        raw_pts = []

    pts = []
    for item in raw_pts:
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            pts.append((float(item[0]), float(item[1])))
        elif isinstance(item, dict):
            pts.append((float(item['x']), float(item['y'])))
    return pts


def main():
    parser = argparse.ArgumentParser(description="Kirim Poligon Lahan Kustom ke Fields2Cover")
    parser.add_argument('--file', '-f', type=str, help="Path ke file YAML berisi koordinat titik")
    parser.add_argument('--preset', '-p', type=str, choices=list(PRESETS.keys()), help="Preset bentuk (hexagon, l_shape, trapezoid, triangle, rectangle)")
    parser.add_argument('--coords', '-c', type=str, help="Koordinat titik manual: 'x1,y1; x2,y2; x3,y3'")
    parser.add_argument('--frame', default='map', help="Frame ID koordinat (default: map)")
    
    # Filter argumen ROS 2 (--ros-args dll)
    args, unknown = parser.parse_known_args()

    points = []
    if args.preset:
        points = PRESETS[args.preset]
        print(f"Menggunakan preset '{args.preset}' ({len(points)} titik).")
    elif args.file:
        points = parse_yaml_file(args.file)
        print(f"Membaca {len(points)} titik dari file: {args.file}")
    elif args.coords:
        points = parse_coords_string(args.coords)
        print(f"Membaca {len(points)} titik dari argumen coords.")
    else:
        # Default fallback: Hexagon
        print("Tidak ada argumen diberikan, menggunakan preset bawaan 'hexagon'.")
        print("Tip: Gunakan --preset <nama>, --file <path.yaml>, atau --coords 'x1,y1; x2,y2; ...'")
        points = PRESETS['hexagon']

    if len(points) < 3:
        print("Error: Poligon membutuhkan minimal 3 titik koordinat!")
        sys.exit(1)

    rclpy.init()
    node = PolygonSender(points, frame_id=args.frame)
    node.publish_polygon()
    
    # Spin sebentar agar subscriber TRANSIENT_LOCAL menerima pesan
    rclpy.spin_once(node, timeout_sec=1.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

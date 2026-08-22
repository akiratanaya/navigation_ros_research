#!/usr/bin/env python3
import rclpy
# SESUAIKAN MENJADI INI
from .coverage_server import CoverageServer

def main(args=None):
    rclpy.init(args=args)
    node = CoverageServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Coverage Server...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
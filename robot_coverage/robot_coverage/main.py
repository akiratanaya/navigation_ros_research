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
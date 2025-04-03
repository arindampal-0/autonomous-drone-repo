#!/usr/bin/env python3
"""mavlink_connection file"""

import sys

import rclpy
from rclpy.node import Node

class MavlinkConnection(Node):
    """MavlinkConnection Node"""
    def __init__(self):
        super().__init__("mavlink_connection")
        self.get_logger().info("Mavlink Connection Node")


def main(args=None):
    """main function"""
    rclpy.init(args=args)

    mavlink_connection_node = MavlinkConnection()
    rclpy.spin(mavlink_connection_node)

    mavlink_connection_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)

#!/usr/bin/env python3
"""mavlink_connection file"""

import sys

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Path

from pymavlink import mavutil
from serial.serialutil import SerialException


class MavlinkConnection(Node):
    """MavlinkConnection Node"""

    def __init__(self):
        super().__init__("mavlink_connection")

        self.declare_parameter(
            "device_connection_string",
            "/dev/ttyACM0",
            ParameterDescriptor(description="Device connection COM port."),
        )

        device_connection_string = (
            self.get_parameter("device_connection_string")
            .get_parameter_value()
            .string_value
        )

        # /zed/zed_node/path_map gives array of geometry_msgs/PoseStamped
        # /zed/zed_node/pose gives geometry_msgs/PoseStamped
        self.declare_parameter(
            "pose_topic",
            "/zed/zed_node/path_map",
            ParameterDescriptor(
                description="Topic which gives Poses or array of Poses"
            ),
        )
        pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Path, pose_topic, self.subscription_callback, 10
        )

        self.get_logger().info("Mavlink Connection Node")
        self.get_logger().info(f"device connection string: {device_connection_string}")

        try:
            self.master = mavutil.mavlink_connection(device_connection_string)
            self.master.wait_heartbeat()
            self.get_logger().info("Connected with Pixhawk.")
        except SerialException as ex:
            self.get_logger().error(f"No such device {device_connection_string}")
            raise SystemExit from ex

    def subscription_callback(self, path: Path):
        """subscription callback"""
        self.get_logger().info(f"timestamp: {path.header.stamp}")


def main(args=None):
    """main function"""
    rclpy.init(args=args)

    mavlink_connection_node = MavlinkConnection()
    rclpy.spin(mavlink_connection_node)

    mavlink_connection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)

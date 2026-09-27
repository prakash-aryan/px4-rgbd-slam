#!/usr/bin/env python3
"""Rewrite the gz-bridged image frame_ids to a consistent TF chain for RTAB-Map."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

OPTICAL = "camera_optical_link"


class FrameRewriter(Node):
    def __init__(self):
        super().__init__("frame_rewriter")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._rgb_pub = self.create_publisher(Image, "/drone_1/camera/image", 5)
        self._depth_pub = self.create_publisher(Image, "/drone_1/depth/image", 5)
        self._info_pub = self.create_publisher(CameraInfo, "/drone_1/camera/camera_info", 5)
        self.create_subscription(Image, "/drone_1/camera/raw", self._on_rgb, qos)
        self.create_subscription(Image, "/drone_1/depth/raw", self._on_depth, qos)
        self.create_subscription(CameraInfo, "/drone_1/camera/camera_info_raw", self._on_info, qos)
        self.get_logger().info("frame_rewriter ready")

    def _on_rgb(self, msg):
        msg.header.frame_id = OPTICAL
        self._rgb_pub.publish(msg)

    def _on_depth(self, msg):
        msg.header.frame_id = OPTICAL
        self._depth_pub.publish(msg)

    def _on_info(self, msg):
        msg.header.frame_id = OPTICAL
        self._info_pub.publish(msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(FrameRewriter())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

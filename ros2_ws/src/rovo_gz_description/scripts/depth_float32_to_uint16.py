#!/usr/bin/env python3
"""Convert Gazebo RGBD depth (32FC1, metres) to RealSense-style 16UC1 (uint16, mm).

Gazebo sim rgbd_camera publishes depth as float32 metres.
RealSense driver publishes 16UC1 uint16 millimetres.
This node converts so downstream tools (rviz depth display, depthimage_to_laserscan)
behave identically with simulation and real hardware.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class DepthConverter(Node):
    def __init__(self):
        super().__init__('depth_float32_to_uint16')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self._cb, qos)
        self._pub = self.create_publisher(
            Image, '/camera/camera/depth/image_rect_raw_uint16', qos)

    def _cb(self, msg: Image):
        if msg.encoding != '32FC1':
            self.get_logger().warn_once(
                f'Expected 32FC1, got {msg.encoding} — passthrough only')
            self._pub.publish(msg)
            return

        arr = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(
            msg.height, msg.width)
        # NaN / inf → 0 (invalid depth, same convention as RealSense driver)
        arr = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
        mm = np.clip(arr * 1000.0, 0, 65535).astype(np.uint16)

        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = '16UC1'
        out.is_bigendian = 0
        out.step = msg.width * 2
        out.data = mm.tobytes()
        self._pub.publish(out)


def main():
    rclpy.init()
    node = DepthConverter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

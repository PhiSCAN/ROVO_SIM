#!/usr/bin/env python3
"""Add Velodyne's per-point ``time`` field to Gazebo lidar scans.

Gazebo's GPU lidar produces each PointCloud2 as a single instantaneous scan
and does not include per-return firing times. In that case each return has a
zero offset from the cloud header timestamp. If a timing field is already
present, the cloud is passed through unchanged.
"""
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


TIMING_FIELDS = {"t", "time", "timestamp"}


class VelodyneTimeAdapter(Node):
    def __init__(self):
        super().__init__("velodyne_time_adapter")
        self._publisher = self.create_publisher(PointCloud2, "/velodyne_points", 10)
        self._subscription = self.create_subscription(
            PointCloud2, "/velodyne_points_raw", self._on_cloud, 10
        )

    def _on_cloud(self, cloud: PointCloud2):
        if any(field.name.lower() in TIMING_FIELDS for field in cloud.fields):
            self._publisher.publish(cloud)
            return

        if cloud.point_step <= 0 or cloud.row_step < cloud.width * cloud.point_step:
            self.get_logger().error("Received malformed PointCloud2 layout")
            return

        # Keep the float aligned even when the original point layout ends on
        # a 2-byte ring field (a common 18-byte Velodyne layout).
        time_offset = (cloud.point_step + 3) & ~3
        output_point_step = time_offset + 4
        row_padding = cloud.row_step - cloud.width * cloud.point_step
        output_row_step = cloud.width * output_point_step + row_padding
        source = memoryview(cloud.data)
        output = bytearray(output_row_step * cloud.height)
        endian = ">" if cloud.is_bigendian else "<"
        time_bytes = struct.pack(endian + "f", 0.0)

        for row in range(cloud.height):
            source_row = row * cloud.row_step
            output_row = row * output_row_step
            for column in range(cloud.width):
                source_offset = source_row + column * cloud.point_step
                output_offset = output_row + column * output_point_step
                output[output_offset:output_offset + cloud.point_step] = source[
                    source_offset:source_offset + cloud.point_step
                ]
                output[output_offset + time_offset:output_offset + time_offset + 4] = time_bytes

            if row_padding:
                source_padding = source_row + cloud.width * cloud.point_step
                output_padding = output_row + cloud.width * output_point_step
                output[output_padding:output_padding + row_padding] = source[
                    source_padding:source_padding + row_padding
                ]

        result = PointCloud2()
        result.header = cloud.header
        result.height = cloud.height
        result.width = cloud.width
        result.fields = list(cloud.fields) + [PointField(
            name="time",
            offset=time_offset,
            datatype=PointField.FLOAT32,
            count=1,
        )]
        result.is_bigendian = cloud.is_bigendian
        result.point_step = output_point_step
        result.row_step = output_row_step
        result.data = output
        result.is_dense = cloud.is_dense
        self._publisher.publish(result)


def main():
    rclpy.init()
    node = VelodyneTimeAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

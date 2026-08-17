#!/usr/bin/env python3
"""Shift EKF biased_pose stamps onto the lidar timeline for NDT bag replay."""

from __future__ import annotations

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2


class BiasedPoseLidarTimeSync(Node):
    def __init__(self) -> None:
        super().__init__("eng_dock_biased_pose_lidar_time_sync")
        self.declare_parameter(
            "input_biased_pose_topic",
            "/localization/pose_twist_fusion_filter/biased_pose_with_covariance",
        )
        self.declare_parameter(
            "output_biased_pose_topic",
            "/localization/pose_twist_fusion_filter/biased_pose_with_covariance_lidar_time",
        )
        self.declare_parameter(
            "lidar_topic", "/localization/util/downsample/pointcloud"
        )

        self._latest_lidar_stamp = None
        self._logged_sync = False

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("lidar_topic").value,
            self._on_lidar,
            best_effort,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("input_biased_pose_topic").value,
            self._on_biased_pose,
            10,
        )
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("output_biased_pose_topic").value,
            10,
        )

    def _on_lidar(self, msg: PointCloud2) -> None:
        self._latest_lidar_stamp = msg.header.stamp

    def _on_biased_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if self._latest_lidar_stamp is None:
            return
        out = copy.deepcopy(msg)
        out.header.stamp = self._latest_lidar_stamp
        self._pub.publish(out)
        if not self._logged_sync:
            pose_time = Time.from_msg(msg.header.stamp)
            lidar_time = Time.from_msg(self._latest_lidar_stamp)
            dt = (pose_time - lidar_time).nanoseconds / 1e9
            self.get_logger().info(
                f"Republishing biased_pose on lidar time for NDT; first pose-lidar dt={dt:.3f}s"
            )
            self._logged_sync = True


def main() -> None:
    rclpy.init()
    node = BiasedPoseLidarTimeSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

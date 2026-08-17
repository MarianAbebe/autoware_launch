#!/usr/bin/env python3
"""Correct GNSS pose Z and fill yaw from motion for eng bag replay.

1) Z: Autoware LocalCartesian forces map_origin.altitude=0, while eng PCD is
   ENU-relative — subtract ENU origin altitude.
2) Yaw: bag has no /autoware_orientation; gnss_poser leaves identity quat.
   Estimate planar yaw from recent GNSS track so EKF/init are not yaw=0.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped


class GnssPoseAltitudeCorrector(Node):
    def __init__(self) -> None:
        super().__init__("eng_dock_gnss_pose_altitude_corrector")
        self.declare_parameter("input_topic", "pose_with_covariance_raw")
        self.declare_parameter("output_topic", "pose_with_covariance")
        # ENU origin altitude used when building the georeferenced PCD (first NovAtel fix).
        self.declare_parameter("map_origin_altitude_m", 1094.784)
        self.declare_parameter("heading_min_travel_m", 1.5)
        self.declare_parameter("heading_yaw_offset_rad", 0.0)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.origin_alt = (
            self.get_parameter("map_origin_altitude_m").get_parameter_value().double_value
        )
        self._min_travel = float(self.get_parameter("heading_min_travel_m").value)
        self._yaw_offset = float(self.get_parameter("heading_yaw_offset_rad").value)
        self._heading_anchor: tuple[float, float] | None = None
        self._yaw: float | None = None
        self._logged_yaw = False

        self._pub = self.create_publisher(PoseWithCovarianceStamped, out_topic, qos)
        self.create_subscription(PoseWithCovarianceStamped, in_topic, self._on_pose, qos)
        self.get_logger().info(
            f"GNSS Z+yaw corrector: {in_topic} → {out_topic} "
            f"(subtract {self.origin_alt:.3f} m; yaw from motion ≥{self._min_travel:.1f} m, "
            f"offset {math.degrees(self._yaw_offset):.1f} deg)"
        )

    def _update_yaw(self, x: float, y: float) -> None:
        if self._heading_anchor is None:
            self._heading_anchor = (x, y)
            return
        x0, y0 = self._heading_anchor
        dist = math.hypot(x - x0, y - y0)
        if dist < self._min_travel:
            return
        raw_yaw = math.atan2(y - y0, x - x0)
        self._yaw = math.atan2(
            math.sin(raw_yaw + self._yaw_offset),
            math.cos(raw_yaw + self._yaw_offset),
        )
        if not self._logged_yaw:
            self.get_logger().info(
                f"GNSS motion heading ready: raw={math.degrees(raw_yaw):.1f} deg "
                f"corrected={math.degrees(self._yaw):.1f} deg "
                f"(travel {dist:.1f} m, offset {math.degrees(self._yaw_offset):.1f} deg)"
            )
            self._logged_yaw = True

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.pose = msg.pose
        out.pose.pose.position.z = msg.pose.pose.position.z - self.origin_alt

        x = out.pose.pose.position.x
        y = out.pose.pose.position.y
        self._update_yaw(x, y)
        if self._yaw is not None:
            # planar yaw → quaternion (roll=pitch=0)
            half = 0.5 * self._yaw
            out.pose.pose.orientation.x = 0.0
            out.pose.pose.orientation.y = 0.0
            out.pose.pose.orientation.z = math.sin(half)
            out.pose.pose.orientation.w = math.cos(half)
            # Reduce yaw covariance once heading is known (was effectively unknown).
            cov = list(out.pose.covariance)
            cov[35] = 0.25  # ~28 deg^2
            out.pose.covariance = cov

        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = GnssPoseAltitudeCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

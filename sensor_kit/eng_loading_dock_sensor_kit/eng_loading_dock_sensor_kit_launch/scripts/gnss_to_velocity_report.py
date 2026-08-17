#!/usr/bin/env python3
"""Publish VelocityReport from GNSS NavSatFix for bag replay without CAN velocity.

Engineering Loading Dock bags do not contain /vehicle/status/velocity_status.
Gyro odometer and pose_initializer stop-checks require vehicle twist derived from it.
This node differentiates successive NavSatFix positions into longitudinal speed.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from autoware_vehicle_msgs.msg import VelocityReport
from sensor_msgs.msg import NavSatFix


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    r = 6378137.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


class GnssToVelocityReport(Node):
    def __init__(self) -> None:
        super().__init__("eng_dock_gnss_to_velocity_report")
        self.declare_parameter("input_gnss_topic", "/novatel/oem7/fix")
        self.declare_parameter("output_velocity_topic", "/vehicle/status/velocity_status")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("max_speed_mps", 30.0)
        self.declare_parameter("min_dt_sec", 0.01)
        self.declare_parameter("min_publish_dt_sec", 0.04)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        in_topic = self.get_parameter("input_gnss_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_velocity_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.max_speed = self.get_parameter("max_speed_mps").get_parameter_value().double_value
        self.min_dt = self.get_parameter("min_dt_sec").get_parameter_value().double_value
        self.min_publish_dt = (
            self.get_parameter("min_publish_dt_sec").get_parameter_value().double_value
        )

        self._prev = None  # (t, lat, lon)
        self._pub = self.create_publisher(VelocityReport, out_topic, 10)
        self._sub = self.create_subscription(NavSatFix, in_topic, self._on_fix, qos)
        self.get_logger().info(
            f"GNSS→VelocityReport: {in_topic} → {out_topic} (frame={self.frame_id})"
        )

    def _on_fix(self, msg: NavSatFix) -> None:
        if msg.status.status < 0:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        lat, lon = msg.latitude, msg.longitude
        v = 0.0
        if self._prev is not None:
            t0, lat0, lon0 = self._prev
            dt = t - t0
            if dt < 0:
                # time jumped backward (bag loop); reset
                self._prev = (t, lat, lon)
                return
            if dt < self.min_publish_dt:
                return
            if dt >= self.min_dt:
                dist = haversine_m(lat0, lon0, lat, lon)
                v = dist / dt
                if v > self.max_speed:
                    v = self.max_speed
        self._prev = (t, lat, lon)

        out = VelocityReport()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.longitudinal_velocity = float(v)
        out.lateral_velocity = 0.0
        out.heading_rate = 0.0
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = GnssToVelocityReport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

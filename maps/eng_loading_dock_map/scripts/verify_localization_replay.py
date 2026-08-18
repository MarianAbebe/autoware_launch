#!/usr/bin/env python3
"""Post-rebuild localization verification: rates + EKF vs GNSS delta."""
from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def yaw_from_quat(x, y, z, w) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class LocVerify(Node):
    def __init__(self) -> None:
        super().__init__("eng_loc_verify")
        self.t0 = time.time()
        self.counts: dict[str, int] = {}
        self.deltas: list[float] = []
        self.yaw_deltas: list[float] = []
        self._latest_gnss: PoseWithCovarianceStamped | None = None

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/sensing/gnss/pose_with_covariance",
            self._on_gnss,
            10,
        )
        self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self._on_ekf,
            10,
        )
        for name, topic, typ in [
            ("downsample", "/localization/util/downsample/pointcloud", PointCloud2),
            ("ndt_pwc", "/localization/pose_estimator/pose_with_covariance", PoseWithCovarianceStamped),
            ("ndt_pose", "/localization/pose_estimator/pose", PoseStamped),
        ]:
            qos = qos_profile_sensor_data if typ is PointCloud2 else 10
            self.create_subscription(
                typ,
                topic,
                lambda _m, n=name: self._bump(n),
                qos,
            )

    def _bump(self, name: str) -> None:
        self.counts[name] = self.counts.get(name, 0) + 1

    def _on_gnss(self, msg: PoseWithCovarianceStamped) -> None:
        self._bump("gnss")
        self._latest_gnss = msg

    def _on_ekf(self, msg: Odometry) -> None:
        self._bump("ekf")
        if self._latest_gnss is None:
            return
        g = self._latest_gnss.pose.pose
        e = msg.pose.pose
        dx = e.position.x - g.position.x
        dy = e.position.y - g.position.y
        dz = e.position.z - g.position.z
        self.deltas.append(math.sqrt(dx * dx + dy * dy + dz * dz))
        gy = yaw_from_quat(g.orientation.x, g.orientation.y, g.orientation.z, g.orientation.w)
        ey = yaw_from_quat(e.orientation.x, e.orientation.y, e.orientation.z, e.orientation.w)
        dyaw = abs(math.atan2(math.sin(ey - gy), math.cos(ey - gy)))
        self.yaw_deltas.append(math.degrees(dyaw))

    def summary(self, elapsed: float) -> dict:
        def rate(key: str) -> float:
            return self.counts.get(key, 0) / elapsed if elapsed > 0 else 0.0

        pos = sorted(self.deltas)
        yaw = sorted(self.yaw_deltas)
        def pct(arr, p):
            return arr[int(len(arr) * p)] if arr else None

        return {
            "elapsed_s": elapsed,
            "counts": dict(self.counts),
            "rates_hz": {
                "downsample": rate("downsample"),
                "ndt_pwc": rate("ndt_pwc"),
                "ekf": rate("ekf"),
                "gnss": rate("gnss"),
            },
            "ekf_gnss_pos_delta_m": {
                "n": len(pos),
                "median": pct(pos, 0.5),
                "p95": pct(pos, 0.95),
                "max": pos[-1] if pos else None,
            },
            "ekf_gnss_yaw_delta_deg": {
                "n": len(yaw),
                "median": pct(yaw, 0.5),
                "p95": pct(yaw, 0.95),
                "max": yaw[-1] if yaw else None,
            },
        }


def main() -> None:
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    rclpy.init()
    node = LocVerify()
    end = time.time() + duration
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    elapsed = time.time() - node.t0
    result = node.summary(elapsed)
    import json

    print(json.dumps(result, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Seed EKF from GNSS without calling pose_initializer's service (avoids deadlock).

For eng bag replay we:
  1) wait until downsample lidar is healthy and sim time tracks lidar stamps
  2) deactivate EKF, publish /initialpose3d stamped with latest lidar time
  3) reactivate EKF and wait for several biased_pose messages
  4) activate NDT last (trigger clears its pose buffer; EKF must already publish)

Uses MultiThreadedExecutor so trigger_node responses can complete while waiting.
"""

from __future__ import annotations

import copy
import math
import time
from collections import deque

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import PointCloud2


class EngDockInitializeAfterReady(Node):
    def __init__(self) -> None:
        super().__init__("eng_dock_initialize_after_ready")
        self.declare_parameter("gnss_topic", "/sensing/gnss/pose_with_covariance")
        self.declare_parameter("twist_topic", "/localization/twist_estimator/twist_with_covariance")
        self.declare_parameter("lidar_ready_topic", "/localization/util/downsample/pointcloud")
        self.declare_parameter(
            "biased_pose_topic",
            "/localization/pose_twist_fusion_filter/biased_pose_internal",
        )
        self.declare_parameter("initialpose_topic", "/initialpose3d")
        self.declare_parameter("ekf_trigger_service", "/localization/pose_twist_fusion_filter/trigger_node")
        self.declare_parameter(
            "ndt_trigger_service", "/localization/pose_estimator/trigger_node"
        )
        self.declare_parameter("wait_timeout_sec", 300.0)
        self.declare_parameter("retry_period_sec", 15.0)
        self.declare_parameter("min_gnss_msgs", 20)
        self.declare_parameter("min_lidar_msgs", 60)
        self.declare_parameter("min_lidar_msgs_recent", 5)
        self.declare_parameter("lidar_recent_window_sec", 3.0)
        self.declare_parameter("require_twist", False)
        self.declare_parameter("startup_delay_sec", 10.0)
        self.declare_parameter("require_motion_heading", True)
        self.declare_parameter("heading_wait_sec", 30.0)
        self.declare_parameter("post_ekf_biased_pose_count", 5)
        self.declare_parameter("post_ekf_warmup_timeout_sec", 8.0)
        self.declare_parameter("post_ndt_settle_sec", 1.0)

        self._latest_gnss: PoseWithCovarianceStamped | None = None
        self._latest_lidar: PointCloud2 | None = None
        self._gnss_count = 0
        self._got_twist = False
        self._lidar_count = 0
        self._lidar_wall_times: deque[float] = deque(maxlen=512)
        self._biased_pose_count = 0
        self._in_flight = False
        self._done = False
        self._next_attempt = time.time()
        self._heading_warned = False
        self._bag_warm = False

        self._cb_group = ReentrantCallbackGroup()
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("gnss_topic").value,
            self._on_gnss,
            reliable,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            TwistWithCovarianceStamped,
            self.get_parameter("twist_topic").value,
            lambda _m: setattr(self, "_got_twist", True),
            best_effort,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("lidar_ready_topic").value,
            self._on_lidar,
            best_effort,
            callback_group=self._cb_group,
        )
        self._biased_pose_sub = None
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("initialpose_topic").value,
            10,
        )
        self._ekf_cli = self.create_client(
            SetBool,
            self.get_parameter("ekf_trigger_service").value,
            callback_group=self._cb_group,
        )
        self._ndt_cli = self.create_client(
            SetBool,
            self.get_parameter("ndt_trigger_service").value,
            callback_group=self._cb_group,
        )
        self._start = time.time()
        self.create_timer(1.0, self._tick, callback_group=self._cb_group)
        self.get_logger().info(
            "Waiting for healthy downsample + GNSS, then EKF seed + NDT activation"
        )

    def _on_gnss(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_gnss = msg
        if self._bag_warm:
            self._gnss_count += 1

    def _on_lidar(self, msg: PointCloud2) -> None:
        self._bag_warm = True
        self._latest_lidar = msg
        self._lidar_count += 1
        self._lidar_wall_times.append(time.time())

    def _on_biased_pose(self, _msg: PoseWithCovarianceStamped) -> None:
        self._biased_pose_count += 1

    @staticmethod
    def _has_motion_heading(pose: PoseWithCovarianceStamped) -> bool:
        q = pose.pose.pose.orientation
        return abs(q.x) + abs(q.y) + abs(q.z) > 1e-3

    def _recent_lidar_count(self) -> int:
        window = float(self.get_parameter("lidar_recent_window_sec").value)
        cutoff = time.time() - window
        while self._lidar_wall_times and self._lidar_wall_times[0] < cutoff:
            self._lidar_wall_times.popleft()
        return len(self._lidar_wall_times)

    def _ready(self) -> bool:
        if time.time() - self._start < float(self.get_parameter("startup_delay_sec").value):
            return False
        if not self._bag_warm or self._latest_lidar is None:
            return False
        if self._latest_gnss is None:
            return False

        need_g = int(self.get_parameter("min_gnss_msgs").value)
        need_l = int(self.get_parameter("min_lidar_msgs").value)
        need_recent = int(self.get_parameter("min_lidar_msgs_recent").value)
        need_twist = bool(self.get_parameter("require_twist").value)
        if not (
            self._gnss_count >= need_g
            and (self._got_twist or not need_twist)
            and self._lidar_count >= need_l
            and self._recent_lidar_count() >= need_recent
        ):
            return False

        if not bool(self.get_parameter("require_motion_heading").value):
            return True
        if self._has_motion_heading(self._latest_gnss):
            return True

        wait = float(self.get_parameter("heading_wait_sec").value)
        elapsed = time.time() - self._start - float(self.get_parameter("startup_delay_sec").value)
        if elapsed >= wait:
            if not self._heading_warned:
                self.get_logger().warn(
                    "Motion heading not ready; seeding with identity yaw "
                    "(vehicle may look rotated vs map until NDT converges)"
                )
                self._heading_warned = True
            return True
        return False

    def _call_trigger(self, client, name: str, flag: bool, timeout_sec: float = 5.0) -> bool:
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"{name} trigger_node service not available")
            return False
        req = SetBool.Request()
        req.data = flag
        fut = client.call_async(req)
        end = time.time() + timeout_sec
        while rclpy.ok() and not fut.done() and time.time() < end:
            time.sleep(0.05)
        if not fut.done():
            self.get_logger().error(f"{name} trigger({flag}) timed out")
            return False
        try:
            res = fut.result()
            self.get_logger().info(f"{name} trigger({flag}) ok={res.success} msg={res.message}")
            return bool(res.success)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"{name} trigger({flag}) failed: {exc}")
            return False

    def _call_ekf_trigger(self, flag: bool) -> bool:
        return self._call_trigger(self._ekf_cli, "EKF", flag)

    def _call_ndt_trigger(self, flag: bool) -> bool:
        return self._call_trigger(self._ndt_cli, "NDT", flag)

    def _wait_for_biased_poses(self) -> bool:
        need = int(self.get_parameter("post_ekf_biased_pose_count").value)
        timeout = float(self.get_parameter("post_ekf_warmup_timeout_sec").value)
        self._biased_pose_count = 0
        if self._biased_pose_sub is None:
            self._biased_pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self.get_parameter("biased_pose_topic").value,
                self._on_biased_pose,
                10,
                callback_group=self._cb_group,
            )
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            if self._biased_pose_count >= need:
                return True
            time.sleep(0.05)
        self.get_logger().error(
            f"Timed out waiting for {need} biased_pose messages (got {self._biased_pose_count})"
        )
        return False

    def _tick(self) -> None:
        if self._done or self._in_flight:
            return
        timeout = float(self.get_parameter("wait_timeout_sec").value)
        if time.time() - self._start > timeout:
            self.get_logger().error(
                "Timed out waiting for init inputs "
                f"(gnss={self._gnss_count} twist={self._got_twist} lidar={self._lidar_count} "
                f"recent_lidar={self._recent_lidar_count()})"
            )
            self._done = True
            return
        if not self._ready() or time.time() < self._next_attempt:
            return

        pose = copy.deepcopy(self._latest_gnss)
        assert pose is not None
        assert self._latest_lidar is not None
        self._in_flight = True

        # Align EKF seed time with the live lidar stream so NDT SmartPoseBuffer can interpolate.
        pose.header.stamp = self._latest_lidar.header.stamp
        pose.header.frame_id = "map"

        q = pose.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.get_logger().info(
            "Seeding localization via /initialpose3d "
            f"xyz=({pose.pose.pose.position.x:.2f},"
            f"{pose.pose.pose.position.y:.2f},"
            f"{pose.pose.pose.position.z:.2f}) "
            f"yaw_deg={math.degrees(yaw):.1f} "
            f"gnss={self._gnss_count} lidar={self._lidar_count} "
            f"recent_lidar={self._recent_lidar_count()}"
        )

        self._call_ekf_trigger(False)
        self._pose_pub.publish(pose)
        time.sleep(0.1)
        if not self._call_ekf_trigger(True):
            self._next_attempt = time.time() + float(self.get_parameter("retry_period_sec").value)
            self._in_flight = False
            return

        if not self._wait_for_biased_poses():
            self._next_attempt = time.time() + float(self.get_parameter("retry_period_sec").value)
            self._in_flight = False
            return

        if not self._call_ndt_trigger(True):
            self._next_attempt = time.time() + float(self.get_parameter("retry_period_sec").value)
            self._in_flight = False
            return

        settle = float(self.get_parameter("post_ndt_settle_sec").value)
        time.sleep(settle)
        self.get_logger().info(
            "Localization seed complete; EKF publishing and NDT scan matcher activated"
        )
        self._done = True
        self._in_flight = False


def main() -> None:
    rclpy.init()
    node = EngDockInitializeAfterReady()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

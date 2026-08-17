#!/usr/bin/env python3
"""Convert bag Velodyne PointCloud2 (XYZI / XYZIR / XYZIRT) to Autoware PointXYZIRC.

Autoware localization crop_box / voxel filters reject legacy layouts and only
accept PointXYZIRC or PointXYZIRCAEDT (see Filter::faster_input_indices_callback).

Engineering Loading Dock bags publish /velodyne_points with float intensity and
optional ring/time fields. This node remaps that cloud into the PointXYZIRC
memory layout without touching Autoware core.
"""

from __future__ import annotations

import struct
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


# PointXYZIRC packing (autoware_point_types::PointXYZIRC)
# float x,y,z; uint8 intensity; uint8 return_type; uint16 channel  → 16 bytes
XYZIRC_POINT_STEP = 16
XYZIRC_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.UINT8, count=1),
    PointField(name="return_type", offset=13, datatype=PointField.UINT8, count=1),
    PointField(name="channel", offset=14, datatype=PointField.UINT16, count=1),
]

_DATATYPE_SIZES = {
    PointField.INT8: 1,
    PointField.UINT8: 1,
    PointField.INT16: 2,
    PointField.UINT16: 2,
    PointField.INT32: 4,
    PointField.UINT32: 4,
    PointField.FLOAT32: 4,
    PointField.FLOAT64: 8,
}


def _field_map(msg: PointCloud2) -> Dict[str, PointField]:
    return {f.name: f for f in msg.fields}


def _unpack_fmt(datatype: int) -> str:
    return {
        PointField.INT8: "b",
        PointField.UINT8: "B",
        PointField.INT16: "h",
        PointField.UINT16: "H",
        PointField.INT32: "i",
        PointField.UINT32: "I",
        PointField.FLOAT32: "f",
        PointField.FLOAT64: "d",
    }[datatype]


def _read_field(data: bytes, base: int, field: PointField, endian: str) -> float:
    size = _DATATYPE_SIZES[field.datatype]
    raw = data[base + field.offset : base + field.offset + size]
    return struct.unpack(endian + _unpack_fmt(field.datatype), raw)[0]


def _already_xyzirc(msg: PointCloud2) -> bool:
    if len(msg.fields) < 6 or msg.point_step < XYZIRC_POINT_STEP:
        return False
    want = [("x", PointField.FLOAT32), ("y", PointField.FLOAT32), ("z", PointField.FLOAT32),
            ("intensity", PointField.UINT8), ("return_type", PointField.UINT8),
            ("channel", PointField.UINT16)]
    for i, (name, dt) in enumerate(want):
        f = msg.fields[i]
        if f.name != name or f.datatype != dt or f.count != 1:
            return False
    return True


class VelodynePointsToXyzirc(Node):
    def __init__(self) -> None:
        super().__init__("eng_dock_velodyne_points_to_xyzirc")
        self.declare_parameter("input_topic", "/velodyne_points")
        self.declare_parameter("output_topic", "/sensing/lidar/concatenated/pointcloud")
        self.declare_parameter("default_return_type", 1)  # SINGLE_STRONGEST

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self._default_return = int(self.get_parameter("default_return_type").value)

        # Bag PointCloud2 is typically RELIABLE; localization crop_box uses BEST_EFFORT.
        # Publish BEST_EFFORT so both RViz and Autoware filters can subscribe.
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub = self.create_publisher(PointCloud2, out_topic, pub_qos)
        self._sub = self.create_subscription(PointCloud2, in_topic, self._on_cloud, sub_qos)
        self.get_logger().info(f"XYZIR→PointXYZIRC: {in_topic} → {out_topic}")

    def _on_cloud(self, msg: PointCloud2) -> None:
        if _already_xyzirc(msg):
            out = PointCloud2()
            out.header = msg.header
            out.height = msg.height
            out.width = msg.width
            out.fields = list(XYZIRC_FIELDS)
            out.is_bigendian = False
            out.point_step = XYZIRC_POINT_STEP
            out.row_step = XYZIRC_POINT_STEP * msg.width
            out.is_dense = msg.is_dense
            # Re-pack first 16 bytes per point if step differs
            if msg.point_step == XYZIRC_POINT_STEP and not msg.is_bigendian:
                out.data = msg.data
            else:
                out.data = self._repack_xyzirc(msg)
            self._pub.publish(out)
            return

        fmap = _field_map(msg)
        if not all(k in fmap for k in ("x", "y", "z")):
            self.get_logger().error("Input cloud missing x/y/z; dropping", throttle_duration_sec=5.0)
            return

        endian = ">" if msg.is_bigendian else "<"
        intensity_f = fmap.get("intensity")
        ring_f = fmap.get("ring") or fmap.get("channel")
        n_points = msg.width * msg.height
        out_buf = bytearray(n_points * XYZIRC_POINT_STEP)

        for i in range(n_points):
            base = i * msg.point_step
            if base + msg.point_step > len(msg.data):
                break
            x = _read_field(msg.data, base, fmap["x"], endian)
            y = _read_field(msg.data, base, fmap["y"], endian)
            z = _read_field(msg.data, base, fmap["z"], endian)
            intensity_u8 = 0
            if intensity_f is not None:
                raw_i = _read_field(msg.data, base, intensity_f, endian)
                if intensity_f.datatype == PointField.FLOAT32:
                    # Common Velodyne scale 0–255 in float
                    intensity_u8 = int(max(0.0, min(255.0, raw_i)))
                else:
                    intensity_u8 = int(max(0, min(255, int(raw_i))))
            channel = 0
            if ring_f is not None:
                channel = int(_read_field(msg.data, base, ring_f, endian)) & 0xFFFF

            o = i * XYZIRC_POINT_STEP
            struct.pack_into("<fffBBH", out_buf, o, x, y, z, intensity_u8, self._default_return, channel)

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = n_points
        out.fields = list(XYZIRC_FIELDS)
        out.is_bigendian = False
        out.point_step = XYZIRC_POINT_STEP
        out.row_step = XYZIRC_POINT_STEP * n_points
        out.is_dense = msg.is_dense
        out.data = bytes(out_buf)
        self._pub.publish(out)

    def _repack_xyzirc(self, msg: PointCloud2) -> bytes:
        n = msg.width * msg.height
        out = bytearray(n * XYZIRC_POINT_STEP)
        for i in range(n):
            src = i * msg.point_step
            dst = i * XYZIRC_POINT_STEP
            out[dst : dst + XYZIRC_POINT_STEP] = msg.data[src : src + XYZIRC_POINT_STEP]
        return bytes(out)


def main() -> None:
    rclpy.init()
    node = VelodynePointsToXyzirc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

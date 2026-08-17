#!/usr/bin/env python3
"""Dedupe MarkerArray (ns,id) for eng loading-dock Lanelet viz.

Autoware's autowareTrafficLightsAsMarkerArray() pushes the same TRIANGLE_LIST
marker on every traffic-light regulatory element. When multiple regs share a
light (common in this OSM), RViz errors with:
  Multiple Markers in the same MarkerArray message had the same ns and id:
  {traffic_light_triangle, N}

Republish unique (ns,id) markers for eng RViz (topic ..._viz).
"""

from __future__ import annotations

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

# RViz/Ogre ManualObject often drops a single TRIANGLE_LIST with >65k verts.
# This campus OSM packs ~150k–170k verts into one marker per namespace.
_MAX_TRIANGLE_POINTS = 60000  # multiple of 3, under 2^16


class DedupeMarkerArray(Node):
    def __init__(self) -> None:
        super().__init__("eng_dock_dedupe_vector_map_markers")
        self.declare_parameter("input_topic", "/map/vector_map_marker")
        self.declare_parameter("output_topic", "/map/vector_map_marker_viz")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(
            MarkerArray, self.get_parameter("output_topic").value, qos
        )
        self.create_subscription(
            MarkerArray,
            self.get_parameter("input_topic").value,
            self._on_markers,
            qos,
        )
        self._logged_drop = False
        self.get_logger().info(
            "Dedupe MarkerArray: "
            f"{self.get_parameter('input_topic').value} → "
            f"{self.get_parameter('output_topic').value}"
        )

    def _on_markers(self, msg: MarkerArray) -> None:
        seen: set[tuple[str, int]] = set()
        out = MarkerArray()
        dropped = 0
        split = 0
        for m in msg.markers:
            for piece in _split_triangle_list(m):
                key = (piece.ns, int(piece.id))
                if key in seen:
                    dropped += 1
                    continue
                seen.add(key)
                out.markers.append(piece)
                if piece is not m:
                    split += 1
        if (dropped or split) and not self._logged_drop:
            self.get_logger().info(
                f"Dedupe/split: dropped {dropped} dupes, split {split} extra "
                f"chunks (kept {len(out.markers)} from {len(msg.markers)})"
            )
            self._logged_drop = True
        self._pub.publish(out)


def _split_triangle_list(marker: Marker) -> list[Marker]:
    if marker.type != Marker.TRIANGLE_LIST:
        return [marker]
    n = len(marker.points)
    if n <= _MAX_TRIANGLE_POINTS:
        return [marker]
    chunks: list[Marker] = []
    for i, start in enumerate(range(0, n, _MAX_TRIANGLE_POINTS)):
        chunk = deepcopy(marker)
        chunk.id = int(marker.id) + i
        stop = min(start + _MAX_TRIANGLE_POINTS, n)
        stop -= (stop - start) % 3
        if stop <= start:
            continue
        chunk.points = list(marker.points[start:stop])
        if marker.colors:
            chunk.colors = list(marker.colors[start:stop])
        chunks.append(chunk)
    return chunks or [marker]


def main() -> None:
    rclpy.init()
    node = DedupeMarkerArray()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

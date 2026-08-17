#!/usr/bin/env python3

import math
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def floats_from_attr(element, name, default):
    value = element.get(name) if element is not None else None
    if not value:
        return default
    return tuple(float(item) for item in value.split())


def fixed_joint_transforms(robot_description: str):
    root = ET.fromstring(robot_description)
    transforms = []

    for joint in root.findall("joint"):
        if joint.get("type") != "fixed":
            continue

        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue

        origin = joint.find("origin")
        xyz = floats_from_attr(origin, "xyz", (0.0, 0.0, 0.0))
        rpy = floats_from_attr(origin, "rpy", (0.0, 0.0, 0.0))
        qx, qy, qz, qw = quaternion_from_rpy(*rpy)

        transform = TransformStamped()
        transform.header.frame_id = parent.get("link")
        transform.child_frame_id = child.get("link")
        transform.transform.translation.x = xyz[0]
        transform.transform.translation.y = xyz[1]
        transform.transform.translation.z = xyz[2]
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        transforms.append(transform)

    return transforms


class FixedTfRepublisher(Node):
    def __init__(self):
        super().__init__("fixed_tf_republisher")
        self.declare_parameter("robot_description", "")
        robot_description = self.get_parameter("robot_description").get_parameter_value().string_value
        self.transforms = fixed_joint_transforms(robot_description)
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.on_timer)
        self.get_logger().info(f"Republishing {len(self.transforms)} fixed transforms on /tf")

    def on_timer(self):
        stamp = self.get_clock().now().to_msg()
        for transform in self.transforms:
            transform.header.stamp = stamp
        self.broadcaster.sendTransform(self.transforms)


def main():
    rclpy.init()
    node = FixedTfRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

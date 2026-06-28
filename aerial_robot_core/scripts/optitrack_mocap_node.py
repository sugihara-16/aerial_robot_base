#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
import math
import select
import socket
import struct

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped
from rclpy.node import Node


class NatNetParseError(Exception):
    pass


class NatNetFrameParser:
    def __init__(self, data):
        self.data = data
        self.offset = 0

    def remaining(self):
        return len(self.data) - self.offset

    def seek(self, offset):
        if offset < 0 or offset > len(self.data):
            raise NatNetParseError("seek outside packet")
        self.offset = offset

    def read(self, fmt):
        size = struct.calcsize("<" + fmt)
        if self.offset + size > len(self.data):
            raise NatNetParseError("packet ended while reading")
        value = struct.unpack_from("<" + fmt, self.data, self.offset)
        self.offset += size
        return value[0] if len(value) == 1 else value

    def skip(self, size):
        if self.offset + size > len(self.data):
            raise NatNetParseError("packet ended while skipping")
        self.offset += size

    def skip_c_string(self):
        end = self.data.find(b"\0", self.offset)
        if end < 0:
            raise NatNetParseError("unterminated string")
        self.offset = end + 1


class OptiTrackMocapNode(Node):
    def __init__(self):
        super().__init__("mocap_node")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("multicast_address", "239.255.42.99")
        self.declare_parameter("data_port", 1511)
        self.declare_parameter("interface_address", "0.0.0.0")
        self.declare_parameter("pose_topic", "mocap/pose")
        self.declare_parameter("pose2d_topic", "mocap/ground_pose")
        self.declare_parameter("parent_frame_id", "world")
        self.declare_parameter("publish_pose2d", True)
        self.declare_parameter("poll_period", 0.001)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.parent_frame_id = str(self.get_parameter("parent_frame_id").value)
        self.publish_pose2d = bool(self.get_parameter("publish_pose2d").value)

        pose_topic = str(self.get_parameter("pose_topic").value)
        pose2d_topic = str(self.get_parameter("pose2d_topic").value)
        multicast_address = str(self.get_parameter("multicast_address").value)
        interface_address = str(self.get_parameter("interface_address").value)
        data_port = int(self.get_parameter("data_port").value)
        poll_period = float(self.get_parameter("poll_period").value)

        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.pose2d_pub = self.create_publisher(Pose2D, pose2d_topic, 10) if self.publish_pose2d else None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", data_port))
        membership = struct.pack("4s4s", socket.inet_aton(multicast_address), socket.inet_aton(interface_address))
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        self.sock.setblocking(False)

        self.last_pose_time = self.get_clock().now()
        self.last_warn_time = self.get_clock().now()
        self.timer = self.create_timer(poll_period, self.poll)

        self.get_logger().info(
            f"listening for NatNet data on {multicast_address}:{data_port}, "
            f"robot_id={self.robot_id}, pose_topic={pose_topic}"
        )

    def destroy_node(self):
        try:
            self.sock.close()
        finally:
            super().destroy_node()

    def poll(self):
        ready, _, _ = select.select([self.sock], [], [], 0.0)
        processed = 0
        while ready and processed < 32:
            try:
                data, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                break

            pose = self.parse_pose(data)
            if pose is not None:
                self.publish_pose(pose)

            processed += 1
            ready, _, _ = select.select([self.sock], [], [], 0.0)

        now = self.get_clock().now()
        if (now - self.last_pose_time).nanoseconds > 2_000_000_000:
            if (now - self.last_warn_time).nanoseconds > 2_000_000_000:
                self.get_logger().warn(f"no NatNet pose received for robot_id={self.robot_id}")
                self.last_warn_time = now

    def parse_pose(self, data):
        if len(data) < 8:
            return None

        message_id, payload_size = struct.unpack_from("<HH", data, 0)
        if message_id != 7 or payload_size + 4 > len(data):
            return None

        candidates = []

        try:
            candidates.extend(self.parse_natnet3_rigid_bodies(data))
        except NatNetParseError:
            pass

        candidates.extend(self.scan_pose_candidates(data))
        if not candidates:
            return None

        return min(candidates, key=lambda candidate: abs(self.quaternion_norm(candidate[3:]) - 1.0))

    def parse_natnet3_rigid_bodies(self, data):
        parser = NatNetFrameParser(data)
        parser.seek(4)
        parser.skip(4)  # Frame number

        marker_set_count = parser.read("i")
        for _ in range(marker_set_count):
            parser.skip_c_string()
            marker_count = parser.read("i")
            parser.skip(marker_count * 12)

        unlabeled_marker_count = parser.read("i")
        parser.skip(unlabeled_marker_count * 12)

        candidates = []
        rigid_body_count = parser.read("i")
        for _ in range(rigid_body_count):
            pose = self.read_rigid_body(parser)
            if pose is not None:
                candidates.append(pose)

        if parser.remaining() >= 4:
            skeleton_count = parser.read("i")
            for _ in range(skeleton_count):
                parser.skip(4)  # Skeleton id
                skeleton_rigid_body_count = parser.read("i")
                for _ in range(skeleton_rigid_body_count):
                    pose = self.read_rigid_body(parser)
                    if pose is not None:
                        candidates.append(pose)

        return candidates

    def read_rigid_body(self, parser):
        rigid_body_id = parser.read("i")
        values = parser.read("7f")
        marker_count = parser.read("i") if parser.remaining() >= 4 else 0
        if marker_count < 0 or marker_count > 10000:
            raise NatNetParseError("invalid marker count")

        parser.skip(marker_count * 12)
        parser.skip(marker_count * 4)
        parser.skip(marker_count * 4)
        if parser.remaining() >= 4:
            parser.skip(4)  # Mean marker error
        if parser.remaining() >= 2:
            parser.skip(2)  # Tracking params

        if rigid_body_id != self.robot_id:
            return None
        return values if self.valid_pose(values) else None

    def scan_pose_candidates(self, data):
        candidates = []
        for offset in range(4, len(data) - 32, 4):
            rigid_body_id = struct.unpack_from("<i", data, offset)[0]
            if rigid_body_id != self.robot_id:
                continue
            values = struct.unpack_from("<7f", data, offset + 4)
            if self.valid_pose(values):
                candidates.append(values)
        return candidates

    @staticmethod
    def quaternion_norm(quaternion):
        qx, qy, qz, qw = quaternion
        return math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)

    def valid_pose(self, values):
        if any(not math.isfinite(value) for value in values):
            return False
        x, y, z, qx, qy, qz, qw = values
        if max(abs(x), abs(y), abs(z)) > 100.0:
            return False
        q_norm = self.quaternion_norm((qx, qy, qz, qw))
        return 0.8 <= q_norm <= 1.2

    def publish_pose(self, values):
        x, y, z, qx, qy, qz, qw = values
        ros_qx, ros_qy, ros_qz, ros_qw = self.normalize_quaternion((qx, -qz, qy, qw))

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(-z)
        msg.pose.position.z = float(y)
        msg.pose.orientation.x = ros_qx
        msg.pose.orientation.y = ros_qy
        msg.pose.orientation.z = ros_qz
        msg.pose.orientation.w = ros_qw
        self.pose_pub.publish(msg)

        if self.pose2d_pub is not None:
            pose2d = Pose2D()
            pose2d.x = msg.pose.position.x
            pose2d.y = msg.pose.position.y
            pose2d.theta = self.yaw_from_quaternion(ros_qx, ros_qy, ros_qz, ros_qw)
            self.pose2d_pub.publish(pose2d)

        self.last_pose_time = self.get_clock().now()

    @staticmethod
    def normalize_quaternion(quaternion):
        norm = OptiTrackMocapNode.quaternion_norm(quaternion)
        if norm == 0.0:
            return 0.0, 0.0, 0.0, 1.0
        return tuple(value / norm for value in quaternion)

    @staticmethod
    def yaw_from_quaternion(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = OptiTrackMocapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

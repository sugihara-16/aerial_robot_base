#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_ARGS = [
    ("robot_ns", "", "Namespace for the mocap node and published topics"),
    ("robot_id", "1", "OptiTrack rigid body id to publish"),
    ("pose_topic", "mocap/pose", "PoseStamped topic published by the OptiTrack receiver"),
    ("pose2d_topic", "mocap/ground_pose", "Pose2D topic published by the OptiTrack receiver"),
    ("parent_frame_id", "world", "Frame id used in the PoseStamped header"),
    ("multicast_address", "239.255.42.99", "OptiTrack NatNet multicast address"),
    ("data_port", "1511", "OptiTrack NatNet data port"),
    ("interface_address", "0.0.0.0", "Local interface address used to join the multicast group"),
    ("poll_period", "0.001", "Socket polling period [s]"),
    ("publish_pose2d", "true", "Publish mocap ground pose as geometry_msgs/Pose2D", ["true", "false"]),
]


def _as_int(context, name):
    value = LaunchConfiguration(name).perform(context)
    try:
        return int(value)
    except ValueError as exc:
        raise RuntimeError(f"{name} must be an integer, got '{value}'") from exc


def _as_float(context, name):
    value = LaunchConfiguration(name).perform(context)
    try:
        return float(value)
    except ValueError as exc:
        raise RuntimeError(f"{name} must be a float, got '{value}'") from exc


def _launch_setup(context, *args, **kwargs):
    robot_ns = LaunchConfiguration("robot_ns").perform(context)
    if robot_ns == "/":
        robot_ns = ""

    robot_id = LaunchConfiguration("robot_id").perform(context)
    if not robot_id:
        raise RuntimeError("robot_id must not be empty")

    publish_pose2d = LaunchConfiguration("publish_pose2d").perform(context).lower() == "true"

    return [
        Node(
            package="aerial_robot_core",
            executable="optitrack_mocap_node.py",
            name="mocap_node",
            namespace=robot_ns,
            parameters=[
                {
                    "robot_id": int(robot_id),
                    "multicast_address": LaunchConfiguration("multicast_address").perform(context),
                    "data_port": _as_int(context, "data_port"),
                    "interface_address": LaunchConfiguration("interface_address").perform(context),
                    "pose_topic": LaunchConfiguration("pose_topic").perform(context),
                    "pose2d_topic": LaunchConfiguration("pose2d_topic").perform(context),
                    "parent_frame_id": LaunchConfiguration("parent_frame_id").perform(context),
                    "publish_pose2d": publish_pose2d,
                    "poll_period": _as_float(context, "poll_period"),
                },
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            name,
            default_value=default_value,
            description=description,
            **({"choices": choices[0]} if choices else {}),
        )
        for name, default_value, description, *choices in _ARGS
    ]

    ld = LaunchDescription()
    for arg in declared_args:
        ld.add_action(arg)
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld

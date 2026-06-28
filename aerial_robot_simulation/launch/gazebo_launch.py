#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, Shutdown
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
    EnvironmentVariable,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix
from ament_index_python.packages import get_package_share_directory

# ---------------------------------------------------------------------------
# Argument declarations  (name, default, description, (optional) choices)
# ---------------------------------------------------------------------------
pkg_share = get_package_share_directory("aerial_robot_simulation")
default_world = os.path.join(pkg_share, "gazebo_model", "world", "empty.world")
_ARGS = [
    ("robot_ns", "hydrus", "Namespace for all robot nodes"),
    ("world_sdf_file", default_world, "Ignition world SDF file path (default: empty world included in this package)"),
    ("headless", "true", "Run without GUI", ["true", "false"]),
    ("sim_param_path", "", "Path to YAML file with simulation parameters"),
    ("spawn_x", "0.0", "Gazebo spawn X position [m] (sim only)"),
    ("spawn_y", "0.0", "Gazebo spawn Y position [m] (sim only)"),
    ("spawn_z", "0.5", "Gazebo spawn Z position [m] (sim only)"),
]


def generate_launch_description():
    # ------------------------------------------------------------------
    # 1.  Declare CLI-overridable arguments
    # ------------------------------------------------------------------
    declared_args = [
        DeclareLaunchArgument(
            name, default_value=default_value, description=description, **({"choices": choices[0]} if choices else {})
        )
        for name, default_value, description, *choices in _ARGS
    ]

    # Resolve / Read at launch time (NOT AT IMPORT TIME)
    robot_ns = LaunchConfiguration("robot_ns")
    world = LaunchConfiguration("world_sdf_file")
    headless = LaunchConfiguration("headless")
    sim_param_path = LaunchConfiguration("sim_param_path")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")

    # ------------------------------------------------------------------
    # 2.  Derived paths & environment variables
    # ------------------------------------------------------------------
    robot_share_dir = PathJoinSubstitution([FindPackagePrefix(robot_ns), "share"])

    # Pass through DISPLAY env var for Gazebo GUI (if not headless)
    set_env = SetEnvironmentVariable(name="DISPLAY", value=os.environ.get("DISPLAY", ""))
    set_xauthority = SetEnvironmentVariable(
        name="XAUTHORITY", value=os.environ.get("XAUTHORITY", os.path.expanduser("~/.Xauthority"))
    )

    # Gazebo looks for models, worlds, and plugins in GZ_SIM_RESOURCE_PATH
    set_gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            TextSubstitution(text=":"),
            robot_share_dir,
        ],
    )

    set_gazebo_default_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            TextSubstitution(text="/opt/ros/humble/lib"),
            TextSubstitution(text=os.pathsep),
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
        ],
    )

    # Fortress (`ign gazebo`) uses IGN_GAZEBO_* env vars
    set_ignition_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
            TextSubstitution(text=":"),
            robot_share_dir,
        ],
    )

    set_ignition_default_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
        value=[
            TextSubstitution(text="/opt/ros/humble/lib"),
            TextSubstitution(text=os.pathsep),
            EnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default_value=""),
        ],
    )

    set_fastrtps_profile = SetEnvironmentVariable(
        name="FASTRTPS_DEFAULT_PROFILES_FILE", value=os.path.join(pkg_share, "config", "fastrtps_profiles.xml")
    )

    # ------------------------------------------------------------------
    # 3.  Execute Gazebo processes
    # ------------------------------------------------------------------
    ign_server = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-s", "libgazebo_ros_factory.so", "-s", "libgazebo_ros_init.so", world],
        output="screen",
    )

    ign_client = ExecuteProcess(
        cmd=[
            "ign",
            "gazebo",
            "-g",
        ],
        condition=UnlessCondition(headless),
        output="screen",
    )

    shutdown_handler = RegisterEventHandler(OnProcessExit(target_action=ign_server, on_exit=[Shutdown()]))

    # ------------------------------------------------------------------
    # 3.  Nodes
    # ------------------------------------------------------------------
    sim_param_server = Node(
        package="aerial_robot_simulation",
        executable="sim_param_server",
        name="sim_param_server",
        namespace=robot_ns,
        parameters=[sim_param_path],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            robot_ns,
            "-topic",
            PythonExpression(["'/' + '", robot_ns, "' + '/robot_description'"]),
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
        ],
        output="screen",
    )

    att_controller_spawner = Node(
        namespace=robot_ns,
        package="controller_manager",
        executable="spawner",
        name="spawn_att_controller",
        output="screen",
        arguments=["attitude_controller", "--param-file", sim_param_path],
        parameters=[{"use_sim_time": True}],
    )

    joint_state_broadcaster_spawner = Node(
        namespace=robot_ns,
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": True}],
    )

    ld = LaunchDescription()

    for arg in declared_args:
        ld.add_action(arg)

    ld.add_action(set_env)
    ld.add_action(set_xauthority)
    ld.add_action(set_gazebo_resource_path)
    ld.add_action(set_gazebo_default_path)
    ld.add_action(set_ignition_resource_path)
    ld.add_action(set_ignition_default_path)
    ld.add_action(set_fastrtps_profile)
    ld.add_action(shutdown_handler)
    ld.add_action(sim_param_server)
    ld.add_action(ign_server)
    ld.add_action(ign_client)
    ld.add_action(clock_bridge)
    ld.add_action(spawn_robot)
    ld.add_action(att_controller_spawner)
    ld.add_action(joint_state_broadcaster_spawner)

    return ld

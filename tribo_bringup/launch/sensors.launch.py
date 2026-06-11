#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction

# Lidar serial port (Silicon Labs CP2102N). Use by-id so it never collides with the
# base board (CH340 -> usb-1a86_USB_Serial); raw /dev/ttyUSB0 ordering is not stable.
LIDAR_SERIAL_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
    "c4c4102ee863ef1196dcdaa9c169b110-if00-port0"
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _include_lidar(context, *args, **kwargs):
    use_lidar = LaunchConfiguration("use_lidar").perform(context)
    if use_lidar.lower() not in ("true", "1", "yes", "on"):
        return []

    pkg = LaunchConfiguration("lidar_launch_pkg").perform(context)
    lf = LaunchConfiguration("lidar_launch_file").perform(context)
    serial_port = LaunchConfiguration("lidar_serial_port").perform(context)

    launch_path = os.path.join(get_package_share_directory(pkg), "launch", lf)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={"serial_port": serial_port}.items(),
    )]

def generate_launch_description():
    # Start lidar driver launch (optional)
    declare_use = DeclareLaunchArgument("use_lidar", default_value="true")
    declare_pkg = DeclareLaunchArgument("lidar_launch_pkg", default_value="sllidar_ros2")
    declare_file = DeclareLaunchArgument("lidar_launch_file", default_value="sllidar_c1_launch.py")
    declare_port = DeclareLaunchArgument("lidar_serial_port", default_value=LIDAR_SERIAL_PORT)

    # Frames
    declare_base = DeclareLaunchArgument("base_frame", default_value="base_link")
    declare_lidar = DeclareLaunchArgument("lidar_frame", default_value="laser")

    # Lidar mounting TF (edit after you measure)
    declare_x = DeclareLaunchArgument("x", default_value="0.15")
    declare_y = DeclareLaunchArgument("y", default_value="0.00")
    declare_z = DeclareLaunchArgument("z", default_value="0.20")
    declare_yaw = DeclareLaunchArgument("yaw", default_value="0.0")

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_static_tf",
        output="screen",
        arguments=[
            LaunchConfiguration("x"),
            LaunchConfiguration("y"),
            LaunchConfiguration("z"),
            "0.0", "0.0",
            LaunchConfiguration("yaw"),
            LaunchConfiguration("base_frame"),
            LaunchConfiguration("lidar_frame"),
        ],
    )

    return LaunchDescription([
        declare_use, declare_pkg, declare_file, declare_port,
        declare_base, declare_lidar,
        declare_x, declare_y, declare_z, declare_yaw,
        static_tf,
        OpaqueFunction(function=_include_lidar),
    ])

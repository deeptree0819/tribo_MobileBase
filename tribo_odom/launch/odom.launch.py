#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_odom = get_package_share_directory("tribo_odom")
    pkg_bringup = get_package_share_directory("tribo_bringup")

    geom_file = LaunchConfiguration("geom_file")
    params_file = LaunchConfiguration("params_file")

    declare_geom = DeclareLaunchArgument(
        "geom_file",
        default_value=os.path.join(pkg_bringup, "config", "robot_geom.yaml"),
        description="Shared geometry parameters (track_width, wheel_radius, ticks_per_rev)"
    )

    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_odom, "config", "odom.yaml"),
        description="tribo_odom parameters"
    )

    odom_node = Node(
        package="tribo_odom",
        executable="odom_publisher",
        name="tribo_odom",
        output="screen",
        parameters=[geom_file, params_file],
    )

    return LaunchDescription([declare_geom, declare_params, odom_node])

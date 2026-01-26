#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_bringup = get_package_share_directory("tribo_bringup")
    pkg_odom = get_package_share_directory("tribo_odom")

    geom_file = LaunchConfiguration("geom_file")
    bringup_params = LaunchConfiguration("bringup_params")
    odom_params = LaunchConfiguration("odom_params")

    declare_geom = DeclareLaunchArgument(
        "geom_file",
        default_value=os.path.join(pkg_bringup, "config", "robot_geom.yaml"),
        description="Shared geometry parameters"
    )
    declare_bringup = DeclareLaunchArgument(
        "bringup_params",
        default_value=os.path.join(pkg_bringup, "config", "bringup.yaml"),
        description="Bringup params file"
    )
    declare_odom = DeclareLaunchArgument(
        "odom_params",
        default_value=os.path.join(pkg_odom, "config", "odom.yaml"),
        description="Odom params file"
    )

    bringup_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, "launch", "bringup.launch.py")),
        launch_arguments={"geom_file": geom_file, "params_file": bringup_params}.items(),
    )

    odom_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_odom, "launch", "odom.launch.py")),
        launch_arguments={"geom_file": geom_file, "params_file": odom_params}.items(),
    )

    return LaunchDescription([declare_geom, declare_bringup, declare_odom, bringup_inc, odom_inc])

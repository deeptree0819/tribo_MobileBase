#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_file = LaunchConfiguration("rviz_config")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    nav_share = get_package_share_directory("tribo_navigation")
    default_rviz_config = os.path.join(
        nav_share, "rviz", "map_building.rviz"
    )

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Full path to the RViz config file",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="map_view_rviz",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_rviz_config,
            rviz_node,
        ]
    )

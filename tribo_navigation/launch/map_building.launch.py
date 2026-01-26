#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('tribo_navigation')
    pkg_slam = get_package_share_directory('slam_toolbox')

    slam_launch_file = os.path.join(pkg_slam, 'launch', 'online_sync_launch.py')

    default_slam_params = os.path.join(pkg_nav, 'config', 'slam_toolbox_mapping.yaml')
    default_scan_filter_params = os.path.join(pkg_nav, 'config', 'scan_filter.yaml')
    default_rviz_cfg = os.path.join(pkg_nav, 'rviz', 'map_building.rviz')

    # ---- Launch args ----
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    use_scan_filter = LaunchConfiguration('use_scan_filter')
    scan_filter_params_file = LaunchConfiguration('scan_filter_params_file')
    scan_in_topic = LaunchConfiguration('scan_in_topic')
    scan_out_topic = LaunchConfiguration('scan_out_topic')

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam_params),

        DeclareLaunchArgument('use_scan_filter', default_value='true'),
        DeclareLaunchArgument('scan_filter_params_file', default_value=default_scan_filter_params),
        DeclareLaunchArgument('scan_in_topic', default_value='/scan'),
        DeclareLaunchArgument('scan_out_topic', default_value='/scan_filtered'),

        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_cfg),
    ]

    # ---- Laser scan filter: /scan -> /scan_filtered ----
    scan_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        output='screen',
        parameters=[scan_filter_params_file],
        remappings=[
            ('scan', scan_in_topic),
            ('scan_filtered', scan_out_topic),
        ],
        condition=IfCondition(use_scan_filter),
    )

    # ---- SLAM Toolbox (online sync) ----
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    # ---- RViz (optional) ----

    return LaunchDescription(
        declare + [
            scan_filter_node,
            slam_toolbox,
            
        ]
    )

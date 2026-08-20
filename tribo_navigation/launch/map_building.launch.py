#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
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
    # wrapper 는 share/ 가 아니라 lib/ 에 설치된다(setup.py 의 scripts/*.sh 규칙).
    default_rviz_launcher = os.path.join(
        get_package_prefix('tribo_navigation'), 'lib', 'tribo_navigation',
        'launch_rviz_on_lcd.sh',
    )

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
        DeclareLaunchArgument('rviz_display', default_value=':0'),
        DeclareLaunchArgument(
            'rviz_xauthority_glob',
            default_value='/run/user/1000/.mutter-Xwaylandauth.*',
        ),
        DeclareLaunchArgument('rviz_launcher', default_value=default_rviz_launcher),
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

    # ---- RViz 를 로봇 LCD 에 전체화면 표시 (optional) ----
    # bringup_launch.xml(Nav2)과 같은 방식이다. XAUTHORITY 는 mutter 가 매 부팅마다
    # 임의 접미사로 새로 만들기 때문에 wrapper 스크립트가 런타임에 글롭으로 해석한다.
    # 매핑 중에는 맵이 자라므로 화면에 맞추는 사전 계산(fit_rviz_to_map.py)을 쓸 수
    # 없다. map_building.rviz 의 고정 Scale 을 쓰고, 필요하면 LCD 에서 휠로 조정한다.
    rviz_lcd = ExecuteProcess(
        cmd=[
            LaunchConfiguration('rviz_launcher'),
            rviz_config,
            LaunchConfiguration('rviz_display'),
            LaunchConfiguration('rviz_xauthority_glob'),
        ],
        name='rviz2_lcd',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declare + [
            scan_filter_node,
            slam_toolbox,
            rviz_lcd,
        ]
    )

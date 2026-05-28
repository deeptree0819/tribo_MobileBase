#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, AndSubstitution, NotSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory("tribo_bringup")

    # --- arguments ---
    geom_file = LaunchConfiguration("geom_file")
    params_file = LaunchConfiguration("params_file")

    use_description = LaunchConfiguration("use_description")
    xacro_file = LaunchConfiguration("xacro_file")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # odom
    use_odom = LaunchConfiguration("use_odom")

    # ekf (sensor fusion: encoder odom + IMU)
    use_ekf = LaunchConfiguration("use_ekf")
    ekf_params_file = LaunchConfiguration("ekf_params_file")

    # lidar
    use_lidar = LaunchConfiguration("use_lidar")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")

    declare_geom = DeclareLaunchArgument(
        "geom_file",
        default_value=os.path.join(pkg_bringup, "config", "robot_geom.yaml"),
        description="Shared geometry parameters (track_width, wheel_radius, ticks_per_rev)",
    )

    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_bringup, "config", "bringup.yaml"),
        description="tribo_bringup parameters",
    )

    declare_use_description = DeclareLaunchArgument(
        "use_description",
        default_value="true",
        description="Start robot_state_publisher with robot_description (URDF upload)",
    )

    declare_xacro = DeclareLaunchArgument(
        "xacro_file",
        default_value=os.path.join(
            get_package_share_directory("tribo_description"),
            "urdf",
            "robot.urdf.xacro",
        ),
        description="Xacro file to generate robot_description",
    )

    declare_use_jsp = DeclareLaunchArgument(
        "use_joint_state_publisher",
        default_value="true",
        description="Start joint_state_publisher",
    )

    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulated time",
    )

    declare_use_odom = DeclareLaunchArgument(
        "use_odom",
        default_value="true",
        description="Start tribo_odom odom_publisher (/odom + TF odom->base_link)",
    )

    declare_use_ekf = DeclareLaunchArgument(
        "use_ekf",
        default_value="true",
        description="Fuse odom+IMU via robot_localization EKF. ON: odom_publisher -> /odom_raw "
                    "(TF off), ekf_node -> /odom + TF. OFF: odom_publisher -> /odom + TF (legacy).",
    )
    declare_ekf_params = DeclareLaunchArgument(
        "ekf_params_file",
        default_value=os.path.join(pkg_bringup, "param", "ekf.yaml"),
        description="EKF parameter file (robot_localization)",
    )

    declare_use_lidar = DeclareLaunchArgument(
        "use_lidar",
        default_value="true",
        description="Start sllidar_ros2 (publishes /scan)",
    )
    declare_lidar_frame = DeclareLaunchArgument(
        "lidar_frame_id",
        default_value="laser_link",
        description="LiDAR frame_id (must exist in TF, e.g., base_link->laser_link in URDF)",
    )

    # --- nodes ---
    bringup_node = Node(
        package="tribo_bringup",
        executable="bringup",
        name="tribo_bringup",
        output="screen",
        parameters=[geom_file, params_file],
    )

    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=IfCondition(use_joint_state_publisher),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        condition=IfCondition(use_description),
        parameters=[
            {"use_sim_time": use_sim_time,
             "robot_description": Command(['xacro', ' ', xacro_file])}
        ],
    )

    # Odom node — 공통 파라미터 (track 보정, sign 등)
    _odom_common_params = {
        "encoder_topic": "encoder_raw",
        "odom_frame": "odom",
        "base_frame": "base_link",
        "invert_left": False,
        "invert_right": False,
        "invert_translation": False,
        "invert_rotation": True,
        # effective (slip-calibrated) track for odom yaw; overrides physical 0.52 in robot_geom.yaml
        "track_width": 0.735,
    }

    # EKF 사용 시: odom_publisher → /odom_raw (TF 끔). EKF가 /odom + TF 담당.
    odom_node_with_ekf = Node(
        package="tribo_odom",
        executable="odom_publisher",
        name="tribo_odom",
        output="screen",
        condition=IfCondition(AndSubstitution(use_odom, use_ekf)),
        parameters=[
            geom_file,
            {**_odom_common_params,
             "output_topic": "odom_raw",
             "publish_tf": False},
        ],
    )

    # EKF 미사용(legacy): odom_publisher → /odom + TF (이전 동작 유지)
    odom_node_no_ekf = Node(
        package="tribo_odom",
        executable="odom_publisher",
        name="tribo_odom",
        output="screen",
        condition=IfCondition(AndSubstitution(use_odom, NotSubstitution(use_ekf))),
        parameters=[
            geom_file,
            {**_odom_common_params,
             "output_topic": "odom",
             "publish_tf": True},
        ],
    )

    # EKF (robot_localization): /odom_raw + /imu/data → 융합 → /odom + TF(odom->base_link)
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        condition=IfCondition(use_ekf),
        parameters=[ekf_params_file],
        remappings=[("odometry/filtered", "odom")],
    )

    sllidar_share = get_package_share_directory("sllidar_ros2")
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_share, "launch", "sllidar_c1_launch.py")
        ),
        condition=IfCondition(use_lidar),
        launch_arguments={
            "frame_id": lidar_frame_id,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_geom,
            declare_params,
            declare_use_description,
            declare_xacro,
            declare_use_jsp,
            declare_sim_time,

            declare_use_odom,
            declare_use_ekf,
            declare_ekf_params,
            declare_use_lidar,
            declare_lidar_frame,

            bringup_node,
            odom_node_with_ekf,
            odom_node_no_ekf,
            ekf_node,
            lidar_launch,
            joint_state_pub,
            robot_state_pub,
        ]
    )

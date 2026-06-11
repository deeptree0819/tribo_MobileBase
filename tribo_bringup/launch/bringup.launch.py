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


def _resolve_port(preferred, *by_id_substrings):
    """Pick a stable serial-device path that survives USB re-plugging / ttyUSB
    renumbering.

    Prefers the udev symlink (see tribo_bringup/udev/99-tribo-serial.rules,
    e.g. /dev/tribo_base, /dev/tribo_lidar). If that symlink is not present
    (rule not installed yet, or a freshly flashed robot), falls back to the
    matching /dev/serial/by-id entry. Both are device-identity based, never the
    unstable /dev/ttyUSBN number.
    """
    if os.path.exists(preferred):
        return preferred
    by_id_dir = "/dev/serial/by-id"
    try:
        for name in sorted(os.listdir(by_id_dir)):
            if any(s in name for s in by_id_substrings):
                return os.path.join(by_id_dir, name)
    except OSError:
        pass
    return preferred  # nothing found; node will report a clear "no such device"


# Base board (Yahboom STM32, CH340) and LiDAR (Silicon Labs CP2102N) serial ports.
# udev symlink first, by-id fallback. They have distinct VID:PID, so they can
# never be confused even if the USB socket or ttyUSB order changes.
BASE_SERIAL_PORT = _resolve_port("/dev/tribo_base", "1a86_USB_Serial")
LIDAR_SERIAL_PORT = _resolve_port(
    "/dev/tribo_lidar", "c4c4102ee863ef1196dcdaa9c169b110", "CP2102N"
)


# Node process names (/proc/<pid>/comm) that THIS launch starts. comm is
# truncated to 15 chars by the kernel, so the long publishers appear cut.
# Matching on comm (not cmdline) means the running 'ros2 launch' process
# (comm 'ros2'/'python3') is never matched -> we can't kill ourselves.
BRINGUP_NODE_COMMS = {
    "bringup",          # tribo_bringup base-board driver (holds base serial)
    "sllidar_node",     # lidar driver (holds lidar serial)
    "odom_publisher",   # tribo_odom  -> /odom_raw or /odom + TF
    "ekf_node",         # robot_localization -> /odom + TF
    "joint_state_pub",  # joint_state_publisher (comm truncated)
    "robot_state_pub",  # robot_state_publisher (comm truncated)
}


def _clean_stale_nodes():
    """Kill leftover nodes from a previous bringup before this one starts.

    A hard-killed or terminal-closed bringup orphans its child nodes. On the
    next launch those orphans cause two failure modes:
      1) serial: the base-board / lidar port is still held -> 'multiple access
         on port' / lidar 'OPERATION_TIMEOUT'.
      2) duplicate ROS nodes: a second odom_publisher / ekf_node keeps
         publishing /odom and the odom->base_link TF, silently corrupting
         odometry even though nothing errors out.

    We scan /proc and SIGKILL a process if it either (a) holds one of our exact
    serial devices, or (b) has a comm in BRINGUP_NODE_COMMS. comm-matching never
    hits this launch process (comm 'ros2'/'python3'), and the new nodes are not
    spawned until generate_launch_description() returns, so only OLD instances
    match. No fuser/pkill, no cmdline pattern matching. Disable with
    TRIBO_AUTOCLEAN=0; this assumes bringup is the sole owner of these nodes
    (e.g. a separate slam/nav launch should not be relying on bringup's
    robot_state_publisher staying up across a bringup restart).
    """
    if os.environ.get("TRIBO_AUTOCLEAN", "1") == "0":
        return

    import signal
    import time

    ports = set()
    for p in (BASE_SERIAL_PORT, LIDAR_SERIAL_PORT):
        try:
            ports.add(os.path.realpath(p))
        except OSError:
            pass

    mypid = os.getpid()
    killed = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == mypid:
            continue

        match = False
        # (b) comm name
        try:
            with open(os.path.join("/proc", pid, "comm")) as f:
                if f.read().strip() in BRINGUP_NODE_COMMS:
                    match = True
        except OSError:
            pass

        # (a) holds one of our serial devices
        if not match and ports:
            fd_dir = os.path.join("/proc", pid, "fd")
            try:
                for fd in os.listdir(fd_dir):
                    try:
                        if os.readlink(os.path.join(fd_dir, fd)) in ports:
                            match = True
                            break
                    except OSError:
                        continue
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                pass

        if match:
            try:
                os.kill(int(pid), signal.SIGKILL)
                killed.append(pid)
            except (ProcessLookupError, PermissionError):
                pass

    if killed:
        print(f"[tribo_bringup] auto-clean: killed stale node pid(s) {killed}")
        time.sleep(1.0)  # let the OS release serial fds / node names


def generate_launch_description():
    # Clean up a previous (hard-killed) bringup before any node starts, so a
    # re-launch neither hits a held serial port nor leaves duplicate odom/TF
    # nodes. Runs synchronously here, before the launch service spawns nodes.
    _clean_stale_nodes()

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
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")

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
    declare_lidar_port = DeclareLaunchArgument(
        "lidar_serial_port",
        default_value=LIDAR_SERIAL_PORT,
        description="LiDAR serial port (by-id, so it never collides with the base board)",
    )

    # --- nodes ---
    bringup_node = Node(
        package="tribo_bringup",
        executable="bringup",
        name="tribo_bringup",
        output="screen",
        # {"port": ...} last -> overrides bringup.yaml so the resolved udev/by-id
        # base port is authoritative (mirrors how the lidar port is passed).
        parameters=[geom_file, params_file, {"port": BASE_SERIAL_PORT}],
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
        # Board reports NEGATIVE encoder ticks for physical forward motion since the
        # invert_cmd_vel fix (commit 0535b1b). Flip translation so odom integrates
        # forward; rotation stays on invert_rotation (verified on robot 2026-06-10).
        "invert_translation": True,
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
            "serial_port": lidar_serial_port,
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
            declare_lidar_port,

            bringup_node,
            odom_node_with_ekf,
            odom_node_no_ekf,
            ekf_node,
            lidar_launch,
            joint_state_pub,
            robot_state_pub,
        ]
    )

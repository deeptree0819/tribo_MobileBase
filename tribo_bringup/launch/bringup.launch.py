#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
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
    "odom_publisher",   # tribo_odom  -> /odom + TF
    "joint_state_pub",  # joint_state_publisher (comm truncated)
    "robot_state_pub",  # robot_state_publisher (comm truncated)
}


def _clean_stale_nodes():
    """Kill leftover nodes from a previous bringup before this one starts.

    A hard-killed or terminal-closed bringup orphans its child nodes. On the
    next launch those orphans cause two failure modes:
      1) serial: the base-board / lidar port is still held -> 'multiple access
         on port' / lidar 'OPERATION_TIMEOUT'.
      2) duplicate ROS nodes: a second odom_publisher keeps publishing /odom
         and the odom->base_link TF, silently corrupting odometry even though
         nothing errors out.

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
    # Per-robot motor gain override (motor_calib.py / motor_calib_converge.sh
    # auto-generates it, .gitignore'd). Loaded AFTER bringup.yaml so its
    # gain_m1~m4 override the shared defaults; ROS2 applies later param files on
    # top of earlier ones. The path is the install-share copy (same as
    # bringup.yaml) — motor_calib.py writes the SOURCE config, and a colcon build
    # copies it here (install config is a separate copy, not symlinked). If the
    # file does not exist yet (never calibrated), it is simply skipped.
    bringup_params = [geom_file, params_file]
    motor_calib_file = os.path.join(pkg_bringup, "config", "motor_calib.yaml")
    if os.path.exists(motor_calib_file):
        bringup_params.append(motor_calib_file)
    # {"port": ...} last -> overrides bringup.yaml so the resolved udev/by-id
    # base port is authoritative (mirrors how the lidar port is passed).
    bringup_params.append({"port": BASE_SERIAL_PORT})

    bringup_node = Node(
        package="tribo_bringup",
        executable="bringup",
        name="tribo_bringup",
        output="screen",
        parameters=bringup_params,
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
        # effective (slip-calibrated) track for odom yaw; overrides physical track in
        # robot_geom.yaml. 4-wheel skid-steer slips laterally on turns, so effective
        # track > physical 0.70 m. This is the SOLE yaw source now (IMU/EKF removed
        # 2026-07-15: board IMU gyro dies ~2.4 s into a sustained turn), so /odom yaw
        # accuracy depends entirely on this value.
        # 2026-07-13: 0.873 -> 0.838 (tribo v2, motor_calib.yaml 적용 후 12V에서 2회 재현).
        # 2026-07-18: 0.838 -> 0.939. wheel_radius 를 0.040->0.0448 (×1.12) 올리자 회전
        # odom 이 정확히 그만큼 부풀려졌다(회전=(d_r-d_l)/track, d∝R). 직진은 R 만으로
        # 결정돼 0.0448 이 맞고, 회전 복구는 track 을 같은 배수로: 0.838×1.12=0.939.
        # (R 은 곱으로만 작용 → track 도 함께 스케일해야 R/track 비율이 유지됨.)
        # 2026-07-18: 라이다-독립 검증(제자리 6바퀴, 출발선 복귀 오차)로 track 확정.
        #   0.939→90° undershoot, 0.959→80° overshoot, 0.981→100° overshoot.
        #   제로크로싱은 0.939~0.959 사이 → 0.950 확정(보간 0.949, 3역산 평균 0.948).
        #   ⚠️ 눈측정 한계로 유효 분해능 ~±3%(0.92~0.98 구분 불가). 이 이상 튜닝 무의미.
        #   ⚠️ SHARED 값: 이 캘리브는 기체 76c02a 에서 함. 기체별 track 은 다를 수 있고
        #      (7b6a 미확정), per-unit override 경로가 아직 없음 → 다른 기체는 이 방법으로 재검증.
        # 2026-07-29: 실측 82mm 메카넘 교체 → 0.950 -> 1.096 (기체 76c02a).
        #   제자리 2바퀴법: odom 738.1° vs 물리 640° → 0.950 × (738.1/640) = 1.096.
        #   ⚠️ 방향이 직관과 반대다. 메카넘은 스크럽 마찰이 줄어 회전이 가벼워지지만,
        #      롤러가 옆으로 자유롭게 구른다는 건 곧 횡방향 접지력이 없다는 뜻이라
        #      같은 바퀴 이동량에 로봇은 "덜" 돈다 → 유효 track 은 오히려 커진다.
        #      슬립비 1.34(=0.950/0.71) → 1.54(=1.096/0.71). 마찰 감소와 접지력 감소는
        #      같이 온다. 고무바퀴로 되돌리면 0.950 으로 함께 되돌릴 것.
        "track_width": 1.096,
    }

    # odom_publisher → /odom + TF(odom->base_link). 휠 오도메트리 단일 경로.
    odom_node = Node(
        package="tribo_odom",
        executable="odom_publisher",
        name="tribo_odom",
        output="screen",
        condition=IfCondition(use_odom),
        parameters=[
            geom_file,
            {**_odom_common_params,
             "output_topic": "odom",
             "publish_tf": True},
        ],
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
            declare_use_lidar,
            declare_lidar_frame,
            declare_lidar_port,

            bringup_node,
            odom_node,
            lidar_launch,
            joint_state_pub,
            robot_state_pub,
        ]
    )

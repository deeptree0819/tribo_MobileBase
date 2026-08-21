"""Gazebo sim 전용 Nav2 런치.

tribo_navigation/config/nav2_params.yaml (실로봇과 공유) 위에
tribo_gazebo/params/nav2_sim_overlay.yaml (sim 전용) 을 깊이 병합해
임시 파일로 쓴 뒤, 그 파일로 tribo_navigation 의 bringup_launch.xml 을 띄운다.

핵심은 공유 파일을 건드리지 않는 것이다. 오버레이는 tribo_gazebo 안에만
있으므로 여기를 아무리 튜닝해도 실로봇 주행 설정은 그대로다.

쓰는 법
  ros2 launch tribo_gazebo nav_sim.launch.py map:=/path/to/map.yaml

병합 결과는 --merged-out 경로에 남으므로 그대로 열어 확인할 수 있다.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _deep_merge(base, overlay):
    """overlay 를 base 위에 깊이 병합한 새 dict 를 돌려준다.

    dict 끼리만 재귀 병합하고 그 외(리스트/스칼라)는 overlay 로 통째 교체한다.
    Nav2 의 plugins 같은 리스트는 부분 병합이 의미가 없어서 교체가 맞다.
    """
    if not isinstance(base, dict) or not isinstance(overlay, dict):
        return overlay
    merged = dict(base)
    for key, value in overlay.items():
        if key in merged:
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _make_merged_params(context, *args, **kwargs):
    base_file = LaunchConfiguration("base_params_file").perform(context)
    overlay_file = LaunchConfiguration("overlay_params_file").perform(context)
    merged_out = LaunchConfiguration("merged_params_out").perform(context)

    base = _load_yaml(base_file)
    overlay = _load_yaml(overlay_file)
    merged = _deep_merge(base, overlay)

    with open(merged_out, "w", encoding="utf-8") as handle:
        handle.write(
            "# 자동 생성 파일 - 직접 고치지 말 것.\n"
            "# 생성  tribo_gazebo/launch/nav_sim.launch.py\n"
            f"# base    {base_file}\n"
            f"# overlay {overlay_file}\n"
        )
        yaml.safe_dump(merged, handle, default_flow_style=False, sort_keys=False,
                       allow_unicode=True)

    # 오버레이가 실제로 무엇을 바꿨는지 런치 로그에 남긴다. 빈 오버레이면
    # "no-op" 이라고 분명히 찍혀야 나중에 원인 추적이 된다.
    changed = _overlay_keys(overlay)
    summary = ", ".join(changed) if changed else "(없음 - base 그대로)"

    return [
        LogInfo(msg=f"[nav_sim] sim 오버레이 적용: {summary}"),
        LogInfo(msg=f"[nav_sim] 병합 결과: {merged_out}"),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("tribo_navigation"),
                    "launch",
                    "bringup_launch.xml",
                )
            ),
            launch_arguments={
                "params_file": merged_out,
                "map": LaunchConfiguration("map"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "set_initial_pose": LaunchConfiguration("set_initial_pose"),
                "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
            }.items(),
        ),
    ]


def _overlay_keys(node, prefix=""):
    """오버레이에서 실제 값을 덮어쓴 리프 키 경로 목록."""
    keys = []
    if not isinstance(node, dict):
        return [prefix]
    for key, value in node.items():
        path = f"{prefix}.{key}" if prefix else key
        keys.extend(_overlay_keys(value, path))
    return keys


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("tribo_gazebo")
    pkg_nav = get_package_share_directory("tribo_navigation")

    return LaunchDescription([
        DeclareLaunchArgument(
            "base_params_file",
            default_value=os.path.join(pkg_nav, "config", "nav2_params.yaml"),
            description="실로봇과 공유하는 Nav2 파라미터. 여기를 고치면 실주행에 영향이 간다.",
        ),
        DeclareLaunchArgument(
            "overlay_params_file",
            default_value=os.path.join(pkg_gazebo, "params", "nav2_sim_overlay.yaml"),
            description="sim 전용 델타. 여기를 고치는 것은 실주행에 영향이 없다.",
        ),
        DeclareLaunchArgument(
            "merged_params_out",
            default_value=os.path.join(tempfile.gettempdir(), "tribo_nav2_sim_merged.yaml"),
            description="병합 결과가 쓰이는 경로. 확인용으로 열어보면 된다.",
        ),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # bringup_launch.xml 의 rviz 는 로봇 LCD 를 겨냥한 wrapper 스크립트라
        # PC sim 에서는 뜨지 않는다. sim 기본값은 끄고 RViz 는 따로 띄운다.
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("set_initial_pose", default_value="true"),
        DeclareLaunchArgument("initial_pose_x", default_value="-3.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        OpaqueFunction(function=_make_merged_params),
    ])

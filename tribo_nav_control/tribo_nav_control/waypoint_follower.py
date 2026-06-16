#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""여러 웨이포인트를 순차 주행하는 Nav2 제어 노드.

nav2_simple_commander 의 BasicNavigator(followWaypoints) 를 사용한다.
- 파라미터 'waypoints' = [x1, y1, yaw1, x2, y2, yaw2, ...] (3개씩 한 점)
- 진행 중인 웨이포인트 인덱스 피드백 출력
- nav_timeout 초과 시 자동 취소
- 결과 보고 후 종료
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_to_quaternion(yaw: float):
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class WaypointFollower(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_follower")

        # [x, y, yaw] 가 3개 단위로 반복. 기본값은 빈 리스트.
        self.declare_parameter("waypoints", [0.0])
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("wait_for_nav2", True)
        self.declare_parameter("nav_timeout", 300.0)
        self.declare_parameter("feedback_period", 2.0)

        raw = list(self.get_parameter("waypoints").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.wait_for_nav2 = bool(self.get_parameter("wait_for_nav2").value)
        self.nav_timeout = float(self.get_parameter("nav_timeout").value)
        self.feedback_period = float(self.get_parameter("feedback_period").value)

        self.waypoints = self._parse_waypoints(raw)
        self.navigator = BasicNavigator()

    def _parse_waypoints(self, raw):
        """평탄 리스트 -> PoseStamped 목록. 3의 배수가 아니면 빈 목록."""
        if len(raw) < 3 or len(raw) % 3 != 0:
            return []
        poses = []
        for i in range(0, len(raw), 3):
            x, y, yaw = float(raw[i]), float(raw[i + 1]), float(raw[i + 2])
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = self.navigator.get_clock().now().to_msg() \
                if hasattr(self, "navigator") else self.get_clock().now().to_msg()
            ps.pose.position.x = x
            ps.pose.position.y = y
            qz, qw = yaw_to_quaternion(yaw)
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            poses.append(ps)
        return poses

    def run(self) -> int:
        if not self.waypoints:
            self.get_logger().error(
                "유효한 waypoints 가 없습니다. "
                "[x1,y1,yaw1, x2,y2,yaw2, ...] (3의 배수) 형식으로 지정하세요."
            )
            return 2

        if self.wait_for_nav2:
            self.get_logger().info("Nav2 활성화 대기 중...")
            self.navigator.waitUntilNav2Active()
            self.get_logger().info("Nav2 활성화 확인됨.")

        self.get_logger().info(f"{len(self.waypoints)}개 웨이포인트 주행 시작.")
        self.navigator.followWaypoints(self.waypoints)

        last_idx = -1
        last_log = 0.0
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback is not None:
                idx = feedback.current_waypoint
                now = self.get_clock().now().nanoseconds * 1e-9
                if idx != last_idx or now - last_log >= self.feedback_period:
                    last_idx = idx
                    last_log = now
                    self.get_logger().info(
                        f"진행 중 - 웨이포인트 {idx + 1}/{len(self.waypoints)}"
                    )

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("모든 웨이포인트 완료 (SUCCEEDED).")
            return 0
        if result == TaskResult.CANCELED:
            self.get_logger().warn("주행이 취소됨 (CANCELED).")
            return 1
        self.get_logger().error(f"주행 실패 ({result}).")
        return 2


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointFollower()
    exit_code = 0
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl-C 감지 - 주행 취소 후 종료.")
        try:
            node.navigator.cancelTask()
        except Exception:
            pass
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()

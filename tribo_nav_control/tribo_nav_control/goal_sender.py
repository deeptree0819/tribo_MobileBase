#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""단일 목표(goal pose)로 Nav2 주행을 프로그래밍 제어하는 노드.

nav2_simple_commander 의 BasicNavigator(goToPose) 를 사용한다.
- 파라미터로 목표 (x, y, yaw) 지정
- 피드백(남은 거리/경과 시간) 출력
- nav_timeout 초과 시 자동 취소
- 결과(SUCCEEDED/CANCELED/FAILED) 보고 후 종료
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_to_quaternion(yaw: float):
    """Z축 회전 yaw(rad) -> (z, w) 쿼터니언 성분."""
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class GoalSender(Node):
    def __init__(self) -> None:
        super().__init__("goal_sender")

        # 목표 좌표 (map 프레임 기준)
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("frame_id", "map")
        # Nav2 활성화 대기 여부 (이미 떠 있으면 빠르게 통과)
        self.declare_parameter("wait_for_nav2", True)
        # 주행 제한 시간(초). 0 이하면 무제한.
        self.declare_parameter("nav_timeout", 120.0)
        # 피드백 출력 주기(초)
        self.declare_parameter("feedback_period", 2.0)

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = float(self.get_parameter("yaw").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.wait_for_nav2 = bool(self.get_parameter("wait_for_nav2").value)
        self.nav_timeout = float(self.get_parameter("nav_timeout").value)
        self.feedback_period = float(self.get_parameter("feedback_period").value)

        self.navigator = BasicNavigator()

    def build_goal(self) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = self.x
        goal.pose.position.y = self.y
        goal.pose.position.z = 0.0
        qz, qw = yaw_to_quaternion(self.yaw)
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        return goal

    def run(self) -> int:
        """주행 실행. 종료 코드(0=성공) 반환."""
        if self.wait_for_nav2:
            self.get_logger().info("Nav2 활성화 대기 중 (waitUntilNav2Active)...")
            self.navigator.waitUntilNav2Active()
            self.get_logger().info("Nav2 활성화 확인됨.")

        goal = self.build_goal()
        self.get_logger().info(
            f"목표 전송: x={self.x:.3f}, y={self.y:.3f}, "
            f"yaw={self.yaw:.3f} rad (frame={self.frame_id})"
        )
        self.navigator.goToPose(goal)

        last_feedback_log = 0.0
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback is not None:
                elapsed = feedback.navigation_time.sec + \
                    feedback.navigation_time.nanosec * 1e-9
                if elapsed - last_feedback_log >= self.feedback_period:
                    last_feedback_log = elapsed
                    self.get_logger().info(
                        f"주행 중 - 남은 거리: "
                        f"{feedback.distance_remaining:.2f} m, "
                        f"경과: {elapsed:.1f} s"
                    )
                if self.nav_timeout > 0.0 and elapsed > self.nav_timeout:
                    self.get_logger().warn(
                        f"제한 시간({self.nav_timeout:.0f}s) 초과 - 주행 취소."
                    )
                    self.navigator.cancelTask()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("목표 도달 성공 (SUCCEEDED).")
            return 0
        if result == TaskResult.CANCELED:
            self.get_logger().warn("주행이 취소됨 (CANCELED).")
            return 1
        if result == TaskResult.FAILED:
            self.get_logger().error("주행 실패 (FAILED).")
            return 2
        self.get_logger().error(f"알 수 없는 결과: {result}")
        return 3


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalSender()
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

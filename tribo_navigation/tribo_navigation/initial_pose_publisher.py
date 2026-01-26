#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose_publisher")

        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("cov_x", 0.25)
        self.declare_parameter("cov_y", 0.25)
        self.declare_parameter("cov_yaw", 0.0685389)
        self.declare_parameter("publish_count", 5)
        self.declare_parameter("publish_period", 0.5)
        self.declare_parameter("initial_delay", 2.0)
        self.declare_parameter("wait_for_subscriber", True)
        self.declare_parameter("use_sim_time", False)

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = float(self.get_parameter("yaw").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.cov_x = float(self.get_parameter("cov_x").value)
        self.cov_y = float(self.get_parameter("cov_y").value)
        self.cov_yaw = float(self.get_parameter("cov_yaw").value)
        self.remaining = int(self.get_parameter("publish_count").value)
        self.publish_period = float(self.get_parameter("publish_period").value)
        self.initial_delay = float(self.get_parameter("initial_delay").value)
        self.wait_for_subscriber = bool(
            self.get_parameter("wait_for_subscriber").value
        )
        self.warned_no_subscriber = False

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", qos)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self.publish_period, self._on_timer)

        self.get_logger().info(
            "Initial pose publisher started "
            f"(x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f} rad)."
        )

    def _on_timer(self) -> None:
        if self.remaining <= 0:
            self.get_logger().info("Initial pose published. Shutting down.")
            self.timer.cancel()
            rclpy.shutdown()
            return

        now = self.get_clock().now()
        if (now - self.start_time).nanoseconds * 1e-9 < self.initial_delay:
            return

        if self.wait_for_subscriber and self.pub.get_subscription_count() == 0:
            if not self.warned_no_subscriber:
                self.get_logger().info("Waiting for /initialpose subscriber...")
                self.warned_no_subscriber = True
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        cov = [0.0] * 36
        cov[0] = self.cov_x
        cov[7] = self.cov_y
        cov[35] = self.cov_yaw
        msg.pose.covariance = cov

        self.pub.publish(msg)
        self.warned_no_subscriber = False
        self.remaining -= 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

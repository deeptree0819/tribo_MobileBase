#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import time
import math


class EncoderCalibTest(Node):
    def __init__(self):
        super().__init__("encoder_calib_test")

        # ---------- parameters ----------
        self.declare_parameter("mode", "straight")  # straight | rotate
        self.declare_parameter("linear_vel", 0.2)  # m/s
        self.declare_parameter("angular_vel", 0.5) # rad/s
        self.declare_parameter("duration", 5.0)    # seconds

        self.mode = self.get_parameter("mode").value
        self.linear_vel = float(self.get_parameter("linear_vel").value)
        self.angular_vel = float(self.get_parameter("angular_vel").value)
        self.duration = float(self.get_parameter("duration").value)

        # ---------- ros I/O ----------
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.sub_enc = self.create_subscription(
            Int32MultiArray,
            "encoder_raw",
            self.cb_enc,
            50
        )

        self.start_ticks = None
        self.end_ticks = None
        self.latest_ticks = None

        self.start_time = None
        self.timer = self.create_timer(0.02, self.loop)

        self.get_logger().info(
            f"EncoderCalibTest started | mode={self.mode}, "
            f"linear_vel={self.linear_vel}, angular_vel={self.angular_vel}, "
            f"duration={self.duration}s"
        )

    def cb_enc(self, msg: Int32MultiArray):
        if len(msg.data) < 5:
            return
        # data = [ms, e1, e2, e3, e4]
        # array.array → list 변환
        self.latest_ticks = list(msg.data[1:5])

    def loop(self):
        now = time.time()

        # wait encoder
        if self.latest_ticks is None:
            return

        # init
        if self.start_time is None:
            self.start_time = now
            self.start_ticks = self.latest_ticks[:]   # 리스트 얕은 복사
            self.get_logger().info(f"START ticks = {self.start_ticks}")


        elapsed = now - self.start_time

        # send cmd
        cmd = Twist()
        if elapsed < self.duration:
            if self.mode == "straight":
                cmd.linear.x = self.linear_vel
            elif self.mode == "rotate":
                cmd.angular.z = self.angular_vel
            self.pub_cmd.publish(cmd)
        else:
            # stop
            self.pub_cmd.publish(Twist())
            self.end_ticks = self.latest_ticks[:]
            self.print_result()
            rclpy.shutdown()

    def print_result(self):
        e1s, e2s, e3s, e4s = self.start_ticks
        e1e, e2e, e3e, e4e = self.end_ticks

        d1 = e1e - e1s
        d2 = e2e - e2s
        d3 = e3e - e3s
        d4 = e4e - e4s

        left = (d1 + d2) / 2.0
        right = (d3 + d4) / 2.0

        self.get_logger().info("===== ENCODER CALIB RESULT =====")
        self.get_logger().info(f"Mode        : {self.mode}")
        self.get_logger().info(f"Duration    : {self.duration:.2f} s")
        self.get_logger().info(f"dTicks L    : {left:.1f}")
        self.get_logger().info(f"dTicks R    : {right:.1f}")
        self.get_logger().info(f"dTicks diff : {(right - left):.1f}")

        if self.mode == "rotate":
            self.get_logger().info(
                "Rotation check: L and R should have opposite signs"
            )
        else:
            self.get_logger().info(
                "Straight check: L and R should have same sign & similar magnitude"
            )

        self.get_logger().info("================================")


def main():
    rclpy.init()
    node = EncoderCalibTest()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

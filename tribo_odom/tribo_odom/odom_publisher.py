#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class TriboOdom(Node):
    """
    /encoder_raw: Int32MultiArray data=[ms, e1, e2, e3, e4] (누적 tick)
      - e1=m1(FL), e2=m2(RL) -> LEFT
      - e3=m3(RR), e4=m4(FR) -> RIGHT

    출력:
      - /odom (nav_msgs/Odometry)
      - TF: odom -> base_link (또는 base_frame 파라미터 값)
    """

    def __init__(self):
        super().__init__("tribo_odom")

        # ----- 파라미터 선언 -----
        self.declare_parameter("encoder_topic", "encoder_raw")  # bringup에서 퍼블리시하는 토픽 이름
        self.declare_parameter("output_topic", "odom")          # 출력 토픽 (/odom + TF 단일 경로)
        self.declare_parameter("wheel_radius", 0.04)            # m
        self.declare_parameter("ticks_per_rev", 4320)           # 한 바퀴당 tick 수
        self.declare_parameter("track_width", 0.397)             # m (좌우 바퀴 사이 거리)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        # 좌/우 엔코더가 반대로 들어가는 경우용
        self.declare_parameter("invert_left", True)
        self.declare_parameter("invert_right", True)
        self.declare_parameter("invert_translation", False)
        self.declare_parameter("invert_rotation", False)
        self.declare_parameter("yaw_offset", 0.0)
        self.invert_translation = bool(self.get_parameter("invert_translation").value)
        self.invert_rotation    = bool(self.get_parameter("invert_rotation").value)

        # ----- 파라미터 읽기 -----
        self.encoder_topic = str(self.get_parameter("encoder_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)

        self.R = float(self.get_parameter("wheel_radius").value)
        self.tpr = int(self.get_parameter("ticks_per_rev").value)
        self.track = float(self.get_parameter("track_width").value)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.inv_left = -1.0 if bool(self.get_parameter("invert_left").value) else 1.0
        self.inv_right = -1.0 if bool(self.get_parameter("invert_right").value) else 1.0
        self.yaw_offset = float(self.get_parameter("yaw_offset").value)


        # ----- ROS 통신 -----
        self.sub = self.create_subscription(
            Int32MultiArray,
            self.encoder_topic,
            self.cb_enc,
            50
        )
        self.pub = self.create_publisher(Odometry, self.output_topic, 50)
        self.tfbr = TransformBroadcaster(self)

        # ----- 상태 변수 -----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev = None        # (left_ticks, right_ticks)
        self.prev_stamp = None  # rclpy.time.Time

        self.get_logger().info(
            f"TriboOdom started. encoder_topic={self.encoder_topic}, output_topic={self.output_topic}, "
            f"R={self.R:.3f} m, tpr={self.tpr}, track={self.track:.3f} m, "
            f"invert_left={self.inv_left < 0}, invert_right={self.inv_right < 0}, "
            f"publish_tf={self.publish_tf}, yaw_offset={self.yaw_offset:.3f} rad"
        )

    # ---------- 유틸 함수 ----------
    @staticmethod
    def _wrap(a: float) -> float:
        """yaw를 [-pi, pi] 범위로 래핑"""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _ticks_to_dist(self, dticks: float) -> float:
        """tick 변화량 → 선형 이동거리(m)"""
        return (dticks / float(self.tpr)) * (2.0 * math.pi * self.R)

    def _yaw_to_quat(self, yaw: float):
        """2D yaw → quaternion(z축 회전만)"""
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    # ---------- 콜백 ----------
    def cb_enc(self, msg: Int32MultiArray):
        # data=[ms,e1,e2,e3,e4]  (누적 tick)
        if len(msg.data) < 5:
            return

        ms = int(msg.data[0])
        e1, e2, e3, e4 = map(int, msg.data[1:5])

        # 좌/우 평균 tick (m1, m2 → left / m3, m4 → right)
        left_now = (e1 + e2) / 2.0
        right_now = (e3 + e4) / 2.0

        stamp = self.get_clock().now()

        # 첫 샘플은 baseline으로만 사용
        if self.prev is None:
            self.prev = (left_now, right_now)
            self.prev_stamp = stamp
            return

        dt = (stamp - self.prev_stamp).nanoseconds * 1e-9
        if dt <= 1e-4:
            return

        left_prev, right_prev = self.prev
        dL_ticks = left_now - left_prev
        dR_ticks = right_now - right_prev

        # 이전 값 갱신
        self.prev = (left_now, right_now)
        self.prev_stamp = stamp

        # 좌/우 방향 반전 옵션 적용
        dL_ticks *= self.inv_left
        dR_ticks *= self.inv_right

        # tick → 거리
        dL = self._ticks_to_dist(dL_ticks)
        dR = self._ticks_to_dist(dR_ticks)

        # 로봇 기준 진행 거리, yaw 변화
        ds = (dR + dL) / 2.0
        dyaw = (dR - dL) / max(self.track, 1e-6)

        if self.invert_translation:
            ds = -ds
        if self.invert_rotation:
            dyaw = -dyaw
            
        # midpoint integration
        yaw_mid = self.yaw + dyaw * 0.5
        self.x += ds * math.cos(yaw_mid)
        self.y += ds * math.sin(yaw_mid)
        self.yaw = self._wrap(self.yaw + dyaw)
        yaw_out = self._wrap(self.yaw + self.yaw_offset)

        vx = ds / dt
        wz = dyaw / dt

        qx, qy, qz, qw = self._yaw_to_quat(yaw_out)

        # ---------- Odometry 메시지 ----------
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # pose
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # twist
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.angular.z = float(wz)

        self.pub.publish(odom)

        # ---------- TF (odom -> base_link) ----------
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tfbr.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TriboOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

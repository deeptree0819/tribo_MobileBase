#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


class OdomSourceNode(Node):
    """
    MCU 드라이버(yahboomcar_driver)가 뿌려주는
    - encoder_raw (Int32MultiArray)
    - vel_raw (Twist)
    - /imu/data_raw (Imu)
    를 모두 구독해서, odom 계산에 필요한 원본 데이터를 모아두는 노드입니다.

    지금은 단순히 최신 값만 저장하고, 주기적으로 로그로 찍기만 합니다.
    나중에 여기 timer 콜백에서 직접 odom 적분(nav_msgs/Odometry, TF)까지 구현하면 됩니다.
    """

    def __init__(self):
        super().__init__('odom_source_node')

        # 최신 값 저장용 변수
        self.last_encoders = [0, 0, 0, 0]
        self.last_vel = Twist()
        self.last_imu = Imu()
        self.have_enc = False
        self.have_vel = False
        self.have_imu = False

        # 구독자들
        self.enc_sub = self.create_subscription(
            Int32MultiArray,
            'encoder_raw',
            self.encoder_callback,
            10
        )

        self.vel_sub = self.create_subscription(
            Twist,
            'vel_raw',
            self.vel_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10
        )

        # 주기적으로 현재 값 출력 (예: 20Hz → 0.05s)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('OdomSourceNode started. Subscribing to encoder_raw, vel_raw, /imu/data_raw.')

    def encoder_callback(self, msg: Int32MultiArray):
        if len(msg.data) >= 4:
            self.last_encoders = list(msg.data[:4])
            self.have_enc = True

    def vel_callback(self, msg: Twist):
        self.last_vel = msg
        self.have_vel = True

    def imu_callback(self, msg: Imu):
        self.last_imu = msg
        self.have_imu = True

    def timer_callback(self):
        # 세 가지 데이터가 다 들어오기 시작하면 디버그 로그로 한 번씩 찍어줌
        if self.have_enc and self.have_vel and self.have_imu:
            # 너무 로그가 많으면 debug 레벨로 바꾸거나 주기를 늘려도 됨
            self.get_logger().debug(
                f"Encoders: {self.last_encoders}, "
                f"Vx: {self.last_vel.linear.x:.3f}, Vy: {self.last_vel.linear.y:.3f}, "
                f"Wz: {self.last_vel.angular.z:.3f}"
            )
            # 여기서 나중에:
            # 1) encoders 변화량으로 각 wheel 거리, robot pose 적분
            # 2) nav_msgs/Odometry 메시지 생성 후 /odom 발행
            # 3) TF broadcaster로 odom -> base_link 브로드캐스트
            # 같은 작업을 수행하면 됩니다.


def main(args=None):
    rclpy.init(args=args)
    node = OdomSourceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

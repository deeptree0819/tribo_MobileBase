#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
tribo_bringup_tribolib

TriboBase (tribolib)를 사용해서 Rosmaster 보드를 제어하는 bringup 노드.

- 하위: tribolib.TriboBase (바이너리 시리얼 프로토콜)
- 상위: /cmd_vel 구독 + 엔코더/휠속도 퍼브리시

주요 기능
1) cmd_vel → 좌/우 속도로 변환 → per-motor gain / invert 적용
2) PWM 모드: TriboBase.set_wheel_pwm(-100~100) 사용
3) auto_report를 통해 들어오는 엔코더 값을 주기적으로 읽어 퍼블리시
4) cmd_timeout 초 동안 명령이 없으면 자동 정지

Motor mapping (Rosmaster 기준)
  m1=FL, m2=RL, m3=RR, m4=FR
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Imu

from .tribolib import TriboBase


class TriboBringupTribolib(Node):
    def __init__(self):
        super().__init__("tribo_bringup")

        # ---- Serial / hardware params ----
        self.declare_parameter("port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("car_type", TriboBase.CARTYPE_X3)
        self.declare_parameter("debug_hw", False)

        # ---- Command / motion params ----
        self.declare_parameter("use_motion_mode", False)   # True면 set_motion(vx,0,wz) 사용
        self.declare_parameter("max_lin_vel", 0.5)         # m/s
        self.declare_parameter("max_ang_vel", 6.0)         # rad/s
        self.declare_parameter("turn_scale", 0.85)          # 회전만 키울 때
        self.declare_parameter("track_width", 0.12)        # m (좌우 바퀴 간 거리)
        self.declare_parameter("invert_cmd_vel", True)
        self.declare_parameter("invert_cmd_vel_angular", False)

        self.declare_parameter("vx_deadzone", 0.02)
        self.declare_parameter("wz_deadzone", 0.05)
        self.declare_parameter("cmd_timeout", 0.4)

        # ---- PWM 모드용 calib gain (정규화 기준 -1~1) ----
        self.declare_parameter("gain_m1", 0.98)
        self.declare_parameter("gain_m2", 0.98)
        self.declare_parameter("gain_m3", 1.0)
        self.declare_parameter("gain_m4", 1.0)
        # ---- 방향별(후진) 보정 계수 ----
        self.declare_parameter("gain_left_rev_factor", 1.0)   # left_norm < 0 일 때만 추가 곱
        self.declare_parameter("gain_right_rev_factor", 1.0)  # right_norm < 0 일 때만 추가 곱

        self.declare_parameter("invert_m1", False)
        self.declare_parameter("invert_m2", False)
        self.declare_parameter("invert_m3", False)
        self.declare_parameter("invert_m4", False)

        # Rosmaster PWM은 -100~100이지만,
        # 너무 작을 때는 모터가 안 도는 deadzone이 있으므로
        # |x|>0 일 때 최소 듀티(%)를 설정
        self.declare_parameter("pwm_min_percent", 20.0)    # 0~100

        # ---- Debug / telemetry ----
        self.declare_parameter("debug_tx", False)
        self.declare_parameter("publish_wheel_speed", True)
        self.declare_parameter("publish_wheel_delta", False)
        self.declare_parameter("debug_enc_speed", True)
        self.declare_parameter("debug_enc_period", 0.5)

        # ---- IMU publish (보드 9축 IMU → sensor_msgs/Imu) ----
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("imu_topic", "imu/data")   # 보드가 자세 융합 제공 → orientation 포함
        self.declare_parameter("imu_frame", "base_link")  # TF에 존재하는 프레임 (별도 imu_link 추가 시 변경)
        self.declare_parameter("imu_rate", 50.0)          # Hz (ekf frequency와 맞춤)
        self.declare_parameter("invert_imu_yaw", False)   # yaw/gz 부호가 odom과 반대일 때 True

        # ---- Read params ----
        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baudrate").value)
        self.car_type = int(self.get_parameter("car_type").value)
        self.debug_hw = bool(self.get_parameter("debug_hw").value)

        self.use_motion_mode = bool(self.get_parameter("use_motion_mode").value)
        self.max_lin = float(self.get_parameter("max_lin_vel").value)
        self.max_ang = float(self.get_parameter("max_ang_vel").value)
        self.turn_scale = float(self.get_parameter("turn_scale").value)
        self.track = float(self.get_parameter("track_width").value)
        self.invert_cmd_vel = bool(self.get_parameter("invert_cmd_vel").value)
        self.invert_cmd_vel_angular = bool(self.get_parameter("invert_cmd_vel_angular").value)

        self.vx_deadzone = float(self.get_parameter("vx_deadzone").value)
        self.wz_deadzone = float(self.get_parameter("wz_deadzone").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        self.gain = {
            "m1": float(self.get_parameter("gain_m1").value),
            "m2": float(self.get_parameter("gain_m2").value),
            "m3": float(self.get_parameter("gain_m3").value),
            "m4": float(self.get_parameter("gain_m4").value),
        }                                   
        self.gain_left_rev_factor = float(self.get_parameter("gain_left_rev_factor").value)
        self.gain_right_rev_factor = float(self.get_parameter("gain_right_rev_factor").value)                   
        
        self.invert = {
            "m1": bool(self.get_parameter("invert_m1").value),
            "m2": bool(self.get_parameter("invert_m2").value),
            "m3": bool(self.get_parameter("invert_m3").value),
            "m4": bool(self.get_parameter("invert_m4").value),
        }

        self.pwm_min_percent = float(self.get_parameter("pwm_min_percent").value)
        self.pwm_min_percent = self._clamp(self.pwm_min_percent, 0.0, 100.0)

        self.debug_tx = bool(self.get_parameter("debug_tx").value)
        self.publish_wheel_speed = bool(self.get_parameter("publish_wheel_speed").value)
        self.publish_wheel_delta = bool(self.get_parameter("publish_wheel_delta").value)
        self.debug_enc_speed = bool(self.get_parameter("debug_enc_speed").value)
        self.debug_enc_period = float(self.get_parameter("debug_enc_period").value)

        self.publish_imu = bool(self.get_parameter("publish_imu").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.imu_rate = float(self.get_parameter("imu_rate").value)
        self.invert_imu_yaw = bool(self.get_parameter("invert_imu_yaw").value)

        # ---- Hardware bringup (TriboBase) ----
        self.base = TriboBase(
            port=self.port,
            baudrate=self.baud,
            car_type=self.car_type,
            delay=0.002,
            debug=self.debug_hw,
        )
        # 수신 스레드 + auto report 켜기
        self.base.start_background_reader()
        time.sleep(0.1)
        self.base.set_auto_report(True, persist=False)
        time.sleep(0.1)

        # ---- ROS I/O ----
        self.create_subscription(Twist, "cmd_vel", self.cb_cmd, 10)
        self.pub_enc = self.create_publisher(Int32MultiArray, "encoder_raw", 50)

        self.pub_speed = None
        if self.publish_wheel_speed:
            self.pub_speed = self.create_publisher(Float32MultiArray, "wheel_ticks_per_sec", 50)

        self.pub_delta = None
        if self.publish_wheel_delta:
            self.pub_delta = self.create_publisher(Int32MultiArray, "wheel_delta_ticks", 50)

        self.pub_imu = None
        if self.publish_imu:
            self.pub_imu = self.create_publisher(Imu, self.imu_topic, 50)

        # ---- State ----
        self.last_cmd_time = self.get_clock().now()
        self._last_enc = None  # (t, e1, e2, e3, e4)
        self._last_speed_log_t = time.time()
        self._start_time = time.time()

        # ---- Timers ----
        # cmd timeout watchdog: cmd_vel이 cmd_timeout초 동안 없으면 자동 정지
        self.create_timer(0.05, self._watchdog)
        # encoder polling (TriboBase 내부 상태를 읽어서 퍼블리시)
        self.create_timer(0.05, self._enc_timer_cb)
        # IMU publishing (보드 자동보고 캐시에서 읽어 /imu/data 퍼블리시)
        if self.pub_imu is not None:
            self.create_timer(1.0 / max(self.imu_rate, 1.0), self._imu_timer_cb)

        # 시작 시 정지
        self._send_stop()

        self.get_logger().info(
            f"TriboBringupTribolib started. port={self.port}@{self.baud}, "
            f"car_type={self.car_type}, use_motion_mode={self.use_motion_mode}, "
            f"gain=({self.gain['m1']:.2f},{self.gain['m2']:.2f},"
            f"{self.gain['m3']:.2f},{self.gain['m4']:.2f}), "
            f"pwm_min_percent={self.pwm_min_percent:.1f}, "
            f"publish_imu={self.publish_imu} (topic={self.imu_topic}, frame={self.imu_frame})"
        )

    # ---------- helpers ----------
    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    @staticmethod
    def _euler_to_quat(roll: float, pitch: float, yaw: float):
        """roll/pitch/yaw [rad] → quaternion (x, y, z, w)."""
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return qx, qy, qz, qw

    def _to_pwm_percent(self, x_norm: float) -> int:
        """
        -1~1 정규화 값을 -100~100 범위의 PWM(%)로 변환.
        |x_norm|>0이면 최소 pwm_min_percent 이상이 되도록 보정.
        """
        if abs(x_norm) < 1e-6:
            return 0
        s = 1 if x_norm > 0 else -1
        a = self._clamp(abs(x_norm), 0.0, 1.0)

        # 최소 듀티 확보 (deadzone 넘어가도록)
        p = self.pwm_min_percent + a * (100.0 - self.pwm_min_percent)
        p = self._clamp(p, 0.0, 100.0)
        return int(round(s * p))

    def apply_gain_norm(self, x_norm: float, g: float, invert: bool) -> float:
        """
        정규화(-1~1) 명령에 gain을 적용한 뒤, -1~1로 클램프해서 반환.
        """
        y = self._clamp(x_norm * g, -1.0, 1.0)
        if invert:
            y = -y
        return y

    def _send_pwm(self, m1: int, m2: int, m3: int, m4: int):
        """
        Rosmaster 보드에 -100~100 범위의 PWM 퍼센트를 전송.
        """
        if self.debug_tx:
            self.get_logger().info(f"TX PWM m1={m1} m2={m2} m3={m3} m4={m4}")
        self.base.set_wheel_pwm(m1, m2, m3, m4)

    def _send_motion(self, vx: float, wz: float):
        """
        TriboBase.set_motion 사용 (보드의 속도 제어/기구학 활용).
        """
        if self.debug_tx:
            self.get_logger().info(f"TX MOTION vx={vx:.3f} wz={wz:.3f}")
        self.base.set_motion(vx, 0.0, wz)

    def _send_stop(self):
        if self.use_motion_mode:
            self._send_motion(0.0, 0.0)
        else:
            self._send_pwm(0, 0, 0, 0)

    # ---------- ROS callbacks ----------
    def cb_cmd(self, msg: Twist):
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)

        if self.invert_cmd_vel:
            vx = -vx
        if self.invert_cmd_vel_angular:
            wz = -wz

        # 회전 스케일
        wz *= self.turn_scale

        # 안전 클램프
        vx = self._clamp(vx, -self.max_lin, self.max_lin)
        wz = self._clamp(wz, -self.max_ang, self.max_ang)

        # 데드존 적용
        if abs(vx) < self.vx_deadzone:
            vx = 0.0
        if abs(wz) < self.wz_deadzone:
            wz = 0.0

        # motion 모드: 보드의 속도 제어 사용
        if self.use_motion_mode:
            self._send_motion(vx, wz)
            self.last_cmd_time = self.get_clock().now()
            return

        # ---- 여기부터 PWM 모드 (per-motor gain 사용) ----
        # 차동 근사: 좌/우 선속도
        v_left = vx - wz * (self.track / 2.0)
        v_right = vx + wz * (self.track / 2.0)

        left_norm = self._clamp(v_left / max(self.max_lin, 1e-3), -1.0, 1.0)
        right_norm = self._clamp(v_right / max(self.max_lin, 1e-3), -1.0, 1.0)
        # ---- 후진일 때만 추가 보정 (방향별 gain) ----
        if left_norm < 0.0:
            left_norm = self._clamp(left_norm * self.gain_left_rev_factor, -1.0, 1.0)
        if right_norm < 0.0:
            right_norm = self._clamp(right_norm * self.gain_right_rev_factor, -1.0, 1.0)
        # gain 적용 (정규화 기준)
        n1 = self.apply_gain_norm(left_norm, self.gain["m1"], self.invert["m1"])
        n2 = self.apply_gain_norm(left_norm, self.gain["m2"], self.invert["m2"])
        n3 = self.apply_gain_norm(right_norm, self.gain["m3"], self.invert["m3"])
        n4 = self.apply_gain_norm(right_norm, self.gain["m4"], self.invert["m4"])

        # -1~1 → -100~100 (%)
        m1 = self._to_pwm_percent(n1)
        m2 = self._to_pwm_percent(n2)
        m3 = self._to_pwm_percent(n3)
        m4 = self._to_pwm_percent(n4)

        self._send_pwm(m1, m2, m3, m4)
        self.last_cmd_time = self.get_clock().now()

    def _watchdog(self):
        """
        일정 시간 이상 cmd_vel이 안 들어오면 정지.
        """
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        if dt > self.cmd_timeout:
            self._send_stop()

    # ---------- IMU publishing ----------
    def _imu_timer_cb(self):
        """
        TriboBase 캐시(보드 자동보고)에서 자세/각속도/가속도를 읽어
        sensor_msgs/Imu 로 퍼블리시. ekf.yaml의 imu0(/imu/data)에 사용.
        """
        if self.pub_imu is None:
            return

        ax, ay, az = self.base.get_accel()           # m/s^2
        gx, gy, gz = self.base.get_gyro()             # rad/s
        roll, pitch, yaw = self.base.get_attitude(in_degrees=False)

        if self.invert_imu_yaw:
            yaw = -yaw
            gz = -gz

        qx, qy, qz, qw = self._euler_to_quat(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame

        msg.orientation.x = float(qx)
        msg.orientation.y = float(qy)
        msg.orientation.z = float(qz)
        msg.orientation.w = float(qw)

        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)

        # 대각 공분산: 2D 융합에서는 yaw / yaw-rate 만 신뢰 (roll/pitch는 큰 값)
        msg.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 0.05,
        ]
        msg.angular_velocity_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 0.01,
        ]
        msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1,
        ]

        self.pub_imu.publish(msg)

    # ---------- encoder polling ----------
    def _enc_timer_cb(self):
        """
        TriboBase 내부 상태에서 엔코더 누적값을 읽어
        encoder_raw / wheel_delta_ticks / wheel_ticks_per_sec 퍼블리시.
        """
        e1, e2, e3, e4 = self.base.get_encoders()
        now_t = time.time()
        ms = int((now_t - self._start_time) * 1000.0)

        # 1) raw encoder
        msg = Int32MultiArray()
        msg.data = [ms, e1, e2, e3, e4]
        self.pub_enc.publish(msg)

        # 이전 값이 없으면 여기서 초기화만
        if self._last_enc is None:
            self._last_enc = (now_t, e1, e2, e3, e4)
            return

        last_t, le1, le2, le3, le4 = self._last_enc
        dt = now_t - last_t
        if dt <= 1e-3:
            return

        de1 = e1 - le1
        de2 = e2 - le2
        de3 = e3 - le3
        de4 = e4 - le4

        # delta ticks publish
        if self.pub_delta is not None:
            dmsg = Int32MultiArray()
            dmsg.data = [ms, de1, de2, de3, de4]
            self.pub_delta.publish(dmsg)

        # ticks/s publish
        if self.pub_speed is not None:
            s1 = de1 / dt
            s2 = de2 / dt
            s3 = de3 / dt
            s4 = de4 / dt

            smsg = Float32MultiArray()
            smsg.data = [float(s1), float(s2), float(s3), float(s4)]
            self.pub_speed.publish(smsg)

            if self.debug_enc_speed and (now_t - self._last_speed_log_t) >= self.debug_enc_period:
                self._last_speed_log_t = now_t
                self.get_logger().info(
                    f"ENC_SPEED ticks/s m1={s1:.1f} m2={s2:.1f} m3={s3:.1f} m4={s4:.1f}"
                )

        self._last_enc = (now_t, e1, e2, e3, e4)

    # ---------- shutdown ----------
    def destroy_node(self):
        # 정지 명령 + auto_report off 정도는 선택적으로
        try:
            self._send_stop()
        except Exception:
            pass
        try:
            self.base.set_auto_report(False, persist=False)
        except Exception:
            pass
        try:
            self.base.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TriboBringupTribolib()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

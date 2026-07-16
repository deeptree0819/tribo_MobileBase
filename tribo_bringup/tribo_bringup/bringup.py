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
from sensor_msgs.msg import BatteryState

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

        # ---- PWM 천장 (전류 상한) ----
        # PWM 은 pwm_min ~ pwm_max 구간으로 매핑된다. 기본 100 = 종전과 동일.
        # ⚠️ max_lin_vel 을 낮춰 "속도를 줄이는" 것은 역효과다. 정규화가
        #    a = |v_바퀴| / max_lin_vel 이라 분모를 줄이면 a 가 커져 PWM 이 오른다.
        #    모터 출력·전류를 실제로 줄이려면 이 천장을 내려야 한다.
        # 제자리 회전은 4바퀴가 동시에 스크럽과 싸워 전류 피크가 가장 크다.
        # 전원(배터리/어댑터)이 그 피크를 못 버티면 과전류 보호가 트립되어
        # 로봇이 리셋된다 → 기체·전원별로 낮춰 잡을 것 (motor_calib.yaml).
        self.declare_parameter("pwm_max_percent", 100.0)   # 0~100

        # ---- 회전 전용 PWM 부스트 ----
        # 4륜 스키드-스티어는 제자리 회전 시 횡방향 타이어 스크럽 마찰로 stall한다.
        # 회전 성분(wz)이 유의미할 때만 그 회전에 기여하는 바퀴의 PWM 바닥을
        # rotate_pwm_min 으로 올려서 stall을 넘긴다.
        # 순수 직진(|wz|이 임계 미만)에는 절대 적용되지 않으며 기존 pwm_min_percent만 쓴다.
        self.declare_parameter("rotate_pwm_min", 45.0)         # 회전 시 적용 최소 듀티(%) (>= pwm_min_percent 권장)
        self.declare_parameter("rotate_wz_threshold", 0.10)    # |wz(turn_scale 적용 후)|가 이 값 이상이면 회전으로 간주 [rad/s]

        # ---- 제자리 회전 선형화 (역아핀 보정) ----
        # 개루프 PWM 에서 실측한 제자리 회전 응답은 비례가 아니라 아핀이다:
        #     실제_wz ≈ rot_lin_offset + rot_lin_slope * 내부_wz      (내부_wz > 0 일 때)
        # offset 은 PWM 바닥(rotate_pwm_min)이 만드는 최소 회전속도이고, 이것 때문에
        # turn_scale 같은 순수 게인으로는 명령을 맞출 수 없다(필요 배율이 명령마다 달라짐).
        # 그래서 목표 wz 를 역으로 풀어 내부 wz 로 바꿔 보낸다:
        #     내부_wz = (목표_wz - offset) / slope
        # 계수는 기체마다 다르다 → motor_calib.yaml 처럼 로봇별로 덮어쓸 것.
        # 측정: 여러 angular_speed 로 제자리 회전시키며 명령 wz 대비 실제 wz 를 잰다.
        #       (실제 wz 는 라이다 스캔 상관으로 측정 — IMU 기반 rotation_calib 은 보드
        #        자이로가 지속 회전에서 죽어 2026-07-15 제거됨.)
        # 주의: 계수는 *제자리* 회전(vx≈0)에서 잰 값이다. 전진 중 선회는 스크럽이 달라
        #       같은 식이 성립하지 않으므로, |vx| 가 작을 때만 적용한다.
        self.declare_parameter("rot_lin_enable", True)
        self.declare_parameter("rot_lin_offset", 0.186)   # rad/s (tribo v2 실측)
        self.declare_parameter("rot_lin_slope", 0.216)    # (tribo v2 실측)
        self.declare_parameter("rot_lin_max_wz", 0.55)    # 실측 물리 천장(~0.6)보다 여유 있게 아래
        self.declare_parameter("rot_lin_vx_max", 0.02)    # |vx| 가 이 이하일 때만 = 제자리 회전으로 간주

        # ---- Debug / telemetry ----
        self.declare_parameter("debug_tx", False)
        self.declare_parameter("publish_wheel_speed", True)
        self.declare_parameter("publish_wheel_delta", False)
        self.declare_parameter("debug_enc_speed", True)
        self.declare_parameter("debug_enc_period", 0.5)

        # ---- Battery (board voltage -> sensor_msgs/BatteryState) ----
        self.declare_parameter("publish_battery", True)
        self.declare_parameter("battery_topic", "battery")
        self.declare_parameter("battery_rate", 1.0)        # Hz (전압은 천천히 변함)
        self.declare_parameter("battery_full_v", 12.6)     # 3S 만충
        self.declare_parameter("battery_empty_v", 9.0)     # 3S 방전(0%)
        self.declare_parameter("battery_warn_v", 10.5)     # 저전압 경고 로그 임계
        # 저전압 부저: v <= buzzer_low_v 면 buzzer_period_s 마다 buzzer_beep_ms 만큼 삐빅.
        # buzzer_low_v <= 0 이면 부저 비활성화.
        self.declare_parameter("buzzer_low_v", 10.0)       # 부저 임계 (V)
        self.declare_parameter("buzzer_beep_ms", 300)      # 삐빅 길이 (ms)
        self.declare_parameter("buzzer_period_s", 2.0)     # 삐빅 간격 (s)
        # 저전압 강제 종료: v <= buzzer_low_v 가 low_batt_shutdown_s 초 연속 지속되면
        # 모터를 세우고 bringup 을 종료한다(과방전/오작동 방지). 0 이면 비활성(부저만).
        # 순간 전압 sag(부하 시 잠깐 강하)로 오종료되지 않게 연속 지속을 요구한다.
        self.declare_parameter("low_batt_shutdown_s", 5.0)

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

        self.pwm_max_percent = float(self.get_parameter("pwm_max_percent").value)
        self.pwm_max_percent = self._clamp(self.pwm_max_percent, 0.0, 100.0)

        self.rotate_pwm_min = float(self.get_parameter("rotate_pwm_min").value)
        self.rotate_pwm_min = self._clamp(self.rotate_pwm_min, 0.0, 100.0)
        self.rotate_wz_threshold = float(self.get_parameter("rotate_wz_threshold").value)

        self.rot_lin_enable = bool(self.get_parameter("rot_lin_enable").value)
        self.rot_lin_offset = float(self.get_parameter("rot_lin_offset").value)
        self.rot_lin_slope = float(self.get_parameter("rot_lin_slope").value)
        self.rot_lin_max_wz = float(self.get_parameter("rot_lin_max_wz").value)
        self.rot_lin_vx_max = float(self.get_parameter("rot_lin_vx_max").value)
        if self.rot_lin_enable and self.rot_lin_slope <= 1e-6:
            self.get_logger().warn(
                f"rot_lin_slope={self.rot_lin_slope} 가 0 이하 → 회전 선형화를 끈다.")
            self.rot_lin_enable = False

        self.debug_tx = bool(self.get_parameter("debug_tx").value)
        self.publish_wheel_speed = bool(self.get_parameter("publish_wheel_speed").value)
        self.publish_wheel_delta = bool(self.get_parameter("publish_wheel_delta").value)
        self.debug_enc_speed = bool(self.get_parameter("debug_enc_speed").value)
        self.debug_enc_period = float(self.get_parameter("debug_enc_period").value)

        self.publish_battery = bool(self.get_parameter("publish_battery").value)
        self.battery_topic = str(self.get_parameter("battery_topic").value)
        self.battery_rate = float(self.get_parameter("battery_rate").value)
        self.battery_full_v = float(self.get_parameter("battery_full_v").value)
        self.battery_empty_v = float(self.get_parameter("battery_empty_v").value)
        self.battery_warn_v = float(self.get_parameter("battery_warn_v").value)
        self._last_batt_warn_t = 0.0
        self.buzzer_low_v = float(self.get_parameter("buzzer_low_v").value)
        self.buzzer_beep_ms = int(self.get_parameter("buzzer_beep_ms").value)
        self.buzzer_period_s = float(self.get_parameter("buzzer_period_s").value)
        self._last_buzz_t = 0.0
        self.low_batt_shutdown_s = float(self.get_parameter("low_batt_shutdown_s").value)
        self._low_batt_since = None   # 저전압 진입 시각 (연속 지속 측정용)
        self._shutting_down = False

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

        self.pub_battery = None
        if self.publish_battery:
            self.pub_battery = self.create_publisher(BatteryState, self.battery_topic, 10)

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
        # Battery publishing (보드 전압 캐시 → /battery)
        if self.pub_battery is not None:
            self.create_timer(1.0 / max(self.battery_rate, 0.1), self._battery_timer_cb)

        # 시작 시 정지
        self._send_stop()

        self.get_logger().info(
            f"TriboBringupTribolib started. port={self.port}@{self.baud}, "
            f"car_type={self.car_type}, use_motion_mode={self.use_motion_mode}, "
            f"gain=({self.gain['m1']:.2f},{self.gain['m2']:.2f},"
            f"{self.gain['m3']:.2f},{self.gain['m4']:.2f}), "
            f"pwm_min_percent={self.pwm_min_percent:.1f}, "
            f"rotate_pwm_min={self.rotate_pwm_min:.1f} (wz_thr={self.rotate_wz_threshold:.2f}), "
            f"publish_battery={self.publish_battery} (topic={self.battery_topic})"
        )

    # ---------- helpers ----------
    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def _to_pwm_percent(self, x_norm: float, pwm_min: float = None) -> int:
        """
        -1~1 정규화 값을 -100~100 범위의 PWM(%)로 변환.
        |x_norm|>0이면 최소 pwm_min 이상이 되도록 보정.

        pwm_min 미지정 시 기본 self.pwm_min_percent(정지마찰 바닥)을 사용.
        회전 stall 보정 시 cb_cmd가 해당 바퀴에만 rotate_pwm_min을 넘겨준다.
        """
        if abs(x_norm) < 1e-6:
            return 0
        if pwm_min is None:
            pwm_min = self.pwm_min_percent
        s = 1 if x_norm > 0 else -1
        a = self._clamp(abs(x_norm), 0.0, 1.0)

        # 듀티는 [pwm_min, pwm_max] 구간으로 매핑한다.
        # pwm_min: 정지마찰/스크럽을 넘기는 바닥. pwm_max: 전류 피크를 묶는 천장.
        # 바닥이 천장보다 높게 잘못 설정돼도 바닥을 우선해 stall 은 피한다.
        pwm_max = max(self.pwm_max_percent, pwm_min)
        p = pwm_min + a * (pwm_max - pwm_min)
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

    def _linearize_wz(self, vx: float, wz: float) -> float:
        """목표 wz [rad/s] → PWM 매핑에 넣을 내부 wz.

        실측 응답이 아핀(실제 = offset + slope*내부)이므로 역으로 푼다.
        offset 미만은 물리적으로 낼 수 없는 속도다(PWM 바닥이 만드는 최소 회전).
        그 구간에서는 0으로 죽이지 않고 최소 회전을 유지한다 — 0으로 만들면
        Nav2 가 각도 오차를 못 줄여 영영 수렴하지 못한다. 대신 목표보다 빨리 돈다.
        """
        if not self.rot_lin_enable:
            return wz
        # 전진 중 선회는 스크럽 조건이 달라 이 계수가 성립하지 않는다 → 손대지 않는다.
        if abs(vx) > self.rot_lin_vx_max:
            return wz
        if abs(wz) < 1e-9:
            return wz

        target = min(abs(wz), self.rot_lin_max_wz)   # 낼 수 없는 속도는 요구하지 않는다
        if target > self.rot_lin_offset:
            inner = (target - self.rot_lin_offset) / self.rot_lin_slope
        else:
            # 목표가 최소 회전속도보다 느림 → 그대로 두면 PWM 바닥이 걸려 어차피
            # offset 만큼 돈다. 0 으로 만들면 스톨하므로 아주 작은 값을 남긴다.
            inner = 1e-3
        inner = self._clamp(inner, 0.0, self.max_ang)
        return math.copysign(inner, wz)

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
        # 제자리 회전 선형화: 목표 wz 를 내부 wz 로 역변환한다(위 파라미터 주석 참고).
        # 여기서부터 wz 는 "보드에 보낼 내부 값"이며, 사용자가 요청한 목표가 아니다.
        wz = self._linearize_wz(vx, wz)

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

        # ---- 회전 전용 PWM 바닥 선택 ----
        # 회전 성분(wz)이 임계 이상일 때만 회전에 기여하는 모든 바퀴의 PWM 바닥을
        # rotate_pwm_min으로 올린다(스크럽 마찰 극복). 순수 직진(|wz|<임계)에는
        # 기존 pwm_min_percent가 그대로 적용되어 직진 거동에 영향이 없다.
        # (rotate_pwm_min < pwm_min_percent로 잘못 설정돼도 max로 안전하게 바닥 보장)
        if abs(wz) >= self.rotate_wz_threshold:
            pwm_floor = max(self.rotate_pwm_min, self.pwm_min_percent)
        else:
            pwm_floor = self.pwm_min_percent

        # -1~1 → -100~100 (%)
        m1 = self._to_pwm_percent(n1, pwm_floor)
        m2 = self._to_pwm_percent(n2, pwm_floor)
        m3 = self._to_pwm_percent(n3, pwm_floor)
        m4 = self._to_pwm_percent(n4, pwm_floor)

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

    # ---------- Battery publishing ----------
    def _battery_timer_cb(self):
        """
        보드 배터리 전압을 sensor_msgs/BatteryState 로 퍼블리시.
        저전압이면 주기적으로 경고 로그.
        """
        if self.pub_battery is None:
            return

        v = float(self.base.get_battery_voltage())

        # 0~1 충전율 (full/empty 파라미터 기준)
        if self.battery_full_v > self.battery_empty_v:
            pct = (v - self.battery_empty_v) / (self.battery_full_v - self.battery_empty_v)
            pct = self._clamp(pct, 0.0, 1.0)
        else:
            pct = float("nan")

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = v
        msg.percentage = float(pct)
        msg.present = v > 1.0
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        # 보드가 안 주는 값은 NaN
        msg.current = float("nan")
        msg.charge = float("nan")
        msg.capacity = float("nan")
        msg.design_capacity = float("nan")
        self.pub_battery.publish(msg)

        # 저전압 경고 (30초마다 1회)
        now_t = time.time()
        if 1.0 < v <= self.battery_warn_v and (now_t - self._last_batt_warn_t) > 30.0:
            self._last_batt_warn_t = now_t
            self.get_logger().warn(
                f"LOW BATTERY: {v:.2f} V (<= {self.battery_warn_v:.1f} V), {pct * 100:.0f}%"
            )

        # 저전압 부저: buzzer_low_v 이하이면 buzzer_period_s 마다 짧게 삐빅.
        # (v>1.0 은 배터리 미연결 0V 오검출 방지. beep 은 duration 후 자동 OFF 라 끌 필요 없음.)
        low = self.buzzer_low_v > 0.0 and 1.0 < v <= self.buzzer_low_v
        if low and (now_t - self._last_buzz_t) >= self.buzzer_period_s:
            self._last_buzz_t = now_t
            try:
                self.base.beep(self.buzzer_beep_ms)
            except Exception as e:
                self.get_logger().warn(f"부저 명령 실패: {e}")

        # 저전압 강제 종료: 저전압이 low_batt_shutdown_s 초 연속 지속되면 종료.
        if self.low_batt_shutdown_s > 0.0 and low:
            if self._low_batt_since is None:
                self._low_batt_since = now_t
            held = now_t - self._low_batt_since
            if held >= self.low_batt_shutdown_s and not self._shutting_down:
                self.get_logger().error(
                    f"저전압 {v:.2f} V 가 {self.low_batt_shutdown_s:.0f}s 연속 지속 "
                    f"→ 모터 정지 후 bringup 종료 (과방전 방지)"
                )
                try:
                    self._send_stop()
                except Exception:
                    pass
                try:
                    self.base.beep(2000)   # 종료 경고음 (길게)
                except Exception:
                    pass
                # main 루프가 감지해 정상 종료(destroy_node/close). 콜백에서 직접
                # rclpy.shutdown() 하면 main finally 와 이중 호출되어 에러난다.
                self._shutting_down = True
        else:
            self._low_batt_since = None   # 전압 회복 시 카운터 리셋

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
        # 저전압 강제 종료(_shutting_down) 시 spin 을 빠져나와 정상 정리한다.
        while rclpy.ok() and not node._shutting_down:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

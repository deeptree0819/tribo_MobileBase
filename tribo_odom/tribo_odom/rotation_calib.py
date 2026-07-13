#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""제자리 회전 캘리브레이션 노드.

제자리 회전을 (방향 교대로) 여러 번 시키며 세 회전 소스를 비교한다:
  - 휠 odom (/odom_raw twist.angular.z)  : 엔코더 기반, 슬립에 취약
  - IMU 자이로 (/imu/data angular_velocity.z) : 실제 회전(ground truth)
  - EKF 융합 (/odom twist.angular.z)     : 현재 융합 결과

각 회전마다 yaw 적분값과 비율을 내고, 평균/표준편차로 슬립이 안정적인지 판단한다.

용도:
  1) /odom_raw 유효 track_width 보정 (자유회전 시 wheel≈IMU 가 되도록)
     -> 권장 track = current_track * mean(wheel/IMU)
  2) 슬립 변동(CV)이 크면 고정 트랙으로는 한계 -> EKF가 IMU vyaw 로 yaw 추정하도록 권장
     (ekf.yaml: odom0 yaw/vyaw=false, imu0 vyaw=true)

주의: 결과만 출력하고 설정을 자동 수정하지 않는다. 측정값을 보고 사용자가 적용.

실행 예:
  ros2 run tribo_odom rotation_calib
  ros2 run tribo_odom rotation_calib --ros-args \
    -p angular_speed:=0.6 -p spin_duration:=5.0 -p num_spins:=4 -p current_track:=0.834
"""
import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from geometry_msgs.msg import Twist


class RotationCalib(Node):
    # 상태
    WARMUP, SPIN, SETTLE, DONE = range(4)

    def __init__(self):
        super().__init__("rotation_calib")

        # ---------- parameters ----------
        self.declare_parameter("angular_speed", 0.6)   # rad/s (회전 명령 크기)
        self.declare_parameter("spin_duration", 5.0)   # s (1회 회전 시간)
        self.declare_parameter("num_spins", 4)         # 회전 횟수
        self.declare_parameter("alternate", True)      # 방향 교대 (제자리 부근 유지)
        self.declare_parameter("settle_time", 1.2)     # s (회전 후 정지·정착)
        self.declare_parameter("warmup", 2.0)          # s (초기 토픽 안정화)
        self.declare_parameter("current_track", 0.873) # 현재 유효 track_width (m), 권장값 산출용
        self.declare_parameter("physical_track", 0.70) # 물리 트랙(좌우), 참고용
        self.declare_parameter("cmd_topic", "cmd_vel")
        self.declare_parameter("odom_raw_topic", "odom_raw")
        self.declare_parameter("imu_topic", "imu/data")
        self.declare_parameter("ekf_topic", "odom")
        self.declare_parameter("use_ekf", True)

        g = self.get_parameter
        self.w_speed = float(g("angular_speed").value)
        self.spin_dur = float(g("spin_duration").value)
        self.num_spins = int(g("num_spins").value)
        self.alternate = bool(g("alternate").value)
        self.settle = float(g("settle_time").value)
        self.warmup = float(g("warmup").value)
        self.cur_track = float(g("current_track").value)
        self.phys_track = float(g("physical_track").value)
        self.use_ekf = bool(g("use_ekf").value)

        # ---------- ros I/O ----------
        self.pub = self.create_publisher(Twist, str(g("cmd_topic").value), 10)
        self.create_subscription(Odometry, str(g("odom_raw_topic").value),
                                 lambda m: self.raw.append((time.monotonic(), m.twist.twist.angular.z)), 50)
        self.create_subscription(Imu, str(g("imu_topic").value),
                                 lambda m: self.imu.append((time.monotonic(), m.angular_velocity.z)), 50)
        if self.use_ekf:
            self.create_subscription(Odometry, str(g("ekf_topic").value),
                                     lambda m: self.ekf.append((time.monotonic(), m.twist.twist.angular.z)), 50)
        # 전압이 낮으면 토크가 모자라 스크럽을 못 이기고 바퀴만 헛돈다 → 측정 전체가 무효가 된다.
        self.batt_v = float("nan")
        self.create_subscription(BatteryState, "battery",
                                 lambda m: setattr(self, "batt_v", float(m.voltage)), 10)

        # ---------- state ----------
        self.raw, self.imu, self.ekf = [], [], []
        self.state = self.WARMUP
        self.t_state = time.monotonic()   # 현재 상태 시작 시각
        self.spin_idx = 0
        self.spin_t0 = None
        self.rows = []                    # (imu_deg, wheel_deg, ekf_deg, r_raw, r_ekf, track_true)

        self.timer = self.create_timer(0.02, self.loop)  # 50 Hz
        self.get_logger().info(
            f"RotationCalib 시작 | speed={self.w_speed} rad/s, dur={self.spin_dur}s, "
            f"spins={self.num_spins}, alternate={self.alternate}, current_track={self.cur_track} m"
        )
        self.get_logger().info("로봇 주변을 비워두세요. 제자리 회전을 시작합니다...")

    # ---------- helpers ----------
    @staticmethod
    def _integ(series, t0, t1):
        """[t0,t1] 사다리꼴 적분 -> 누적 각(rad, 절대값)."""
        pts = [(t, v) for (t, v) in series if t0 <= t <= t1]
        tot = 0.0
        for i in range(1, len(pts)):
            tot += 0.5 * (pts[i][1] + pts[i - 1][1]) * (pts[i][0] - pts[i - 1][0])
        return abs(tot)

    def _cmd(self, wz):
        tw = Twist()
        tw.angular.z = wz
        self.pub.publish(tw)

    def _spin_sign(self):
        if self.alternate and (self.spin_idx % 2 == 1):
            return -1.0
        return 1.0

    # ---------- state machine ----------
    def loop(self):
        now = time.monotonic()
        el = now - self.t_state

        if self.state == self.WARMUP:
            if el >= self.warmup:
                if not (self.raw and self.imu):
                    self.get_logger().warn(
                        f"토픽 대기중... raw={len(self.raw)} imu={len(self.imu)} "
                        "(bringup 실행 중인지 확인)")
                    self.t_state = now  # 계속 대기
                    return
                self._check_battery()
                self._begin_spin(now)
            return

        if self.state == self.SPIN:
            self._cmd(self._spin_sign() * self.w_speed)
            if el >= self.spin_dur:
                self._end_spin(now)
            return

        if self.state == self.SETTLE:
            self._cmd(0.0)
            if el >= self.settle:
                if self.spin_idx >= self.num_spins:
                    self._finish()
                else:
                    self._begin_spin(now)
            return

    def _check_battery(self):
        """전압이 낮으면 결과를 못 믿는다. 측정 전에 한 번, 리포트에서 다시 경고한다."""
        if math.isnan(self.batt_v):
            self.get_logger().warn(
                "배터리 전압을 못 읽었다(/battery 없음). 전압이 낮으면 슬립이 폭주해 결과가 무효가 된다.")
        elif self.batt_v < self.BATT_MIN_V:
            self.get_logger().warn(
                f"⚠ 배터리 {self.batt_v:.1f}V < {self.BATT_MIN_V:.1f}V — 토크 부족으로 바퀴가 헛돈다. "
                "이 상태의 측정값은 신뢰할 수 없다. 충전 후 다시 측정할 것.")
        else:
            self.get_logger().info(f"배터리 {self.batt_v:.1f}V — 측정 가능.")

    def _begin_spin(self, now):
        self.spin_idx += 1
        self.spin_t0 = now
        self.state = self.SPIN
        self.t_state = now
        self.get_logger().info(
            f"[spin {self.spin_idx}/{self.num_spins}] wz={self._spin_sign()*self.w_speed:+.2f} rad/s")

    def _end_spin(self, now):
        t0, t1 = self.spin_t0, now
        imu_a = self._integ(self.imu, t0, t1)
        raw_a = self._integ(self.raw, t0, t1)
        ekf_a = self._integ(self.ekf, t0, t1) if self.use_ekf else float("nan")
        r_raw = raw_a / imu_a if imu_a > 1e-3 else float("nan")
        r_ekf = (ekf_a / imu_a) if (self.use_ekf and imu_a > 1e-3) else float("nan")
        track_true = self.cur_track * r_raw if not math.isnan(r_raw) else float("nan")
        sign = self._spin_sign()
        # 명령 대비 실제 회전(개루프 PWM이라 cmd_vel이 실제 rad/s를 보장하지 않음)
        cmd_a = self.w_speed * self.spin_dur
        r_cmd = imu_a / cmd_a if cmd_a > 1e-6 else float("nan")
        self.rows.append((math.degrees(imu_a), math.degrees(raw_a),
                          math.degrees(ekf_a), r_raw, r_ekf, track_true, sign, r_cmd))
        self.get_logger().info(
            f"  IMU={math.degrees(imu_a):6.1f}deg  wheel={math.degrees(raw_a):6.1f}deg  "
            f"EKF={math.degrees(ekf_a):6.1f}deg | wheel/IMU={r_raw:.3f} "
            f"EKF/IMU={r_ekf:.3f}  track_true={track_true:.3f}  실제/명령={r_cmd:.2f}")
        self.state = self.SETTLE
        self.t_state = now

    def _finish(self):
        self._cmd(0.0)
        self._report()
        self.state = self.DONE
        self.timer.cancel()
        rclpy.shutdown()

    # ---------- report ----------
    @staticmethod
    def _mean_std(a):
        a = [x for x in a if not math.isnan(x)]
        if not a:
            return float("nan"), float("nan")
        m = sum(a) / len(a)
        s = (sum((x - m) ** 2 for x in a) / len(a)) ** 0.5
        return m, s

    # 제어 비대칭(CW/CCW 회전량 차). 모터/기구 문제이며 odom 스케일과는 별개다.
    ASYM_WARN = 5.0     # %
    # track_true 의 방향간 편차. track_width 는 이 값 하나로 정하므로,
    # "단일 track 이 성립하는가" 는 IMU 각 비대칭이 아니라 이걸로 판정해야 한다.
    TRACK_SPREAD_WARN = 5.0     # %
    # 명령 대비 실제 회전이 이 아래면 개루프 PWM 매핑이 명령을 못 따라가는 것.
    CMD_TRACK_WARN = 0.85
    # 배터리가 낮으면 토크 부족으로 스크럽을 못 이겨 슬립이 폭주한다 → 측정 자체가 무효.
    BATT_MIN_V = 11.0

    def _report(self):
        L = self.get_logger().info
        W = self.get_logger().warn
        L("========== ROTATION CALIB RESULT ==========")
        L(" #  dir IMU(deg) wheel(deg) EKF(deg)  wheel/IMU EKF/IMU track_true 실제/명령")
        for i, r in enumerate(self.rows):
            d = "CCW" if r[6] > 0 else "CW "
            L(f" {i+1}  {d} {r[0]:7.1f} {r[1]:8.1f} {r[2]:8.1f}   "
              f"{r[3]:7.3f} {r[4]:7.3f} {r[5]:8.3f} {r[7]:8.2f}")

        rr = [r[3] for r in self.rows]
        re = [r[4] for r in self.rows]
        tt = [r[5] for r in self.rows]
        mr, sr = self._mean_std(rr)
        me, se = self._mean_std(re)
        mt, st = self._mean_std(tt)
        cv = (100.0 * sr / mr) if mr and not math.isnan(mr) and mr != 0 else float("nan")

        # ---- 방향별로 갈라 본다 (평균만 보면 계통 비대칭이 노이즈로 위장된다) ----
        ccw = [r for r in self.rows if r[6] > 0]
        cw = [r for r in self.rows if r[6] < 0]
        asym = float("nan")       # 제어 비대칭 (CW/CCW 회전량 차)
        spread = float("nan")     # odom 비대칭 (CW/CCW track_true 차)
        if ccw and cw:
            m_ccw, _ = self._mean_std([r[0] for r in ccw])   # IMU 각
            m_cw, _ = self._mean_std([r[0] for r in cw])
            t_ccw, _ = self._mean_std([r[5] for r in ccw])   # track_true
            t_cw, _ = self._mean_std([r[5] for r in cw])
            base = 0.5 * (m_ccw + m_cw)
            asym = 100.0 * abs(m_ccw - m_cw) / base if base > 1e-6 else float("nan")
            tbase = 0.5 * (t_ccw + t_cw)
            spread = 100.0 * abs(t_ccw - t_cw) / tbase if tbase > 1e-6 else float("nan")
            L("-------------------------------------------")
            L(f" 방향별 IMU  : CW={m_cw:.1f}deg  CCW={m_ccw:.1f}deg  → 제어 비대칭 {asym:.1f}%")
            L(f" 방향별 track: CW={t_cw:.3f} m  CCW={t_ccw:.3f} m  → odom 편차 {spread:.1f}%")

        # ---- 명령 대비 실제 (개루프 PWM은 cmd_vel 대로 안 돈다) ----
        mc, _ = self._mean_std([r[7] for r in self.rows])
        cmd_deg = math.degrees(self.w_speed * self.spin_dur)

        L("-------------------------------------------")
        L(f" wheel/IMU : 평균={mr:.3f} 표준편차={sr:.3f} (변동 {cv:.0f}%)")
        L(f" EKF/IMU   : 평균={me:.3f} 표준편차={se:.3f}")
        L(f" track_true: 평균={mt:.3f} m (현재 {self.cur_track}, 물리 {self.phys_track})")
        L(f" 실제/명령 : 평균={mc:.2f}  (명령 {cmd_deg:.0f}deg 당 실제 {mc*cmd_deg:.0f}deg)")
        L(f" 배터리    : {self.batt_v:.1f} V")
        L("-------------------------------------------")

        # 전압이 낮으면 아래 판단 전부가 무의미하므로 가장 먼저 못박는다.
        if not math.isnan(self.batt_v) and self.batt_v < self.BATT_MIN_V:
            W(f" ⚠ 배터리 {self.batt_v:.1f}V < {self.BATT_MIN_V:.1f}V — 이 측정은 무효다.")
            W("   토크가 모자라 바퀴가 헛돌아 슬립이 폭주한다. 아래 수치를 근거로 설정을 바꾸지 말 것.")
            W("   충전(12V대) 후 재측정할 것.")

        # ---- 판단 ----
        # track_width 권장은 "단일 track 이 성립하는가" 로만 막는다.
        # 그 판정 기준은 odom 편차(track_true 의 방향간 차)이지, 제어 비대칭(IMU 각 차)이 아니다.
        # 로봇이 한쪽으로 더 빨리 돌아도(제어 비대칭) 휠 odom 스케일은 일정할 수 있고,
        # 그 경우 단일 track 은 여전히 유효하다. 둘을 섞으면 멀쩡한 권장을 막는다.
        if not math.isnan(spread) and spread > self.TRACK_SPREAD_WARN:
            W(f" odom 편차 {spread:.1f}% (>{self.TRACK_SPREAD_WARN:.0f}%) → 방향에 따라 휠 odom 스케일이 다르다.")
            W("   단일 track 으로는 양방향을 동시에 못 맞춘다. track 조정은 보류하고 먼저 확인할 것:")
            W("     - 배터리 전압 (낮으면 슬립 폭주 → 측정 무효)")
            W("     - 모터 게인이 이 기체 것인가 (config/motor_calib.yaml, README 8-1)")
        elif not math.isnan(mr):
            if math.isnan(cv) or cv > 15.0:
                L(" 판단: 슬립 변동이 큼(변동>15%) → 고정 track 으로는 한계.")
                L("       권장: EKF 가 회전을 IMU 자이로로 추정하도록 설정")
                L("       ekf.yaml: odom0 yaw/vyaw=false, imu0 vyaw=true")
            else:
                L(f" 판단: 슬립 안정 + odom 편차 작음 → 유효 track 을 {mt:.3f} m 로 설정 권장")
                L("       (bringup.launch.py _odom_common_params track_width)")

        # 제어 비대칭은 track 과 무관한 별개 문제 — 막지 말고 따로 보고한다.
        if not math.isnan(asym) and asym > self.ASYM_WARN:
            W(f" 제어 비대칭 {asym:.1f}% (>{self.ASYM_WARN:.0f}%) → 로봇이 한쪽으로 더 많이 돈다.")
            W("   odom 이 아니라 모터/기구 문제다 (track 으로는 못 고친다). 확인할 것:")
            W("     - 정/역방향 비대칭 보정 (bringup.yaml gain_left_rev_factor / gain_right_rev_factor)")
            W("     - IMU 자이로 z 바이어스 (정지 상태에서 /imu/data 의 angular_velocity.z 확인)")

        if not math.isnan(mc) and mc < self.CMD_TRACK_WARN:
            W(f" 실제/명령={mc:.2f} → 로봇이 명령한 각속도의 {mc*100:.0f}% 만 돈다.")
            W("   개루프 PWM 모드(use_motion_mode=false)라 cmd_vel 이 실제 rad/s 를 보장하지 않는다.")
            W("   Nav2 는 명령대로 돌 것을 가정하므로 회전이 undershoot 된다.")
            W("   대응: turn_scale 상향, 또는 보드 속도 폐루프(use_motion_mode=true) 검토.")

        if not math.isnan(me):
            if abs(me - 1.0) <= 0.10:
                L(f" EKF/IMU={me:.3f} → EKF 회전이 이미 실제와 일치(±10%). 양호.")
                L("   (EKF yaw 는 IMU 기반이므로 track_width 조정은 /odom_raw 에만 영향)")
            else:
                L(f" EKF/IMU={me:.3f} → EKF 회전이 실제와 어긋남. 위 권장 적용 필요.")
        L("===========================================")


def main():
    rclpy.init()
    node = RotationCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 안전 정지
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

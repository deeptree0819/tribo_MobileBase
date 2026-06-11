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
from sensor_msgs.msg import Imu
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
        self.rows.append((math.degrees(imu_a), math.degrees(raw_a),
                          math.degrees(ekf_a), r_raw, r_ekf, track_true))
        self.get_logger().info(
            f"  IMU={math.degrees(imu_a):6.1f}deg  wheel={math.degrees(raw_a):6.1f}deg  "
            f"EKF={math.degrees(ekf_a):6.1f}deg | wheel/IMU={r_raw:.3f} "
            f"EKF/IMU={r_ekf:.3f}  track_true={track_true:.3f}")
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

    def _report(self):
        L = self.get_logger().info
        L("========== ROTATION CALIB RESULT ==========")
        L(" #  IMU(deg) wheel(deg) EKF(deg)  wheel/IMU EKF/IMU track_true")
        for i, r in enumerate(self.rows):
            L(f" {i+1}  {r[0]:7.1f} {r[1]:8.1f} {r[2]:8.1f}   {r[3]:7.3f} {r[4]:7.3f} {r[5]:8.3f}")

        rr = [r[3] for r in self.rows]
        re = [r[4] for r in self.rows]
        tt = [r[5] for r in self.rows]
        mr, sr = self._mean_std(rr)
        me, se = self._mean_std(re)
        mt, st = self._mean_std(tt)
        cv = (100.0 * sr / mr) if mr and not math.isnan(mr) and mr != 0 else float("nan")

        L("-------------------------------------------")
        L(f" wheel/IMU : 평균={mr:.3f} 표준편차={sr:.3f} (변동 {cv:.0f}%)")
        L(f" EKF/IMU   : 평균={me:.3f} 표준편차={se:.3f}")
        L(f" track_true: 평균={mt:.3f} m (현재 {self.cur_track}, 물리 {self.phys_track})")
        L("-------------------------------------------")
        # 권장
        if not math.isnan(mr):
            if math.isnan(cv) or cv > 15.0:
                L(" 판단: 슬립 변동이 큼(변동>15%) → 고정 track 으로는 한계.")
                L("       권장: EKF 가 회전을 IMU 자이로로 추정하도록 설정")
                L("       ekf.yaml: odom0 yaw/vyaw=false, imu0 vyaw=true")
            else:
                L(f" 판단: 슬립 비교적 안정 → 유효 track 을 {mt:.3f} m 로 설정 권장")
                L("       (bringup.launch.py _odom_common_params track_width)")
            if not math.isnan(me):
                if abs(me - 1.0) <= 0.10:
                    L(f" EKF/IMU={me:.3f} → EKF 회전이 이미 실제와 일치(±10%). 양호.")
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

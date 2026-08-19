#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spin_test.py — 제자리 회전 계측 (4b·6·7단계용)

두 가지 모드가 있다.

  turns  : odom 기준으로 N 바퀴 돌고 멈춘다. 물리 회전각을 눈으로 재서
           odom 유효 track 을 잡는다(6단계).
             new_track = 현재_track * (odom 누적각 / 물리 실제각)
           ⚠️ 이 방법이 유일하게 신뢰할 수 있는 이유: odom yaw 자체가
              track 으로 계산되므로, odom 을 자로 쓰면 순환 참조가 된다.
              물리 각도만이 독립적인 기준이다.

  sweep  : wz 를 여러 값으로 주고 각각의 실제 yaw rate 를 odom 으로 잰다.
           - rotate_pwm_min 을 낮춰둔 상태로 돌리면 "제자리 회전이 시작되는
             최저 duty"를 찾을 수 있다(4b단계).
           - track 확정 후 돌리면 rot_lin 역아핀 계수를 피팅할 수 있다(7단계).
               실제_wz = rot_lin_offset + rot_lin_slope * 명령_wz
           ⚠️ sweep 은 odom yaw 를 자로 쓰므로 6단계(track 확정) 이후에만
              의미가 있다. 그 전에는 4b(움직이나/안 움직이나) 판정에만 쓸 것.

주의:
    - 바퀴를 바닥에 둔다. 제자리 회전은 스크럽 마찰이 핵심이라 무부하 측정은 무의미하다.
    - 로봇 전폭이 0.78m 다. 반경 1m 정도 빈 공간을 확보할 것.
    - 배터리 11V 이상. 제자리 회전은 네 바퀴가 동시에 스크럽을 이겨야 해서
      전류 피크가 가장 크다. 저전압이면 보드가 리셋된다.
    - bringup 이 이미 떠 있어야 한다.

사용법:
    export ROS_DOMAIN_ID=38
    source ~/tribo_ws/install/setup.bash

    # 6단계 — 3바퀴 회전 후 물리 오차 측정
    python3 scripts/spin_test.py turns 3
    python3 scripts/spin_test.py turns 3 --wz 0.5     # 회전 속도 지정

    # 4b/7단계 — wz 스윕
    python3 scripts/spin_test.py sweep 0.2 0.3 0.4 0.6 0.8
"""

import sys
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


DEFAULT_WZ = 0.4        # rad/s — turns 모드 기본 회전 명령
SWEEP_RUN = 5.0         # s — sweep 모드에서 각 wz 를 유지할 시간
SETTLE = 3.0            # s — 구간 사이 정지(메카넘은 회전 활주도 길다)
MEASURE_FRAC = 0.6      # 뒤쪽 몇 %를 정상 yaw rate 로 채택할지
PUB_HZ = 20.0
TURNS_TIMEOUT = 120.0   # s — turns 모드 안전 상한


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SpinTest(Node):
    def __init__(self, mode, turns, wz_list, wz):
        super().__init__("spin_test")
        self.mode = mode
        self.turns = turns
        self.wz_list = wz_list
        self.wz = wz

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self._cb_odom, 50)

        self.yaw_prev = None
        self.yaw_acc = 0.0      # 언랩된 누적 yaw [rad]
        self.wz_latest = None

        self.t0 = None
        self.done = False

        # sweep 상태
        self.seq = []
        if mode == "sweep":
            for w in wz_list:
                self.seq.append(("stop", 0.0, SETTLE))
                self.seq.append((f"wz {w:.2f}", w, SWEEP_RUN))
            self.seq.append(("stop", 0.0, SETTLE))
        self.idx = 0
        self.seg_t0 = None
        self.samples = []
        self.seg_yaw0 = None
        self.results = []

        self.create_timer(1.0 / PUB_HZ, self._tick)
        if mode == "turns":
            self.get_logger().info(
                f"turns 모드 — odom 기준 {turns} 바퀴({turns*360}도), wz={wz} rad/s. "
                f"바닥에 기준선을 그어두고 시작할 것."
            )
        else:
            self.get_logger().info(f"sweep 모드 — wz {wz_list}")

    def _cb_odom(self, msg):
        y = yaw_of(msg)
        if self.yaw_prev is not None:
            d = y - self.yaw_prev
            # 언랩 (-pi~pi 경계 넘어갈 때)
            if d > math.pi:
                d -= 2.0 * math.pi
            elif d < -math.pi:
                d += 2.0 * math.pi
            self.yaw_acc += d
        self.yaw_prev = y
        self.wz_latest = float(msg.twist.twist.angular.z)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        if self.yaw_prev is None or self.done:
            return
        if self.mode == "turns":
            self._tick_turns()
        else:
            self._tick_sweep()

    # ---------- turns ----------
    def _tick_turns(self):
        now = self._now()
        if self.t0 is None:
            self.t0 = now
            self.yaw_acc = 0.0
            self.get_logger().info("회전 시작")

        target = self.turns * 2.0 * math.pi
        elapsed = now - self.t0

        if abs(self.yaw_acc) < target and elapsed < TURNS_TIMEOUT:
            cmd = Twist()
            cmd.angular.z = self.wz
            self.pub.publish(cmd)
            return

        self.pub.publish(Twist())
        self.done = True
        deg = math.degrees(self.yaw_acc)
        print()
        print("=" * 70)
        print("  제자리 회전 결과 (turns 모드)")
        print("=" * 70)
        print(f"  명령 wz          : {self.wz:.3f} rad/s")
        print(f"  경과 시간        : {elapsed:.2f} s")
        print(f"  odom 누적 회전   : {deg:.1f} deg  ({deg/360.0:.3f} 바퀴)")
        print(f"  평균 yaw rate    : {self.yaw_acc/elapsed:.4f} rad/s")
        if elapsed >= TURNS_TIMEOUT:
            print("  ⚠️ 시간 초과로 중단됐다. 회전이 너무 느리거나 스톨했다.")
        print()
        print("  다음: 로봇이 실제로 몇 도 돌았는지 물리적으로 재라.")
        print("    출발선 대비 남거나 지나친 각도를 눈으로 재서 더한다.")
        print(f"    예) 3바퀴 명령인데 실제로 1080+25 도 돌았다면 물리각 = 1105")
        print()
        print("    new_track = 현재_track * (odom누적각 / 물리각)")
        print(f"              = 현재_track * ({deg:.1f} / 물리각)")
        print()
        print("  ⚠️ 눈측정 분해능이 ~3% 다. 그 이상 자릿수는 무의미하다.")
        print("     바퀴 수를 늘릴수록(3~6) 상대 오차가 준다.")
        print("=" * 70)
        rclpy.shutdown()

    # ---------- sweep ----------
    def _tick_sweep(self):
        now = self._now()
        if self.idx >= len(self.seq):
            self.pub.publish(Twist())
            self.done = True
            self._report_sweep()
            rclpy.shutdown()
            return

        label, w, dur = self.seq[self.idx]
        if self.seg_t0 is None:
            self.seg_t0 = now
            self.samples = []
            self.seg_yaw0 = self.yaw_acc

        elapsed = now - self.seg_t0
        if elapsed < dur:
            cmd = Twist()
            cmd.angular.z = w
            self.pub.publish(cmd)
            if w != 0.0 and elapsed >= dur * (1.0 - MEASURE_FRAC):
                if self.wz_latest is not None:
                    self.samples.append(abs(self.wz_latest))
            return

        if w != 0.0:
            dyaw = abs(self.yaw_acc - self.seg_yaw0)
            mean_wz = sum(self.samples) / len(self.samples) if self.samples else float("nan")
            self.results.append((abs(w), mean_wz, dyaw, dur))
            self.get_logger().info(
                f"[{label}] 명령 {abs(w):.3f} -> 실측 {mean_wz:.3f} rad/s "
                f"(odom 회전 {math.degrees(dyaw):.1f} deg / {dur:.1f}s)"
            )

        self.idx += 1
        self.seg_t0 = None

    def _report_sweep(self):
        print()
        print("=" * 70)
        print("  wz 스윕 결과 — 명령 vs 실제 (odom 기준)")
        print("=" * 70)
        print(f"  {'명령[rad/s]':>12} {'실측[rad/s]':>12} {'명령대비':>10}")
        print("  " + "-" * 40)
        pts = []
        for cw, mw, dyaw, dur in self.results:
            ratio = mw / cw if cw else float("nan")
            print(f"  {cw:>12.3f} {mw:>12.3f} {ratio:>9.2f}x")
            if not math.isnan(mw):
                pts.append((cw, mw))
        print()
        if len(pts) >= 2:
            n = len(pts)
            sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
            sxx = sum(p[0] * p[0] for p in pts); sxy = sum(p[0] * p[1] for p in pts)
            den = n * sxx - sx * sx
            if abs(den) > 1e-12:
                slope = (n * sxy - sx * sy) / den
                offset = (sy - slope * sx) / n
                print(f"  아핀 피팅: 실제_wz = {offset:.4f} + {slope:.4f} * 명령_wz")
                print(f"    -> rot_lin_offset: {offset:.3f}")
                print(f"    -> rot_lin_slope : {slope:.3f}")
                print("    (bringup.yaml 또는 per-unit motor_calib.yaml 에 넣고")
                print("     rot_lin_enable: true 로 켠 뒤 이 스윕을 다시 돌려 확인할 것)")
        print()
        print("  ⚠️ 이 표는 odom yaw 를 자로 쓴다. 6단계에서 유효 track 을 확정한")
        print("     뒤에만 rot_lin 피팅에 쓸 수 있다. 그 전에는 '도는가/안 도는가'")
        print("     판정에만 사용할 것.")
        print("=" * 70)


def main():
    argv = [a for a in sys.argv[1:]]
    wz = DEFAULT_WZ
    if "--wz" in argv:
        i = argv.index("--wz")
        wz = float(argv[i + 1])
        del argv[i:i + 2]

    if not argv:
        print(__doc__)
        return
    mode = argv[0]
    if mode == "turns":
        turns = float(argv[1]) if len(argv) > 1 else 3.0
        wz_list = []
    elif mode == "sweep":
        turns = 0.0
        wz_list = [float(a) for a in argv[1:]] or [0.2, 0.3, 0.4, 0.6, 0.8]
    else:
        print(f"알 수 없는 모드: {mode}  (turns | sweep)")
        return

    rclpy.init()
    node = SpinTest(mode, turns, wz_list, wz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

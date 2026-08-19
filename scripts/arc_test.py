#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arc_test.py — 곡선 주행 검증 (vx>0 이면서 wz>0 인 원호)

무엇을 보는가:
    명령한 곡률이 실제로 나오는가. 직진과 제자리 회전은 각각 speed_sweep.py /
    spin_test.py 로 잡았지만, 그 둘 사이의 "원호"는 별도 문제다.

    특히 회전 전용 PWM 바닥(rotate_pwm_min)이 곡선 주행에도 걸리면, 안쪽 바퀴
    듀티가 바닥까지 끌어올려져 좌우 차이가 붕괴하고 곡률이 사라진다. 그러면
    로봇이 낼 수 있는 동작이 "직진"과 "제자리 회전" 둘뿐이 되어, Nav2 가 원호를
    명령해도 직진으로 뭉개진다. bringup.py 의 rotate_pwm_vx_max 가 그 방지책이고,
    이 스크립트가 그게 실제로 동작하는지 확인한다.

무엇을 재는가:
    각 (vx, wz) 마다 odom twist 로 실제 vx / wz 를 재고 실제 반경을 구한다.
        명령 반경 = vx / wz          실제 반경 = vx_실측 / wz_실측
    곡률이 뭉개지면 실제 반경이 명령 반경보다 크게 나온다(더 곧게 감).

공간 절약:
    각 케이스를 (vx, wz) -> 정지 -> (-vx, -wz) 로 왕복시킨다. 부호를 둘 다 뒤집으면
    같은 원호를 되짚어 오므로 대략 출발점으로 돌아온다.

주의:
    - 바닥에 두고 실행한다. 반경 R 인 원호를 T 초 달리면 대략 R*(1-cos) 만큼
      옆으로 벌어진다. 기본 설정에서 2.5m x 2m 정도면 충분하다.
    - rot_lin 은 |vx| <= rot_lin_vx_max(0.02) 일 때만 걸리므로 곡선 주행에는
      적용되지 않는다. 즉 여기서 wz 는 보정 없이 그대로 들어간다.
    - bringup 이 이미 떠 있어야 한다.

사용법:
    export ROS_DOMAIN_ID=38
    source ~/tribo_ws/install/setup.bash
    python3 scripts/arc_test.py
    python3 scripts/arc_test.py 0.25,0.2 0.25,0.4 0.15,0.3     # vx,wz 쌍 직접 지정
"""

import sys
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# (vx, wz) 기본 케이스. 완만한 원호 -> 급한 원호 순.
DEFAULT_CASES = [
    (0.25, 0.10),   # R = 2.50 m
    (0.25, 0.20),   # R = 1.25 m  <- 커밋 메시지가 문제 삼은 조합
    (0.25, 0.40),   # R = 0.63 m
    (0.15, 0.30),   # R = 0.50 m
]

RUN = 4.0            # s — 한 방향 주행 시간
SETTLE = 3.0         # s — 구간 사이 정지
MEASURE_FRAC = 0.6   # 뒤쪽 몇 %를 정상상태로 채택
PUB_HZ = 20.0


class ArcTest(Node):
    def __init__(self, cases):
        super().__init__("arc_test")
        self.cases = cases
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self._cb_odom, 50)

        self.vx_latest = None
        self.wz_latest = None

        self.seq = []
        for vx, wz in cases:
            self.seq.append(("stop", 0.0, 0.0, SETTLE))
            self.seq.append((f"fwd vx={vx:.2f} wz={wz:.2f}", vx, wz, RUN))
            self.seq.append(("stop", 0.0, 0.0, SETTLE))
            self.seq.append((f"rev vx={vx:.2f} wz={wz:.2f}", -vx, -wz, RUN))
        self.seq.append(("stop", 0.0, 0.0, SETTLE))

        self.idx = 0
        self.seg_t0 = None
        self.s_vx = []
        self.s_wz = []
        self.results = []

        self.create_timer(1.0 / PUB_HZ, self._tick)
        self.get_logger().info(
            f"arc_test 시작 — {len(cases)} 케이스, 예상 {sum(d for _,_,_,d in self.seq):.0f}s"
        )

    def _cb_odom(self, msg):
        self.vx_latest = float(msg.twist.twist.linear.x)
        self.wz_latest = float(msg.twist.twist.angular.z)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        if self.vx_latest is None:
            return
        now = self._now()

        if self.idx >= len(self.seq):
            self.pub.publish(Twist())
            self._report()
            rclpy.shutdown()
            return

        label, vx, wz, dur = self.seq[self.idx]
        if self.seg_t0 is None:
            self.seg_t0 = now
            self.s_vx = []
            self.s_wz = []

        elapsed = now - self.seg_t0
        if elapsed < dur:
            cmd = Twist()
            cmd.linear.x = vx
            cmd.angular.z = wz
            self.pub.publish(cmd)
            if vx != 0.0 and elapsed >= dur * (1.0 - MEASURE_FRAC):
                self.s_vx.append(abs(self.vx_latest))
                self.s_wz.append(abs(self.wz_latest))
            return

        if vx != 0.0 and self.s_vx:
            mvx = sum(self.s_vx) / len(self.s_vx)
            mwz = sum(self.s_wz) / len(self.s_wz)
            self.results.append((label, abs(vx), abs(wz), mvx, mwz))
            r_cmd = abs(vx) / abs(wz)
            r_act = mvx / mwz if mwz > 1e-6 else float("inf")
            self.get_logger().info(
                f"[{label}] 실측 vx={mvx:.3f} wz={mwz:.3f} | "
                f"반경 명령 {r_cmd:.2f} -> 실제 {r_act:.2f} m"
            )

        self.idx += 1
        self.seg_t0 = None

    def _report(self):
        print()
        print("=" * 82)
        print("  곡선 주행 검증 — 명령 곡률 vs 실제 곡률")
        print("=" * 82)
        print(f"  {'vx':>5} {'wz':>5} {'방향':>5} {'vx실측':>8} {'wz실측':>8} "
              f"{'R명령':>7} {'R실제':>7} {'R오차':>8}")
        print("  " + "-" * 76)
        worst = 0.0
        for label, vx, wz, mvx, mwz in self.results:
            d = "전진" if label.startswith("fwd") else "후진"
            r_cmd = vx / wz
            r_act = mvx / mwz if mwz > 1e-6 else float("inf")
            err = (r_act / r_cmd - 1.0) * 100.0
            worst = max(worst, abs(err))
            print(f"  {vx:>5.2f} {wz:>5.2f} {d:>5} {mvx:>8.3f} {mwz:>8.3f} "
                  f"{r_cmd:>7.2f} {r_act:>7.2f} {err:>7.1f}%")
        print()
        print("  판정 기준:")
        print("   - R실제가 R명령보다 크게 나오면 곡률이 뭉개진 것이다(더 곧게 감).")
        print("     회전 바닥이 곡선에도 걸릴 때 나타나는 전형적 증상이다.")
        print("   - vx/wz 각각의 추종 오차(직진 ±7%, 회전 ±2%)가 섞여 들어오므로")
        print("     R 오차 10% 안쪽이면 곡률 자체는 살아 있다고 본다.")
        print(f"   - 이번 최대 R 오차 = {worst:.1f}%")
        print("=" * 82)


def main():
    args = sys.argv[1:]
    if args:
        cases = []
        for a in args:
            vx, wz = a.split(",")
            cases.append((float(vx), float(wz)))
    else:
        cases = DEFAULT_CASES

    rclpy.init()
    node = ArcTest(cases)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
speed_sweep.py — 명령 속도 vs 실제 속도 매핑 측정 (4·5단계용)

무엇을 재는가:
    cmd_vel.linear.x 를 여러 값으로 주면서 각각의 "정상속도"를 /odom 으로 잰다.
    결과로 max_lin_vel 재산정(5단계)과 pwm_min_percent 하한 탐색(4단계)에 필요한
    명령→실제 대응표가 나온다.

왜 /odom 을 자로 쓰는가:
    wheel_radius 가 확정된 뒤로 odom 거리는 줄자와 일치한다(2026-08-19 2회 검증).
    주행마다 줄자를 재는 것보다 훨씬 빠르고 재현성이 좋다.

공간 절약:
    각 속도를 전진 -> 정지 -> 후진 -> 정지 로 왕복시켜 제자리로 돌아온다.
    필요한 여유 공간은 아래 DIST_BUDGET(기본 1.5m) + 안전 마진 정도다.
    덤으로 전/후진 대칭성도 같이 나온다.

주의:
    - 바퀴를 바닥에 둔 상태로 실행한다(무부하가 아니라 실주행 조건을 재는 것).
    - 배터리 11V 이상. 저전압이면 전부 쓰레기값이다.
    - bringup 이 이미 떠 있어야 한다. 이 스크립트는 bringup 을 띄우지 않는다.

사용법:
    source /opt/ros/jazzy/setup.bash && source ~/tribo_ws/install/setup.bash
    export ROS_DOMAIN_ID=38
    python3 scripts/speed_sweep.py
    python3 scripts/speed_sweep.py 0.10 0.15 0.20 0.25 0.30 0.40    # 속도 직접 지정
"""

import sys
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# 기본 스윕 속도 [m/s]. 낮은 쪽은 PWM 바닥(4단계), 높은 쪽은 정규화 분모(5단계)용.
DEFAULT_SPEEDS = [0.10, 0.15, 0.20, 0.25, 0.30, 0.40]

DIST_BUDGET = 1.5      # m — 한 구간에서 이동을 허용할 거리(공간 제약)
RUN_MIN = 3.0          # s — 최소 주행 시간(가속 구간이 전체를 먹지 않도록)
RUN_MAX = 8.0          # s — 최대 주행 시간
SETTLE = 3.0           # s — 구간 사이 정지(메카넘은 활주가 길다)
MEASURE_FRAC = 0.6     # 주행 시간의 뒤쪽 몇 %를 "정상속도"로 채택할지
PUB_HZ = 20.0


class SpeedSweep(Node):
    def __init__(self, speeds):
        super().__init__("speed_sweep")
        self.speeds = speeds
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self._cb_odom, 50)

        self.vx_latest = None      # /odom 의 순간 선속도
        self.pos_latest = None     # (x, y)

        # 시퀀스 생성: 각 속도마다 전진/후진 왕복
        self.seq = []
        for v in speeds:
            run_t = self._run_time(v)
            self.seq.append(("stop", 0.0, SETTLE))
            self.seq.append((f"fwd {v:.2f}", +v, run_t))
            self.seq.append(("stop", 0.0, SETTLE))
            self.seq.append((f"rev {v:.2f}", -v, run_t))
        self.seq.append(("stop", 0.0, SETTLE))

        self.idx = 0
        self.seg_t0 = None
        self.samples = []          # 현재 구간의 정상속도 샘플
        self.seg_pos0 = None
        self.results = []          # (label, cmd_v, mean_v, odom_dist, dt)

        self.create_timer(1.0 / PUB_HZ, self._tick)
        self.get_logger().info(
            f"speed_sweep 시작 — 속도 {speeds}, 구간당 최대 {DIST_BUDGET} m 이동. "
            f"예상 소요 {self._total_time():.0f}s"
        )

    def _run_time(self, v):
        """공간 예산 안에서 가능한 주행 시간."""
        if v <= 1e-6:
            return RUN_MIN
        return max(RUN_MIN, min(RUN_MAX, DIST_BUDGET / v))

    def _total_time(self):
        return sum(d for _, _, d in self.seq)

    def _cb_odom(self, msg):
        self.vx_latest = float(msg.twist.twist.linear.x)
        p = msg.pose.pose.position
        self.pos_latest = (float(p.x), float(p.y))

    def _tick(self):
        if self.pos_latest is None:
            return  # odom 대기

        now = self.get_clock().now().nanoseconds * 1e-9

        if self.idx >= len(self.seq):
            self.pub.publish(Twist())
            self._report()
            rclpy.shutdown()
            return

        label, v, dur = self.seq[self.idx]

        if self.seg_t0 is None:
            self.seg_t0 = now
            self.samples = []
            self.seg_pos0 = self.pos_latest

        elapsed = now - self.seg_t0

        if elapsed < dur:
            cmd = Twist()
            cmd.linear.x = v
            self.pub.publish(cmd)
            # 뒤쪽 MEASURE_FRAC 구간만 정상속도로 채택(가속 구간 제외)
            if v != 0.0 and elapsed >= dur * (1.0 - MEASURE_FRAC):
                if self.vx_latest is not None:
                    self.samples.append(abs(self.vx_latest))
            return

        # 구간 종료
        if v != 0.0:
            dx = self.pos_latest[0] - self.seg_pos0[0]
            dy = self.pos_latest[1] - self.seg_pos0[1]
            dist = math.hypot(dx, dy)
            mean_v = sum(self.samples) / len(self.samples) if self.samples else float("nan")
            self.results.append((label, abs(v), mean_v, dist, dur))
            self.get_logger().info(
                f"[{label}] 명령 {abs(v):.3f} -> 실측 {mean_v:.3f} m/s "
                f"(odom 이동 {dist:.3f} m / {dur:.1f}s, 샘플 {len(self.samples)})"
            )

        self.idx += 1
        self.seg_t0 = None

    def _report(self):
        print()
        print("=" * 74)
        print("  속도 스윕 결과 — 명령 vs 실제")
        print("=" * 74)
        print(f"  {'명령[m/s]':>10} {'전진실측':>10} {'후진실측':>10} {'평균':>9} "
              f"{'명령대비':>9} {'전후차':>8}")
        print("  " + "-" * 70)

        by_cmd = {}
        for label, cv, mv, dist, dur in self.results:
            by_cmd.setdefault(cv, {})["fwd" if label.startswith("fwd") else "rev"] = mv

        rows = []
        for cv in sorted(by_cmd):
            f = by_cmd[cv].get("fwd", float("nan"))
            r = by_cmd[cv].get("rev", float("nan"))
            avg = (f + r) / 2.0
            ratio = avg / cv if cv else float("nan")
            asym = (f - r) / avg * 100.0 if avg else float("nan")
            rows.append((cv, f, r, avg, ratio))
            print(f"  {cv:>10.3f} {f:>10.3f} {r:>10.3f} {avg:>9.3f} "
                  f"{ratio:>8.2f}x {asym:>7.1f}%")

        print()
        print("  해석:")
        print("   - '명령대비'가 1.00 이면 그 속도에서 명령=실제. 1보다 작으면 덜 나간다.")
        print("   - 저속에서 실측이 바닥을 치고 더 안 내려가면 그게 pwm_min_percent 바닥이다(4단계).")
        print("   - Nav2 동작대역(0.20~0.30)에서 1.00 이 되도록 max_lin_vel 을 역산한다(5단계).")
        print("   - '전후차'가 5% 넘으면 gain_left/right_rev_factor 를 본다.")

        # max_lin_vel 역산 힌트: 동작대역 평균 비율로 스케일
        band = [(cv, ratio) for cv, _, _, _, ratio in rows if 0.19 <= cv <= 0.31]
        if band:
            mean_ratio = sum(r for _, r in band) / len(band)
            print()
            print(f"  동작대역(0.2~0.3) 평균 명령대비 = {mean_ratio:.3f}")
            print(f"  → 1차 근사로 max_lin_vel 을 현재값 x {mean_ratio:.3f} 하면 그 대역이 맞는다.")
            print("     (응답이 아핀이라 정확하지 않다. 적용 후 이 스윕을 다시 돌려 확인할 것.)")
        print("=" * 74)


def main():
    speeds = [float(a) for a in sys.argv[1:]] or DEFAULT_SPEEDS
    rclpy.init()
    node = SpeedSweep(speeds)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

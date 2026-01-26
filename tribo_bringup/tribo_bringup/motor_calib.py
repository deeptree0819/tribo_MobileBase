#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tribo motor/encoder calibration helper (ROS2)

Subscribes:
  - /encoder_raw : std_msgs/Int32MultiArray  data=[ms,e1,e2,e3,e4]  (cumulative ticks)
  - /odom        : nav_msgs/Odometry        (optional but recommended for yaw-rate)

Publishes:
  - /cmd_vel     : geometry_msgs/Twist

Sequence:
  stop -> forward -> stop -> backward -> stop -> rotate_left -> stop -> rotate_right -> stop
Outputs suggested bringup.yaml tuning values:
  - gain_m1..gain_m4
  - gain_left_rev_factor / gain_right_rev_factor
  - turn_scale

Notes:
  - Assumes m1=FL, m2=RL, m3=RR, m4=FR like your odom_publisher uses.
  - Uses encoder deltas with int32 wrap handling.
"""

import math
import statistics
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    # yaw from quaternion (Z-axis)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def unwrap_angle(prev: float, cur: float) -> float:
    # unwrap to keep continuity
    d = cur - prev
    while d > math.pi:
        cur -= 2.0 * math.pi
        d = cur - prev
    while d < -math.pi:
        cur += 2.0 * math.pi
        d = cur - prev
    return cur


def diff_int32(curr: int, prev: int) -> int:
    # signed int32 wrap diff
    mask = 0xFFFFFFFF
    d = (curr - prev) & mask
    if d & 0x80000000:
        d -= 0x100000000
    return int(d)


@dataclass
class SegmentResult:
    name: str
    duration_s: float
    # tick/sec for each motor [m1,m2,m3,m4]
    rates: List[float]
    # side rates
    left_rate: float
    right_rate: float
    # odom yaw-rate (rad/s), abs meaningful for turn_scale
    yaw_rate: Optional[float] = None
    # sign check
    signs: List[int] = None


class TriboCalibrator(Node):
    def __init__(self):
        super().__init__("tribo_calibrator")

        # ---- parameters ----
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("encoder_topic", "/encoder_raw")
        self.declare_parameter("odom_topic", "/odom")

        # motion for test
        self.declare_parameter("test_vx", 0.20)          # m/s
        self.declare_parameter("test_wz", 1.00)          # rad/s (command)
        self.declare_parameter("run_time", 1.5)          # seconds each motion segment
        self.declare_parameter("stop_time", 0.8)         # seconds between segments
        self.declare_parameter("pub_rate", 20.0)         # cmd_vel publish rate (Hz)

        # current bringup settings (for computing "new = current * ratio")
        self.declare_parameter("current_turn_scale", 1.0)
        self.declare_parameter("current_gain_m1", 1.0)
        self.declare_parameter("current_gain_m2", 1.0)
        self.declare_parameter("current_gain_m3", 1.0)
        self.declare_parameter("current_gain_m4", 1.0)
        self.declare_parameter("current_gain_left_rev_factor", 1.0)
        self.declare_parameter("current_gain_right_rev_factor", 1.0)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.encoder_topic = str(self.get_parameter("encoder_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)

        self.test_vx = float(self.get_parameter("test_vx").value)
        self.test_wz = float(self.get_parameter("test_wz").value)
        self.run_time = float(self.get_parameter("run_time").value)
        self.stop_time = float(self.get_parameter("stop_time").value)
        self.pub_rate = float(self.get_parameter("pub_rate").value)

        self.cur_turn_scale = float(self.get_parameter("current_turn_scale").value)
        self.cur_gain = [
            float(self.get_parameter("current_gain_m1").value),
            float(self.get_parameter("current_gain_m2").value),
            float(self.get_parameter("current_gain_m3").value),
            float(self.get_parameter("current_gain_m4").value),
        ]
        self.cur_left_rev = float(self.get_parameter("current_gain_left_rev_factor").value)
        self.cur_right_rev = float(self.get_parameter("current_gain_right_rev_factor").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.sub_enc = self.create_subscription(Int32MultiArray, self.encoder_topic, self._cb_enc, qos)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self._cb_odom, qos)

        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)

        # encoder state
        self._enc_prev: Optional[List[int]] = None  # [e1..e4]
        self._enc_prev_t: Optional[float] = None

        # odom yaw state
        self._yaw_prev: Optional[float] = None
        self._yaw_prev_t: Optional[float] = None
        self._yaw_latest: Optional[float] = None

        # segment accumulation
        self._seg_active = False
        self._seg_name = ""
        self._seg_start_t = 0.0
        self._seg_end_t = 0.0
        self._seg_start_enc: Optional[List[int]] = None
        self._seg_end_enc: Optional[List[int]] = None
        self._seg_start_yaw: Optional[float] = None
        self._seg_end_yaw: Optional[float] = None

        self._results: List[SegmentResult] = []

        # sequence definition: (name, vx, wz, duration)
        self._sequence: List[Tuple[str, float, float, float]] = [
            ("stop0", 0.0, 0.0, self.stop_time),
            ("forward", +self.test_vx, 0.0, self.run_time),
            ("stop1", 0.0, 0.0, self.stop_time),
            ("backward", -self.test_vx, 0.0, self.run_time),
            ("stop2", 0.0, 0.0, self.stop_time),
            ("rotate_left", 0.0, +self.test_wz, self.run_time),
            ("stop3", 0.0, 0.0, self.stop_time),
            ("rotate_right", 0.0, -self.test_wz, self.run_time),
            ("stop4", 0.0, 0.0, self.stop_time),
        ]
        self._seq_idx = 0

        self.get_logger().info(
            "Calibrator started. Stop teleop first. "
            f"Will publish on {self.cmd_topic} and read {self.encoder_topic}, {self.odom_topic}."
        )

        # timers
        self._pub_timer = self.create_timer(1.0 / self.pub_rate, self._tick_publish)
        self._seq_timer = self.create_timer(0.05, self._tick_sequence)  # state machine

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _cb_enc(self, msg: Int32MultiArray):
        if len(msg.data) < 5:
            return
        # msg.data = [ms, e1, e2, e3, e4]
        e = [int(msg.data[1]), int(msg.data[2]), int(msg.data[3]), int(msg.data[4])]
        t = self._now()

        if self._enc_prev is None:
            self._enc_prev = e
            self._enc_prev_t = t

        # if a segment is active, keep last encoder snapshot
        if self._seg_active:
            self._seg_end_enc = e

    def _cb_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        t = self._now()

        if self._yaw_prev is None:
            self._yaw_prev = yaw
            self._yaw_prev_t = t
            self._yaw_latest = yaw
            return

        yaw_unwrapped = unwrap_angle(self._yaw_prev, yaw)
        self._yaw_latest = yaw_unwrapped
        self._yaw_prev = yaw_unwrapped
        self._yaw_prev_t = t

        if self._seg_active:
            self._seg_end_yaw = yaw_unwrapped

    def _publish_cmd(self, vx: float, wz: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub_cmd.publish(msg)

    def _tick_publish(self):
        # publish current segment command
        if self._seq_idx >= len(self._sequence):
            # keep stopped at end
            self._publish_cmd(0.0, 0.0)
            return
        _, vx, wz, _ = self._sequence[self._seq_idx]
        self._publish_cmd(vx, wz)

    def _start_segment(self, name: str, duration: float):
        t = self._now()
        self._seg_active = True
        self._seg_name = name
        self._seg_start_t = t
        self._seg_end_t = t + duration
        self._seg_start_enc = self._enc_prev[:] if self._enc_prev else None
        self._seg_end_enc = self._enc_prev[:] if self._enc_prev else None

        # yaw start/end (optional)
        self._seg_start_yaw = self._yaw_latest
        self._seg_end_yaw = self._yaw_latest

        self.get_logger().info(f"[SEG START] {name}  ({duration:.2f}s)")

    def _finish_segment(self):
        if not self._seg_active:
            return
        t_end = self._now()
        name = self._seg_name
        t0 = self._seg_start_t
        t1 = min(t_end, self._seg_end_t)
        dt = max(1e-6, t1 - t0)

        if self._seg_start_enc is None or self._seg_end_enc is None:
            self.get_logger().warning(f"[SEG END] {name}: no encoder data")
            self._seg_active = False
            return

        d_ticks = [diff_int32(c, p) for c, p in zip(self._seg_end_enc, self._seg_start_enc)]
        rates = [d / dt for d in d_ticks]  # ticks/sec
        left_rate = (rates[0] + rates[1]) / 2.0
        right_rate = (rates[2] + rates[3]) / 2.0
        signs = [0 if r == 0 else (1 if r > 0 else -1) for r in rates]

        yaw_rate = None
        if self._seg_start_yaw is not None and self._seg_end_yaw is not None:
            dyaw = self._seg_end_yaw - self._seg_start_yaw
            yaw_rate = dyaw / dt

        self._results.append(
            SegmentResult(
                name=name,
                duration_s=dt,
                rates=rates,
                left_rate=left_rate,
                right_rate=right_rate,
                yaw_rate=yaw_rate,
                signs=signs,
            )
        )
        self.get_logger().info(
            f"[SEG END] {name} dt={dt:.2f}s "
            f"rates(t/s)={['%.1f'%r for r in rates]} "
            f"LR=({left_rate:.1f},{right_rate:.1f}) "
            f"yaw_rate={None if yaw_rate is None else ('%.3f'%yaw_rate)}"
        )
        self._seg_active = False

    def _tick_sequence(self):
        t = self._now()

        # wait until we have encoder baseline
        if self._enc_prev is None:
            return

        # all segments done
        if self._seq_idx >= len(self._sequence):
            if not hasattr(self, "_printed"):
                setattr(self, "_printed", True)
                self._print_summary_and_exit()
            return

        name, _, _, dur = self._sequence[self._seq_idx]

        if not self._seg_active:
            self._start_segment(name, dur)
            return

        if t >= self._seg_end_t:
            self._finish_segment()
            self._seq_idx += 1

    def _find(self, name: str) -> Optional[SegmentResult]:
        for r in self._results:
            if r.name == name:
                return r
        return None

    def _safe_ratio(self, a: float, b: float, default: float = 1.0) -> float:
        if abs(b) < 1e-6:
            return default
        return a / b

    def _print_summary_and_exit(self):
        fwd = self._find("forward")
        back = self._find("backward")
        rl = self._find("rotate_left")
        rr = self._find("rotate_right")

        self.get_logger().info("========== CALIBRATION SUMMARY ==========")

        def print_seg(sr: Optional[SegmentResult]):
            if sr is None:
                self.get_logger().info("  (missing segment)")
                return
            self.get_logger().info(
                f"  {sr.name}: "
                f"m1={sr.rates[0]:.1f} m2={sr.rates[1]:.1f} m3={sr.rates[2]:.1f} m4={sr.rates[3]:.1f} "
                f" | L={sr.left_rate:.1f} R={sr.right_rate:.1f}"
                + ("" if sr.yaw_rate is None else f" | yaw_rate={sr.yaw_rate:.3f} rad/s")
            )

        print_seg(fwd)
        print_seg(back)
        print_seg(rl)
        print_seg(rr)

        # ---- Recommendations ----
        self.get_logger().info("---------- RECOMMENDATIONS ----------")

        # 1) Straight balance (forward) -> per-side gain suggestion
        if fwd is not None:
            # We want left_rate ~= right_rate in forward
            # If right is slower: multiply right gains by (left/right)
            corr_right = self._safe_ratio(abs(fwd.left_rate), abs(fwd.right_rate), 1.0)
            # If right is faster, corr_right < 1; alternatively scale left.
            corr_left = self._safe_ratio(abs(fwd.right_rate), abs(fwd.left_rate), 1.0)

            # Also per-wheel within side
            # For left: match m1 and m2
            l1, l2 = abs(fwd.rates[0]), abs(fwd.rates[1])
            r3, r4 = abs(fwd.rates[2]), abs(fwd.rates[3])
            corr_m2 = self._safe_ratio(l1, l2, 1.0)  # multiply m2 to match m1
            corr_m4 = self._safe_ratio(r3, r4, 1.0)  # multiply m4 to match m3

            # Compose: side correction + within-side correction
            # Keep m1,m3 as references, adjust m2,m4 for within-side, then adjust right side overall.
            new_gain = self.cur_gain[:]
            new_gain[1] *= corr_m2
            new_gain[3] *= corr_m4
            new_gain[2] *= corr_right
            new_gain[3] *= corr_right

            self.get_logger().info(
                f"[직진 보정(Forward)] L={fwd.left_rate:.1f}, R={fwd.right_rate:.1f} ticks/s\n"
                f"  - 오른쪽이 느리면 corr_right>1 로 보정됩니다. corr_right={corr_right:.3f}\n"
                f"  - 추천 gain_m*: "
                f"m1={new_gain[0]:.3f}, m2={new_gain[1]:.3f}, m3={new_gain[2]:.3f}, m4={new_gain[3]:.3f}"
            )

            # Sign sanity for forward
            if any(s == 0 for s in fwd.signs):
                self.get_logger().warning("[직진] 어떤 바퀴는 tick rate가 0에 가깝습니다. pwm_min_percent를 올리거나 기계적 저항을 확인하세요.")
            if not (fwd.left_rate > 0 and fwd.right_rate > 0):
                self.get_logger().warning(
                    "[직진] 전진인데 좌/우 tick 부호가 기대와 다릅니다. "
                    "invert_m# 또는 invert_translation 중 한 군데에서만 바로잡으세요."
                )

        # 2) Reverse balance -> rev factors
        if back is not None:
            # In backward, absolute speeds should still balance L/R
            corr_right_rev = self._safe_ratio(abs(back.left_rate), abs(back.right_rate), 1.0)
            # Recommend rev factors relative to current
            new_left_rev = self.cur_left_rev
            new_right_rev = self.cur_right_rev * corr_right_rev
            self.get_logger().info(
                f"[후진 보정(Backward)] L={back.left_rate:.1f}, R={back.right_rate:.1f} ticks/s\n"
                f"  - 후진에서 오른쪽이 느리면 gain_right_rev_factor를 키웁니다.\n"
                f"  - 추천 gain_left_rev_factor={new_left_rev:.3f}, gain_right_rev_factor={new_right_rev:.3f}"
            )

        # 3) Turn scale from odom yaw-rate (use rotate_left magnitude)
        # We want |yaw_rate_meas| ~= |cmd_wz| (test_wz)
        yaw_rates = []
        if rl is not None and rl.yaw_rate is not None:
            yaw_rates.append(abs(rl.yaw_rate))
        if rr is not None and rr.yaw_rate is not None:
            yaw_rates.append(abs(rr.yaw_rate))
        if yaw_rates:
            yaw_meas = statistics.mean(yaw_rates)
            ratio = self._safe_ratio(abs(self.test_wz), yaw_meas, 1.0)
            new_turn_scale = self.cur_turn_scale * ratio
            self.get_logger().info(
                f"[회전 보정] cmd_wz={self.test_wz:.3f} rad/s, meas_yaw_rate≈{yaw_meas:.3f} rad/s\n"
                f"  - 추천 turn_scale = current_turn_scale({self.cur_turn_scale:.3f}) * ({ratio:.3f}) = {new_turn_scale:.3f}\n"
                f"  - (참고) 회전이 여전히 둔하면 pwm_min_percent도 같이 올리세요."
            )
        else:
            self.get_logger().warning(
                "[회전 보정] /odom에서 yaw를 못 받아서 turn_scale 산출을 못 했습니다. "
                "odom_topic이 맞는지 확인하거나, 회전 구간에서 /odom이 업데이트되는지 확인하세요."
            )

        # 4) Rotation sign sanity
        if rl is not None:
            # rotate_left should produce opposite signs between left and right (in ticks/s)
            if rl.left_rate == 0 or rl.right_rate == 0:
                self.get_logger().warning("[좌회전] 한쪽이 거의 안 도는 것 같습니다. pwm_min_percent를 올리거나 기계적 저항/배선 확인.")
            if (rl.left_rate > 0 and rl.right_rate > 0) or (rl.left_rate < 0 and rl.right_rate < 0):
                self.get_logger().warning(
                    "[좌회전] 좌/우가 같은 부호로 움직입니다(둘 다 전진/후진). "
                    "차동 회전이 안 나옵니다. 모터 매핑(m1~m4) 또는 invert 설정을 점검하세요."
                )

        # Print YAML snippet
        self.get_logger().info("---------- YAML SNIPPET (bringup.yaml) ----------")
        self.get_logger().info(
            "tribo_bringup:\n"
            "  ros__parameters:\n"
            f"    turn_scale: {self.cur_turn_scale:.3f}   # <- 위 추천값으로 교체\n"
            "    # pwm_min_percent: 35.0  # 필요 시 30~40 범위에서 조정\n"
            "    # gain_m1: 1.0\n"
            "    # gain_m2: 1.0\n"
            "    # gain_m3: 1.0\n"
            "    # gain_m4: 1.0\n"
            "    # gain_left_rev_factor: 1.0\n"
            "    # gain_right_rev_factor: 1.0\n"
        )

        self.get_logger().info("Done. Stopping robot (cmd_vel=0) and shutting down.")
        self._publish_cmd(0.0, 0.0)
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TriboCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_cmd(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

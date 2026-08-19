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
import os
import statistics
import tempfile
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# Default source-tree path of the per-robot override file that bringup.launch.py
# loads (see config/motor_calib.yaml). Not the install-share copy: motor_calib
# writes the SOURCE and the converge script rebuilds so install picks it up.
DEFAULT_CALIB_YAML = os.path.expanduser(
    "~/tribo_ws/src/tribo/tribo_bringup/config/motor_calib.yaml"
)


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


# =============================================================================
#  진단(자동) — 순수 함수. rclpy 의존 없음 → 오프라인 단위테스트 가능.
# =============================================================================

# verdict → 권장 문구 매핑. FAIL_SIGN 은 {X} 에 문제 모터 번호를 채운다.
RECOMMEND = {
    "FAIL_STUCK": "pwm_min_percent 상향 또는 배선/기계 저항 점검",
    "FAIL_SIGN": "bringup.yaml invert_m{X} 토글",
    "NOT_CONVERGED": "비선형/포화 가능성, 재실행 또는 tol 완화 검토",
    "PASS": "직진성 실주행 점검 후 커밋",
}


@dataclass
class Diagnosis:
    verdict: str            # PASS | FAIL_STUCK | FAIL_SIGN | NOT_CONVERGED
    imbalance: float        # (max_r - min_r) / mean_r
    flags: List[str]        # 예: ["STUCK m2", "SIGN_FLIP m3"]
    recommendation: str     # 사람이 읽는 권장 문구
    min_index: int          # |rate| 최저 모터의 0-based 인덱스(=보정 기준)
    abs_rates: List[float]  # [|rate_i|]


def diagnose(
    rates: List[float],
    signs: List[int],
    gains: List[float],
    tol: float,
    stuck_floor_tps: float = 30.0,
    stuck_frac: float = 0.3,
) -> Diagnosis:
    """forward 구간 4모터 tick rate/부호/게인으로 직진 캘리브 상태를 진단.

    rclpy 비의존 순수 함수(오프라인 테스트용).

    규칙:
      - r = [|rate_i|], max_r/min_r/mean_r (mean guard>0)
      - imbalance = (max_r - min_r) / mean_r,  converged = imbalance < tol
      - STUCK m{i}: r_i < max(stuck_floor_tps, stuck_frac*max_r)
      - SIGN_FLIP m{i}: forward 다수 부호와 반대(0 은 STUCK 로 처리)
      - verdict 우선순위: FAIL_STUCK > FAIL_SIGN > (converged? PASS : NOT_CONVERGED)
    """
    n = len(rates)
    abs_rates = [abs(float(r)) for r in rates]
    max_r = max(abs_rates) if abs_rates else 0.0
    min_r = min(abs_rates) if abs_rates else 0.0
    mean_r = statistics.mean(abs_rates) if abs_rates else 0.0
    imbalance = (max_r - min_r) / mean_r if mean_r > 1e-9 else 0.0
    converged = imbalance < tol
    min_index = abs_rates.index(min_r) if abs_rates else 0

    flags: List[str] = []

    # STUCK: 거의 안 도는 모터
    stuck_thr = max(stuck_floor_tps, stuck_frac * max_r)
    stuck = [i for i in range(n) if abs_rates[i] < stuck_thr]
    for i in stuck:
        flags.append(f"STUCK m{i + 1}")

    # SIGN_FLIP: 다수 부호와 반대인 모터(부호 0=STUCK 는 제외)
    pos = sum(1 for s in signs if s > 0)
    neg = sum(1 for s in signs if s < 0)
    majority = 1 if pos >= neg else -1
    sign_flip = [i for i in range(n) if signs[i] != 0 and signs[i] != majority]
    for i in sign_flip:
        flags.append(f"SIGN_FLIP m{i + 1}")

    # verdict 우선순위
    if stuck:
        verdict = "FAIL_STUCK"
    elif sign_flip:
        verdict = "FAIL_SIGN"
    elif converged:
        verdict = "PASS"
    else:
        verdict = "NOT_CONVERGED"

    if verdict == "FAIL_SIGN":
        motors = "/".join(str(i + 1) for i in sign_flip)
        recommendation = RECOMMEND["FAIL_SIGN"].replace("{X}", motors)
    else:
        recommendation = RECOMMEND[verdict]

    return Diagnosis(
        verdict=verdict,
        imbalance=imbalance,
        flags=flags,
        recommendation=recommendation,
        min_index=min_index,
        abs_rates=abs_rates,
    )


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
        # ⚠️ 0.20 은 낮다. duty = pwm_min + (vx/max_lin_vel)*gain*(pwm_max-pwm_min) 이라
        #    pwm_min(20%) 이 고정 오프셋으로 깔린다. vx=0.20, max_lin_vel=0.85 이면
        #    gain 이 좌우하는 몫은 duty 의 1/3 도 안 되고, 게다가 duty→속도 응답이
        #    오목해서 저듀티에서 더 둔하다. 2026-08-19 실측에서 gain_m4 를 0.50→0.42
        #    (-16%) 로 낮췄는데 실제 tick rate 는 0.5% 밖에 안 줄었다. 이 상태로는
        #    수렴 루프가 보정량의 ~1/7 만 반영해 tol 아래로 못 내려간다.
        #    0.35 는 결과 duty 가 Nav2 동작대역(38~48%)에 들어오도록 고른 값이다.
        self.declare_parameter("test_vx", 0.35)          # m/s
        self.declare_parameter("test_wz", 1.00)          # rad/s (command)
        # ⚠️ 이 캘리브는 "바퀴를 든 상태"로 돌린다. 무부하 바퀴는 (a) 정상속도에
        #    도달하는 데 시간이 걸리고 (b) 명령을 끊어도 관성으로 한참 공회전한다.
        #    2026-08-19 실측(82mm→100mm 교체 후 5회 반복)에서 run_time=1.5s 는
        #    가속 구간만 재고 끝나 stop 구간 코스팅 속도가 주행 구간 평균의 2.5배로
        #    나왔다. 즉 gain 이 "정상속도"가 아니라 "가속도"에 맞춰 피팅되고 있었다.
        #    다음 구간이 시작될 때 이전 회전이 남아 있어 backward/rotate_right 도 오염된다.
        #    구간을 늘려 정상속도가 평균을 지배하게 하고, stop 구간에서 실제로 멈추게 한다.
        #    검증법: stop 구간 rate 가 0 근처로 떨어지는지 로그로 확인할 것.
        self.declare_parameter("run_time", 4.0)          # seconds each motion segment
        self.declare_parameter("stop_time", 3.0)         # seconds between segments
        self.declare_parameter("pub_rate", 20.0)         # cmd_vel publish rate (Hz)

        # current bringup settings (for computing "new = current * ratio")
        self.declare_parameter("current_turn_scale", 1.0)
        self.declare_parameter("current_gain_m1", 1.0)
        self.declare_parameter("current_gain_m2", 1.0)
        self.declare_parameter("current_gain_m3", 1.0)
        self.declare_parameter("current_gain_m4", 1.0)
        self.declare_parameter("current_gain_left_rev_factor", 1.0)
        self.declare_parameter("current_gain_right_rev_factor", 1.0)

        # ---- convergence / write-back (방식 A: 측정→기록→재시작 반복) ----
        self.declare_parameter("write_yaml", False)
        self.declare_parameter("calib_yaml_path", DEFAULT_CALIB_YAML)
        self.declare_parameter("converge_tol", 0.05)
        self.declare_parameter("bringup_node_name", "tribo_bringup")

        self.write_yaml = bool(self.get_parameter("write_yaml").value)
        self.calib_yaml_path = str(self.get_parameter("calib_yaml_path").value)
        self.converge_tol = float(self.get_parameter("converge_tol").value)
        self.bringup_node_name = str(self.get_parameter("bringup_node_name").value)

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

        # Live gain baseline: query the RUNNING bringup node so every re-calibration
        # pass corrects relative to the gains actually applied right now (방식 A의
        # 재시작 반복에서 항상 현재값 기준). Falls back to the current_gain_m* params
        # if the service is unavailable. Done BEFORE timers start so the motion
        # state machine does not advance during the blocking service call.
        self.live_gain = self._fetch_live_gains()

        # timers
        self._pub_timer = self.create_timer(1.0 / self.pub_rate, self._tick_publish)
        self._seq_timer = self.create_timer(0.05, self._tick_sequence)  # state machine

    def _fetch_live_gains(self) -> List[float]:
        """실행 중인 bringup 노드의 gain_m1~m4 를 파라미터 서비스로 조회.

        성공 시 [m1,m2,m3,m4] float 리스트, 실패 시 current_gain_m* 폴백값.
        """
        names = ["gain_m1", "gain_m2", "gain_m3", "gain_m4"]
        try:
            # rclpy Jazzy: 클래스명은 AsyncParameterClient (단수). import 실패 시에도
            # 아래 except 로 폴백되도록 try 안에서 import 한다.
            from rclpy.parameter_client import AsyncParameterClient

            client = AsyncParameterClient(self, self.bringup_node_name)
            if not client.wait_for_services(timeout_sec=5.0):
                self.get_logger().warning(
                    f"[live-gain] '{self.bringup_node_name}' 파라미터 서비스 없음 "
                    f"-> current_gain_m* 폴백 {['%.3f' % g for g in self.cur_gain]}"
                )
                return self.cur_gain[:]
            future = client.get_parameters(names)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if not future.done() or future.result() is None:
                self.get_logger().warning(
                    "[live-gain] 조회 타임아웃 -> current_gain_m* 폴백 "
                    f"{['%.3f' % g for g in self.cur_gain]}"
                )
                return self.cur_gain[:]
            vals = future.result().values  # rcl_interfaces/ParameterValue[]
            gains = [float(v.double_value) for v in vals]
            if len(gains) != 4 or any(g == 0.0 for g in gains):
                self.get_logger().warning(
                    f"[live-gain] 유효하지 않은 응답 {gains} -> current_gain_m* 폴백"
                )
                return self.cur_gain[:]
            self.get_logger().info(
                f"[live-gain] running bringup gains = "
                f"m1={gains[0]:.3f} m2={gains[1]:.3f} m3={gains[2]:.3f} m4={gains[3]:.3f}"
            )
            return gains
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(
                f"[live-gain] 조회 실패({e}) -> current_gain_m* 폴백 "
                f"{['%.3f' % g for g in self.cur_gain]}"
            )
            return self.cur_gain[:]

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _cb_enc(self, msg: Int32MultiArray):
        if len(msg.data) < 5:
            return
        # msg.data = [ms, e1, e2, e3, e4]
        e = [int(msg.data[1]), int(msg.data[2]), int(msg.data[3]), int(msg.data[4])]
        t = self._now()

        # if a segment is active, keep last encoder snapshot
        if self._seg_active:
            self._seg_end_enc = e

        # ⚠️ 2026-08-19 버그 수정. 예전에는 아래 대입이 `if self._enc_prev is None:`
        #    안에 있어서 _enc_prev 가 "노드 기동 후 첫 샘플"에 영구히 고정됐다.
        #    _start_segment 가 _seg_start_enc = _enc_prev 로 잡으므로, 모든 구간의
        #    d_ticks 가 "구간 델타"가 아니라 "실행 시작부터의 누적"이 되고,
        #    rate = 누적 / 구간길이 라는 무의미한 값이 나왔다.
        #    증상: stop 구간 rate 가 주행 구간보다 크게 나오고(stop3 가 rotate_left
        #    의 2.5배), backward 가 forward 와 같은 부호로 찍히며, rotate_right 의
        #    좌우 부호가 rotate_left 와 동일하게 보인다. 그 결과 아래 값이 전부 쓰레기:
        #      - gain_left_rev_factor / gain_right_rev_factor
        #      - "전진인데 좌/우 tick 부호가 기대와 다릅니다" 경고
        #    forward 만 우연히 정상이었다(직전 stop0 에서 아무것도 안 움직여
        #    시작 누적이 0 이므로 누적 == 구간 델타). 즉 과거 gain_m* 결과 자체는
        #    유효했고, 나머지 추천값만 오염돼 있었다.
        self._enc_prev = e
        self._enc_prev_t = t

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

    def _write_calib_yaml(self, gains: List[float]):
        """계산된 gain_m1~m4 를 calib_yaml_path 에 원자적으로 병합 기록.

        - 기존 파일의 다른 키(tribo_bringup/ros__parameters 하위 포함)는 보존.
        - gain_m1~4 만 갱신. 파일이 없으면 새로 생성.
        - 임시파일에 쓰고 os.replace 로 원자적 교체.
        """
        path = self.calib_yaml_path
        data: Dict = {}
        try:
            if os.path.exists(path):
                with open(path, "r") as f:
                    loaded = yaml.safe_load(f)
                if isinstance(loaded, dict):
                    data = loaded
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(
                f"[write_yaml] 기존 파일 읽기 실패({e}), 새로 생성합니다."
            )
            data = {}

        node = data.get("tribo_bringup")
        if not isinstance(node, dict):
            node = {}
            data["tribo_bringup"] = node
        params = node.get("ros__parameters")
        if not isinstance(params, dict):
            params = {}
            node["ros__parameters"] = params

        params["gain_m1"] = round(float(gains[0]), 4)
        params["gain_m2"] = round(float(gains[1]), 4)
        params["gain_m3"] = round(float(gains[2]), 4)
        params["gain_m4"] = round(float(gains[3]), 4)

        header = (
            "# AUTO-GENERATED by motor_calib.py — per-robot, DO NOT COMMIT\n"
            "# gain_m1~m4: 모터별 PWM 배수. bringup.launch.py 가 bringup.yaml 뒤에\n"
            "# 로드하여 오버라이드. 재생성: scripts/motor_calib_converge.sh (바퀴 들고).\n"
        )
        try:
            d = os.path.dirname(path) or "."
            os.makedirs(d, exist_ok=True)
            fd, tmp = tempfile.mkstemp(prefix=".motor_calib.", suffix=".yaml", dir=d)
            try:
                with os.fdopen(fd, "w") as f:
                    f.write(header)
                    yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
                os.replace(tmp, path)
            except Exception:
                try:
                    os.unlink(tmp)
                except OSError:
                    pass
                raise
            self.get_logger().info(
                f"[write_yaml] gain 기록됨 -> {path} : "
                f"m1={params['gain_m1']} m2={params['gain_m2']} "
                f"m3={params['gain_m3']} m4={params['gain_m4']}"
            )
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"[write_yaml] 파일 기록 실패: {e}")

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

        # 1) Straight balance (forward) -> per-motor gain (방식 A)
        if fwd is not None:
            # ---- Forward per-motor gain (방식 A) ----
            # 목표: 4모터 tick rate 를 그중 '최저' 모터에 맞춤.
            #   new_gain[i] = live_gain[i] * (min_rate / |rate_i|)
            #
            # ⚠️ 이 식만 쓰면 계수가 항상 <=1 이라 게인이 내려가기만 한다. 빠른 모터를
            #    누르면 다음 반복에서 최저 모터의 순위가 바뀌고, 그러면 직전의 최저
            #    모터에도 <1 이 곱해진다. 반복할수록 4개 게인이 모두 0 을 향해 깎여
            #    로봇이 캘리브를 돌릴 때마다 약해진다 (실측: 최대 게인 0.73 -> 0.619).
            #    토크가 사라지면 제자리 회전이 스크럽 마찰을 못 이겨 stall 하고,
            #    그걸 rotate_pwm_min 으로 억지로 덮으면 전류가 튀어 전원이 트립된다.
            #
            # 그래서 균형(비율)은 유지한 채 최대 게인이 1.0 이 되도록 재정규화한다.
            # 적어도 한 모터는 명령 PWM 을 100% 받으므로 토크를 버리지 않는다.
            abs_rates = [abs(r) for r in fwd.rates]
            eps = 1e-6
            min_rate = min(abs_rates)
            max_rate = max(abs_rates)
            mean_rate = statistics.mean(abs_rates) if abs_rates else 0.0

            new_gain = self.live_gain[:]
            if min_rate < eps or any(r < eps for r in abs_rates):
                gain_ok = False
                self.get_logger().warning(
                    "[직진] 어떤 모터 tick rate 가 0 에 가깝습니다. gain 재계산을 건너뜁니다 "
                    "(pwm_min_percent/기계저항 확인). 현재 게인을 유지합니다."
                )
            else:
                gain_ok = True
                raw_gain = [
                    self.live_gain[i] * (min_rate / abs_rates[i]) for i in range(4)
                ]
                g_max = max(raw_gain)
                if g_max > eps:
                    new_gain = [g / g_max for g in raw_gain]   # 비율 유지, 최대=1.0
                else:
                    new_gain = raw_gain
                    self.get_logger().warning(
                        "[직진] 재정규화 실패(모든 게인이 0 에 가까움). 원값을 유지합니다."
                    )

            # 불균형 지표: (max-min)/mean (0=완전 균형). converge_tol 미만이면 수렴.
            imbalance = (max_rate - min_rate) / mean_rate if mean_rate > eps else 0.0

            self.get_logger().info(
                f"[직진 보정(Forward)] rates(t/s)={['%.1f' % r for r in fwd.rates]} "
                f"L={fwd.left_rate:.1f} R={fwd.right_rate:.1f}\n"
                f"  - live_gain={['%.3f' % g for g in self.live_gain]}\n"
                f"  - 추천 gain_m*: "
                f"m1={new_gain[0]:.3f}, m2={new_gain[1]:.3f}, "
                f"m3={new_gain[2]:.3f}, m4={new_gain[3]:.3f}"
            )

            # 수렴 판정 한 줄 (오케스트레이션 스크립트가 grep). 주의: NOT_CONVERGED 는
            # CONVERGED 를 부분문자열로 포함하므로 스크립트는 NOT_CONVERGED 를 먼저 검사.
            if imbalance < self.converge_tol:
                self.get_logger().info(f"CONVERGED imbalance={imbalance:.4f}")
            else:
                self.get_logger().info(f"NOT_CONVERGED imbalance={imbalance:.4f}")

            # ---- 진단(자동): 순수 함수로 판정 → per-run 리포트 ----
            diag = diagnose(fwd.rates, fwd.signs, self.live_gain, self.converge_tol)
            k = diag.min_index + 1
            block = [
                "========== 진단(자동) ==========",
                f"판정: {diag.verdict}  imbalance={diag.imbalance:.4f} "
                f"(tol={self.converge_tol:.4f})",
                f"rate(ticks/s): m1={fwd.rates[0]:.1f} m2={fwd.rates[1]:.1f} "
                f"m3={fwd.rates[2]:.1f} m4={fwd.rates[3]:.1f}  (기준=최저 m{k})",
            ]
            if diag.flags:
                block.append(f"[경고] {', '.join(diag.flags)}")
            block.append(
                f"갱신 gain: m1={new_gain[0]:.3f} m2={new_gain[1]:.3f} "
                f"m3={new_gain[2]:.3f} m4={new_gain[3]:.3f}"
            )
            block.append(f"권장: {diag.recommendation}")
            # 스크립트 파싱용 머신 판정 라인
            block.append(f"VERDICT: {diag.verdict}")
            self.get_logger().info("\n".join(block))

            # write-back: 소스 config/motor_calib.yaml 에 gain 병합 기록
            if self.write_yaml:
                if gain_ok:
                    self._write_calib_yaml(new_gain)
                else:
                    self.get_logger().warning(
                        "[write_yaml] gain 재계산 불가로 파일을 갱신하지 않았습니다."
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

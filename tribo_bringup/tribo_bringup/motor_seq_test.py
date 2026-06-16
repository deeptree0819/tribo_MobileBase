#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터 순차 점검 도구 (standalone, ROS 노드 아님)

목적
  4개 모터를 한 번에 하나씩 순서대로 저속 구동해서, 각 바퀴가
  - 도는지 / 안 도는지 / 공회전(엔코더는 도는데 바닥 접지 X)하는지
  를 육안 + 엔코더 틱으로 점검하기 위한 진단 도구.

설계
  - bringup 노드가 시리얼 포트(/dev/tribo_base)를 점유하므로, 이 도구는
    standalone 으로 직접 tribolib.TriboBase 를 열어 set_wheel_pwm 으로
    모터를 개별 제어한다. => 실행 전 bringup 을 반드시 멈춰야 한다
    (포트 동시 점유 충돌 방지). 실행 시 포트 열기 실패하면 그 사실을 알려준다.
  - ROS 토픽/서비스/파라미터 없음. 순수 시리얼.
  - 게인 보정은 bringup(cb_cmd)에만 있는 ROS 레이어이며 tribolib 에는 없다.
    이 도구는 기본 raw PWM(게인 미적용) — 모터 간 "원래" 속도 차이 비교에 유리.
    --use-gain 옵션으로 per-motor 게인을 곱해볼 수도 있다(비교용).

모터 매핑 (Rosmaster, bringup.py / odom 과 동일)
  m1 = FL (앞-좌, front-left)
  m2 = RL (뒤-좌, rear-left)
  m3 = RR (뒤-우, rear-right)
  m4 = FR (앞-우, front-right)
  좌측 = m1,m2 / 우측 = m3,m4

부호
  set_wheel_pwm 인자는 "로봇 기준 양수=전진" (tribolib 내부 MOTOR_POLARITY=-1
  로 보드 부호를 뒤집어 줌). 따라서 여기서 +PWM = 전진, -PWM = 후진.

안전
  - 저PWM 기본값(30%). 시작/종료/예외/Ctrl-C 시 항상 4모터 0 으로 정지.
  - 항상 "바퀴 들고" 테스트할 것.
"""

import argparse
import os
import signal
import sys
import time

# 패키지로 설치되면 상대 import, 직접 파일 실행이면 같은 디렉터리에서 import
try:
    from .tribolib import TriboBase
except ImportError:  # python3 motor_seq_test.py 직접 실행 대비
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from tribolib import TriboBase


# bringup.launch.py 의 _resolve_port 와 동일한 규칙: udev 심링크 우선, by-id fallback.
def resolve_base_port(preferred="/dev/tribo_base", *by_id_substrings):
    if not by_id_substrings:
        by_id_substrings = ("1a86_USB_Serial",)
    if os.path.exists(preferred):
        return preferred
    by_id_dir = "/dev/serial/by-id"
    try:
        for name in sorted(os.listdir(by_id_dir)):
            if any(s in name for s in by_id_substrings):
                return os.path.join(by_id_dir, name)
    except OSError:
        pass
    return preferred


# 모터 인덱스(1~4) -> (라벨, 한글 위치). set_wheel_pwm 인자 위치와 1:1.
MOTOR_LABELS = {
    1: ("FL", "앞-좌 (front-left)"),
    2: ("RL", "뒤-좌 (rear-left)"),
    3: ("RR", "뒤-우 (rear-right)"),
    4: ("FR", "앞-우 (front-right)"),
}

# bringup.yaml 기본 per-motor gain (이 도구의 --use-gain 비교 옵션용. 보정값은 건드리지 않음.)
DEFAULT_GAIN = {1: 0.98, 2: 0.98, 3: 1.0, 4: 1.0}


def build_pwm_tuple(motor_idx: int, pwm: int):
    """motor_idx(1~4) 만 pwm, 나머지는 0 인 (m1,m2,m3,m4) 튜플."""
    vals = [0, 0, 0, 0]
    vals[motor_idx - 1] = int(pwm)
    return tuple(vals)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Tribo 4모터 순차 점검 도구 (standalone, bringup 정지 후 실행)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", default=None,
                        help="시리얼 포트. 미지정 시 /dev/tribo_base -> by-id 자동해석")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--pwm", type=int, default=30,
                        help="구동 PWM 퍼센트 절댓값 (1~100). 저속 권장.")
    parser.add_argument("--run", type=float, default=3.0,
                        help="모터 1개당 구동 시간(초)")
    parser.add_argument("--gap", type=float, default=1.0,
                        help="다음 모터로 넘어가기 전 정지 시간(초)")
    parser.add_argument("--order", default="1,2,3,4",
                        help="구동 순서. 콤마구분 모터인덱스 (예: 1,4,2,3)")
    parser.add_argument("--direction", choices=["fwd", "rev", "both"], default="fwd",
                        help="fwd=정방향만, rev=역방향만, both=정/역 둘 다(각각 run초)")
    parser.add_argument("--cycles", type=int, default=1,
                        help="전체 시퀀스 반복 횟수")
    parser.add_argument("--use-gain", action="store_true",
                        help="per-motor 게인을 곱해 PWM 적용(기본은 raw, 게인 미적용)")
    args = parser.parse_args(argv)

    pwm_mag = max(1, min(100, abs(args.pwm)))
    try:
        order = [int(x) for x in args.order.split(",") if x.strip()]
    except ValueError:
        print("ERROR: --order 는 콤마구분 정수여야 합니다 (예: 1,2,3,4)")
        return 2
    for m in order:
        if m not in MOTOR_LABELS:
            print(f"ERROR: 잘못된 모터 인덱스 {m} (1~4만 허용)")
            return 2

    port = args.port or resolve_base_port()

    print("=" * 64)
    print(" Tribo 모터 순차 점검 도구")
    print(f"  port      = {port}")
    print(f"  pwm       = {pwm_mag}%  (use_gain={args.use_gain})")
    print(f"  run/gap   = {args.run}s / {args.gap}s,  dir={args.direction},  cycles={args.cycles}")
    print(f"  order     = {order}  (m1=FL, m2=RL, m3=RR, m4=FR)")
    print("  ⚠ 바퀴를 들고(공중에서) 테스트하세요. bringup 은 멈춘 상태여야 합니다.")
    print("=" * 64)

    try:
        base = TriboBase(port=port, baudrate=args.baud, car_type=TriboBase.CARTYPE_X3)
    except Exception as e:
        print(f"\nERROR: 시리얼 포트 열기 실패 ({port}): {e}")
        print("  -> bringup 이 아직 포트를 점유 중일 수 있습니다. 먼저 bringup 을 멈추세요:")
        print("     pkill -f tribo_bringup.bringup")
        return 1

    stop_all = lambda: base.set_wheel_pwm(0, 0, 0, 0)

    def cleanup(*_):
        try:
            stop_all()
            time.sleep(0.05)
            stop_all()
        finally:
            try:
                base.close()
            except Exception:
                pass

    # SIGTERM 도 안전 정지
    signal.signal(signal.SIGTERM, lambda *_: (cleanup(), sys.exit(0)))

    # 엔코더 자동보고 수신 시작
    base.start_background_reader()
    time.sleep(0.1)
    base.set_auto_report(True, persist=False)
    time.sleep(0.2)
    stop_all()

    if args.direction == "fwd":
        dirs = [(+1, "정방향")]
    elif args.direction == "rev":
        dirs = [(-1, "역방향")]
    else:
        dirs = [(+1, "정방향"), (-1, "역방향")]

    def drive_phase(step_label, motor_idx, sign, dir_name):
        lab, pos = MOTOR_LABELS[motor_idx]
        gain = DEFAULT_GAIN[motor_idx] if args.use_gain else 1.0
        pwm = int(round(sign * pwm_mag * gain))
        e_start = base.get_encoders()
        print(f"\n[{step_label}] m{motor_idx} {lab} {pos}  {dir_name}  PWM={pwm:+d}%"
              + (f"  (gain x{gain})" if args.use_gain else ""))
        base.set_wheel_pwm(*build_pwm_tuple(motor_idx, pwm))

        t0 = time.time()
        last_print = 0.0
        while time.time() - t0 < args.run:
            time.sleep(0.1)
            el = time.time() - t0
            if el - last_print >= 0.5:
                last_print = el
                e = base.get_encoders()
                d = e[motor_idx - 1] - e_start[motor_idx - 1]
                others = [e[i] - e_start[i] for i in range(4) if i != motor_idx - 1]
                print(f"    t={el:4.1f}s  m{motor_idx} 틱Δ={d:+6d}   "
                      f"다른모터 틱Δ={others}")

        stop_all()
        e_end = base.get_encoders()
        d_target = e_end[motor_idx - 1] - e_start[motor_idx - 1]
        verdict = ("회전 확인 OK" if abs(d_target) > 5
                   else "틱 변화 거의 없음 -> 모터 안 돔/엔코더 미연결 의심")
        print(f"    => m{motor_idx} 최종 틱Δ={d_target:+d}  [{verdict}]")
        if args.gap > 0:
            time.sleep(args.gap)

    rc = 0
    try:
        for c in range(args.cycles):
            print(f"\n########## CYCLE {c + 1}/{args.cycles} ##########")
            for sign, dir_name in dirs:
                for i, motor_idx in enumerate(order):
                    step = f"{i + 1}/{len(order)}"
                    drive_phase(step, motor_idx, sign, dir_name)
        print("\n시퀀스 완료. 모든 모터 정지.")
    except KeyboardInterrupt:
        print("\n[Ctrl-C] 중단 -> 모든 모터 정지.")
    except Exception as e:
        print(f"\nERROR 발생: {e} -> 모든 모터 정지.")
        rc = 1
    finally:
        cleanup()
    return rc


if __name__ == "__main__":
    sys.exit(main())

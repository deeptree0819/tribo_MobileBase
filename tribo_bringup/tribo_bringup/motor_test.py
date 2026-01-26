#!/usr/bin/env python3
# coding: utf-8

"""
motor_mapping_test.py

각 모터(m1~m4)를 개별적으로 돌려보면서:
  - 이 모터가 물리적으로 어디에 붙어있는지(FL, RL, RR, FR)
  - +PWM일 때 어떤 방향으로 도는지
  - 엔코더가 +로 증가하는지, -로 감소하는지

를 확인하기 위한 스크립트.
"""

import time
from tribolib import TriboBase


def print_enc(bot: TreeboBase, tag: str):
    e1, e2, e3, e4 = bot.get_encoders()
    print(f"[{tag}] ENC = ({e1}, {e2}, {e3}, {e4})")
    return (e1, e2, e3, e4)


def test_one_motor(bot: TreeboBase, label: str, m1: int, m2: int, m3: int, m4: int,
                   duration: float = 1.5):
    print("\n======================================")
    print(f"[TEST] {label}")
    print(f"  PWM = (m1={m1}, m2={m2}, m3={m3}, m4={m4})")
    print("  ※ 이 때 실제 어떤 바퀴가 도는지, 앞/뒤 어느 방향으로 도는지 눈으로 확인하세요.")
    start = print_enc(bot, f"{label} START")

    bot.set_wheel_pwm(m1, m2, m3, m4)
    time.sleep(duration)
    bot.set_wheel_pwm(0, 0, 0, 0)
    time.sleep(0.3)

    end = print_enc(bot, f"{label} END")
    de = (end[0] - start[0], end[1] - start[1], end[2] - start[2], end[3] - start[3])
    print(f"[{label}] ΔENC = {de}")
    print("======================================")


def main():
    port = "/dev/ttyUSB0"  # 필요시 수정
    car_type = TreeboBase.CARTYPE_X3

    print("=== Motor Mapping Test 시작 ===")
    bot = TreeboBase(port=port, car_type=car_type, debug=True)

    bot.start_background_reader()
    time.sleep(0.1)
    bot.set_auto_report(True, persist=False)
    time.sleep(0.2)

    fw = bot.get_firmware_version()
    print(f"[INFO] FW version = {fw:.2f}")
    bot.beep(80)
    time.sleep(0.3)

    # m1만 +40
    test_one_motor(bot, "M1 +40", 40, 0, 0, 0)
    # m1만 -40
    test_one_motor(bot, "M1 -40", -40, 0, 0, 0)

    # m2만 +40
    test_one_motor(bot, "M2 +40", 0, 40, 0, 0)
    # m2만 -40
    test_one_motor(bot, "M2 -40", 0, -40, 0, 0)

    # m3만 +40
    test_one_motor(bot, "M3 +40", 0, 0, 40, 0)
    # m3만 -40
    test_one_motor(bot, "M3 -40", 0, 0, -40, 0)

    # m4만 +40
    test_one_motor(bot, "M4 +40", 0, 0, 0, 40)
    # m4만 -40
    test_one_motor(bot, "M4 -40", 0, 0, 0, -40)

    bot.set_wheel_pwm(0, 0, 0, 0)
    time.sleep(0.2)
    bot.close()
    print("=== Motor Mapping Test 종료 ===")


if __name__ == "__main__":
    main()

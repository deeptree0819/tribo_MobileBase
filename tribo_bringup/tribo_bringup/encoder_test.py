#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from Rosmaster_Lib import Rosmaster

PORT = "/dev/myserial"
CAR_TYPE = 1
DEBUG = True

def main():
    bot = Rosmaster(car_type=CAR_TYPE, com=PORT.strip(), debug=DEBUG)
    bot.create_receive_threading()
    bot.set_auto_report_state(True)
    time.sleep(0.3)

    print("[INFO] Battery:", bot.get_battery_voltage())

    # beep
    bot.set_beep(100)
    time.sleep(0.2)
    bot.set_beep(0)

    def read_enc(tag=""):
        enc = bot.get_motor_encoder()
        print(f"{tag} enc={enc}")
        return enc

    print("\n[TEST] Encoder should change while motors run")
    e0 = read_enc("start")

    # 더 강하게 (20이 약할 수 있음)
    pwm = 40
    print(f"\n[RUN] set_motor({pwm},{pwm},{pwm},{pwm}) for 2s, printing enc every 0.2s")
    bot.set_motor(pwm, pwm, pwm, pwm)

    t_end = time.time() + 2.0
    last = None
    while time.time() < t_end:
        last = read_enc("run ")
        time.sleep(0.2)

    bot.set_motor(0, 0, 0, 0)
    time.sleep(0.3)

    e1 = read_enc("stop")

    print("\n[RESULT]")
    print("  start:", e0)
    print("  stop :", e1)
    print("  diff :", tuple(e1[i]-e0[i] for i in range(4)))

if __name__ == "__main__":
    main()

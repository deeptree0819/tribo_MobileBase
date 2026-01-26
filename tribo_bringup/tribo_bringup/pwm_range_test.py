import serial
import time
import sys

PORT = "/dev/ttyACM0"   # Windows면 "COMx"
BAUD = 115200

START = 1800
END   = 3500

STEP_FINE = 50
DO_COARSE_FIRST = True
STEP_COARSE = 200

DWELL_FINE = 2.5
DWELL_COARSE = 3.5

RAMP_DOWN_STEP = 100
RAMP_DOWN_DWELL = 0.8

# ✅ STM32가 UART로 "ENC ..."를 보내는 경우 켜기
READ_BACK = True
READ_BACK_WINDOW_SEC = 0.3


def send(ser, m1, m2=0, m3=0, m4=0):
    cmd = f"m {m1} {m2} {m3} {m4}\n"
    ser.write(cmd.encode("ascii"))
    print(f">>> {cmd.strip()}")


def read_back_lines(ser, window_sec=0.3):
    """
    UART로 들어오는 줄을 읽어서:
      - ENC 라인이면 파싱해서 (ms,e1,e2,e3,e4) 저장
      - 다른 줄이면 그대로 출력(디버그용)
    반환: 마지막으로 받은 ENC 튜플 또는 None
    기대 포맷: "ENC <ms> <e1> <e2> <e3> <e4>"
    """
    last_enc = None
    t0 = time.time()

    while time.time() - t0 < window_sec:
        line = ser.readline()
        if not line:
            continue

        text = line.decode(errors="ignore").strip()
        if not text:
            continue

        if text.startswith("ENC"):
            parts = text.split()
            if len(parts) >= 6:
                try:
                    ms = int(parts[1])
                    e1 = int(parts[2])
                    e2 = int(parts[3])
                    e3 = int(parts[4])
                    e4 = int(parts[5])
                    last_enc = (ms, e1, e2, e3, e4)
                except ValueError:
                    print("<<< [ENC PARSE ERR]", text)
            else:
                print("<<< [ENC FORMAT?]", text)
        else:
            print("<<<", text)

    return last_enc


def sweep(ser, values, dwell):
    for v in values:
        send(ser, v, 0, 0, 0)
        time.sleep(dwell)

        if READ_BACK:
            enc = read_back_lines(ser, READ_BACK_WINDOW_SEC)
            if enc is not None:
                ms, e1, e2, e3, e4 = enc
                print(f"    [ENC@{v}] ms={ms} e=({e1},{e2},{e3},{e4})")
            else:
                print(f"    [ENC@{v}] (no ENC received)")


def ramp_down(ser, start_value):
    print("\n--- RAMP DOWN (safety) ---")
    v = start_value
    while v > 0:
        send(ser, v, 0, 0, 0)
        time.sleep(RAMP_DOWN_DWELL)
        if READ_BACK:
            read_back_lines(ser, 0.1)
        v -= RAMP_DOWN_STEP

    send(ser, 0, 0, 0, 0)
    time.sleep(0.5)
    if READ_BACK:
        read_back_lines(ser, 0.2)


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.2)
    except Exception as e:
        print("❌ Serial open failed:", e)
        sys.exit(1)

    print("✅ Serial opened")
    time.sleep(2.0)

    last_sent = 0

    try:
        send(ser, 0, 0, 0, 0)
        time.sleep(0.5)
        if READ_BACK:
            read_back_lines(ser, 0.3)

        if DO_COARSE_FIRST:
            print(f"\n--- COARSE SWEEP {START}~{END} step={STEP_COARSE} ---")
            coarse_values = list(range(START, END + 1, STEP_COARSE))
            sweep(ser, coarse_values, DWELL_COARSE)
            last_sent = coarse_values[-1]

        print(f"\n--- FINE SWEEP {START}~{END} step={STEP_FINE} ---")
        fine_values = list(range(START, END + 1, STEP_FINE))
        sweep(ser, fine_values, DWELL_FINE)
        last_sent = fine_values[-1]

        print(f"\n--- FINE SWEEP DOWN {END}~{START} step={STEP_FINE} ---")
        fine_down = list(range(END, START - 1, -STEP_FINE))
        sweep(ser, fine_down, DWELL_FINE)
        last_sent = fine_down[-1]

    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")

    finally:
        ramp_down(ser, max(last_sent, 0))
        ser.close()
        print("✅ Test done")


if __name__ == "__main__":
    main()

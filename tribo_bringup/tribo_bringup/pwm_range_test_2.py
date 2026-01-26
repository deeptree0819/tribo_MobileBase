import serial
import time
import sys

PORT = "/dev/ttyACM0"   # Windows: "COMx"
BAUD = 115200

def send_tick(ser, t):
    cmd = f"t {t}\n"
    ser.write(cmd.encode("ascii"))
    print(f">>> {cmd.strip()}")

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.2)
    except Exception as e:
        print("❌ Serial open failed:", e)
        sys.exit(1)

    time.sleep(2.0)

    # 0 -> 100 스윕 (당신이 측정한 최대가 ~96 근처였으니 100이면 충분)
    ticks = [0, 20, 40, 50, 60, 70, 80, 90, 96]
    for t in ticks:
        send_tick(ser, t)
        time.sleep(3.0)   # 각 단계에서 Live Expression 관찰 시간

    # 정지
    send_tick(ser, 0)
    time.sleep(1.0)
    ser.close()

if __name__ == "__main__":
    main()

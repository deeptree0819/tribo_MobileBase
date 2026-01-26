#!/usr/bin/env python3
import serial, time, sys, csv, statistics

PORT="/dev/ttyACM0"
BAUD=115200

PWM_LIST=[1800,2000,2200,2400,2600,2800,3000,3200,3400,3500]
REPEATS=4               # ✅ 각 점 반복 횟수
SETTLE_SEC=1.0
MEASURE_SEC=2.5
ENC_WINDOW_SEC=0.6

DROP_IF_BELOW=50.0      # ✅ ticks/s가 이보다 작으면 드롭아웃으로 간주(0/급락 제거)
CSV_PATH="motor_speed_calib.csv"

def send(ser,m1,m2,m3,m4):
    ser.write(f"m {m1} {m2} {m3} {m4}\n".encode("ascii"))
    ser.flush()

def read_enc_once(ser, window_sec=0.6):
    t0=time.time()
    last=None
    while time.time()-t0 < window_sec:
        line=ser.readline()
        if not line: 
            continue
        s=line.decode(errors="ignore").strip()
        if s.startswith("ENC "):
            parts=s.split()
            if len(parts)>=6:
                try:
                    ms=int(parts[1])
                    e=list(map(int, parts[2:6]))
                    last=(ms,e)
                except:
                    pass
    return last

def get_enc_blocking(ser, tries=10):
    for _ in range(tries):
        x=read_enc_once(ser, ENC_WINDOW_SEC)
        if x is not None:
            return x
    raise RuntimeError("ENC not received")

def speed(a, b):
    ms_a, ea = a
    ms_b, eb = b
    dt=(ms_b-ms_a)/1000.0
    if dt<=1e-6:
        return 0.0, [0.0]*4
    v=[ (eb[i]-ea[i]) / dt for i in range(4) ]  # ticks/s
    return dt, v

def measure_single_motor(ser, motor_idx, pwm):
    m=[0,0,0,0]
    m[motor_idx-1]=pwm
    send(ser,*m)
    time.sleep(SETTLE_SEC)
    a=get_enc_blocking(ser)
    time.sleep(MEASURE_SEC)
    b=get_enc_blocking(ser)
    dt, v = speed(a,b)
    send(ser,0,0,0,0)
    time.sleep(0.3)
    return dt, v[motor_idx-1]

def main():
    try:
        ser=serial.Serial(PORT, BAUD, timeout=0.2)
    except Exception as e:
        print("❌ Serial open failed:", e); sys.exit(1)

    print("✅ Serial opened")
    time.sleep(2.0)
    send(ser,0,0,0,0); time.sleep(0.3)

    with open(CSV_PATH,"w",newline="") as f:
        wr=csv.writer(f)
        wr.writerow(["pwm","motor","repeat","dt_sec","ticks_per_sec","is_dropout"])

        # 결과: pwm -> [m1,m2,m3,m4] (median)
        medians={}

        for pwm in PWM_LIST:
            print(f"\n=== PWM {pwm} ===")
            row=[None,None,None,None]

            for motor in [1,2,3,4]:
                samples=[]
                dropouts=0
                for r in range(REPEATS):
                    dt, v = measure_single_motor(ser, motor, pwm)
                    is_dropout = (abs(v) < DROP_IF_BELOW)
                    if is_dropout:
                        dropouts += 1
                    else:
                        samples.append(v)
                    wr.writerow([pwm,motor,r,round(dt,3),round(v,1),int(is_dropout)])
                    print(f"  motor{motor} rep{r}: {v:.1f} ticks/s (dt={dt:.2f}) {'DROP' if is_dropout else ''}")

                if len(samples)==0:
                    row[motor-1]=0.0
                    print(f"  motor{motor}: ❗all dropouts -> 0.0")
                else:
                    med=statistics.median(samples)
                    row[motor-1]=med
                    print(f"  motor{motor}: median={med:.1f} ticks/s (kept {len(samples)}/{REPEATS}, drop={dropouts})")

            medians[pwm]=tuple(row)

        print("\n=== MEDIAN TABLE (ticks/s) ===")
        for pwm in PWM_LIST:
            v=medians[pwm]
            print(f"PWM {pwm}: m1={v[0]:.1f}, m2={v[1]:.1f}, m3={v[2]:.1f}, m4={v[3]:.1f}")

    send(ser,0,0,0,0)
    ser.close()
    print(f"\n✅ Done. CSV saved: {CSV_PATH}")

if __name__=="__main__":
    main()

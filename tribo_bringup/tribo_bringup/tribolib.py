#!/usr/bin/env python3
# coding: utf-8

import struct
import time
import serial
import threading
from enum import IntEnum, auto
from typing import Tuple, Optional, Callable


class OpCode(IntEnum):
    """보드 펌웨어가 사용하는 기능 코드(opcode) 정의."""
    AUTO_REPORT      = 0x01
    BEEP             = 0x02
    PWM_SERVO        = 0x03
    PWM_SERVO_ALL    = 0x04
    RGB              = 0x05
    RGB_EFFECT       = 0x06

    REPORT_SPEED     = 0x0A
    REPORT_MPU_RAW   = 0x0B
    REPORT_IMU_ATT   = 0x0C
    REPORT_ENCODER   = 0x0D
    REPORT_ICM_RAW   = 0x0E
    RESET_STATE      = 0x0F

    MOTOR_PWM        = 0x10
    CAR_RUN          = 0x11
    MOTION           = 0x12
    SET_MOTOR_PID    = 0x13
    SET_YAW_PID      = 0x14
    SET_CAR_TYPE     = 0x15

    UART_SERVO       = 0x20
    UART_SERVO_ID    = 0x21
    UART_SERVO_TORQUE= 0x22
    ARM_CTRL         = 0x23
    ARM_OFFSET       = 0x24

    AKM_DEF_ANGLE    = 0x30
    AKM_STEER_ANGLE  = 0x31

    REQUEST_DATA     = 0x50
    VERSION          = 0x51

    RESET_FLASH      = 0xA0


class TriboFrameBuilder:
    """
    프레임 생성 유틸리티.
    프로토콜 포맷: [HEAD, DEVICE_ID, LEN, FUNC, ..., CHECKSUM]
    """

    HEAD = 0xFF

    def __init__(self, device_id: int, complement: int, func_code: int):
        self.device_id = device_id
        self.complement = complement
        self.func_code = func_code & 0xFF
        self.payload = []

    def add_u8(self, *values: int) -> "TreeboFrameBuilder":
        for v in values:
            self.payload.append(int(v) & 0xFF)
        return self

    def add_i8(self, *values: int) -> "TreeboFrameBuilder":
        for v in values:
            self.payload.append(struct.pack("b", int(v))[0])
        return self

    def add_i16(self, *values: int) -> "TreeboFrameBuilder":
        for v in values:
            b = struct.pack("<h", int(v))
            self.payload.extend(b)
        return self

    def build(self) -> bytes:
        # LEN = 전체 길이 - 1 (원 라이브러리와 동일 규칙)
        length = 3 + len(self.payload)  # DEVICE_ID, LEN, FUNC + payload
        frame = [self.HEAD, self.device_id, length, self.func_code]
        frame.extend(self.payload)

        checksum = (sum(frame) + self.complement) & 0xFF
        frame.append(checksum)
        return bytes(frame)


class TriboBase:
    """
    Tribo 전용 모터·IMU·엔코더·전압 제어/조회 드라이버 (단일 파일 버전).
    내부적으로 보드의 시리얼 프로토콜을 사용한다.
    """

    # 하드웨어 기본 설정
    _HEAD = 0xFF
    _DEVICE_ID = 0xFC
    _COMPLEMENT = 257 - _DEVICE_ID

    # 주요 차량 타입(필요한 것만 사용)
    CARTYPE_X3      = 0x01
    CARTYPE_X3_PLUS = 0x02
    CARTYPE_X1      = 0x04
    CARTYPE_R2      = 0x05

    _CAR_ADJUST_FLAG = 0x80
    MOTOR_POLARITY = -1  # 보드가 +를 '후진'으로 쓰고 있어서, 로봇 기준으로 의미를 뒤집는다.

    # 수신 스레드 상태
    _uart_thread_state = 0

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        car_type: int = CARTYPE_X3,
        delay: float = 0.002,
        debug: bool = False,
    ) -> None:
        self._debug = debug
        self._delay = delay
        self._car_type = car_type & 0xFF

        # 시리얼 포트 열기
        # timeout=0.1: read 가 주기적으로 리턴해 rx_loop 가 정지 신호(_uart_thread_state)를
        # 확인할 수 있게 한다(없으면 블로킹 read 라 close 시 스레드가 안 빠져나가 race 발생).
        self._ser = serial.Serial(port, baudrate, timeout=0.1)
        if self._ser.is_open:
            print(f"TriboBase: serial opened on {port} @ {baudrate}")
        else:
            raise RuntimeError(f"Failed to open serial port: {port}")

        # IMU, 속도, 엔코더, 전압 상태 캐시
        self._ax = self._ay = self._az = 0.0
        self._gx = self._gy = self._gz = 0.0
        self._mx = self._my = self._mz = 0.0
        self._vx = self._vy = self._vz = 0.0
        self._roll = self._pitch = self._yaw = 0.0
        self._enc_m1 = self._enc_m2 = self._enc_m3 = self._enc_m4 = 0
        self._battery_raw = 0

        # 버전, PID, 기타 상태
        self._version_major = 0
        self._version_minor = 0
        self._version = -1.0

        self._pid_index = 0
        self._kp_raw = self._ki_raw = self._kd_raw = 0

        # Ackermann, car_type read
        self._akm_default_angle = 100
        self._akm_angle_ready = False
        self._akm_servo_id = 0x01

        self._car_type_read = 0

        # arm 관련 간단 상태 (필요 시 확장)
        self._arm_ctrl_enable = True

        # 수신 스레드 핸들
        self._rx_thread: Optional[threading.Thread] = None

    # ---------------------- 공용 유틸 ----------------------

    def _build_frame(self, opcode: OpCode) -> TriboFrameBuilder:
        return TriboFrameBuilder(
            device_id=self._DEVICE_ID,
            complement=self._COMPLEMENT,
            func_code=int(opcode),
        )

    def _send(self, frame: bytes) -> None:
        if self._debug:
            print("TX:", list(frame))
        self._ser.write(frame)
        time.sleep(self._delay)

    # ---------------------- 수신 스레드 ----------------------

    def start_background_reader(self) -> None:
        """보드가 자동 보고하는 데이터를 읽는 백그라운드 스레드 시작."""
        if self._uart_thread_state != 0:
            return

        def rx_loop():
          try:
            self._ser.reset_input_buffer()
            while self._uart_thread_state != 2:
                header = self._ser.read(1)
                if not header:
                    continue
                if header[0] != self._HEAD:
                    continue

                dev_byte = self._ser.read(1)
                if not dev_byte:
                    continue
                dev = dev_byte[0]

                # 기본 프로토콜: 두 번째 헤더 바이트 = DEVICE_ID - 1
                if dev != self._DEVICE_ID - 1:
                    # 프로토콜 다르면 건너뜀
                    continue

                length_b = self._ser.read(1)
                func_b = self._ser.read(1)
                if not length_b or not func_b:
                    continue
                ext_len = length_b[0]
                func = func_b[0]

                # ext_len = 전체 - 1, 그 중 2바이트는 len, func
                data_len = ext_len - 2
                data = bytearray()
                checksum_calc = ext_len + func
                rx_checksum = 0

                while len(data) < data_len:
                    chunk = self._ser.read(1)
                    if not chunk:
                        break
                    data.append(chunk[0])
                    if len(data) == data_len:
                        rx_checksum = data[-1]
                    else:
                        checksum_calc += data[-1]

                if (checksum_calc % 256) != rx_checksum:
                    if self._debug:
                        print("RX checksum error")
                    continue

                self._handle_report(func, data)
          except Exception:
            # close() 로 시리얼이 닫히면 read 가 예외를 낸다. 종료(state==2) 중이면
            # 조용히 빠져나가고, 그 외의 예상치 못한 예외만 표면화한다.
            if self._uart_thread_state != 2:
                import traceback
                traceback.print_exc()

        self._rx_thread = threading.Thread(target=rx_loop, daemon=True)
        self._rx_thread.start()
        self._uart_thread_state = 1
        print("TriboBase: background reader started")

    # ---------------------- 수신 데이터 파싱 ----------------------

    def _handle_report(self, func: int, data: bytearray) -> None:
        """보드에서 자동으로 보내주는 데이터 패킷 처리."""
        try:
            opcode = OpCode(func)
        except ValueError:
            if self._debug:
                print("Unknown report opcode:", func)
            return

        if opcode == OpCode.REPORT_SPEED:
            # vx, vy, vz (mm/s → m/s)
            self._vx = struct.unpack("<h", data[0:2])[0] / 1000.0
            self._vy = struct.unpack("<h", data[2:4])[0] / 1000.0
            self._vz = struct.unpack("<h", data[4:6])[0] / 1000.0
            self._battery_raw = data[6]

        elif opcode == OpCode.REPORT_MPU_RAW:
            gyro_ratio = 1.0 / 3754.9
            accel_ratio = 1.0 / 1671.84
            mag_ratio = 1.0

            self._gx = struct.unpack("<h", data[0:2])[0] * gyro_ratio
            self._gy = struct.unpack("<h", data[2:4])[0] * -gyro_ratio
            self._gz = struct.unpack("<h", data[4:6])[0] * -gyro_ratio

            self._ax = struct.unpack("<h", data[6:8])[0] * accel_ratio
            self._ay = struct.unpack("<h", data[8:10])[0] * accel_ratio
            self._az = struct.unpack("<h", data[10:12])[0] * accel_ratio

            self._mx = struct.unpack("<h", data[12:14])[0] * mag_ratio
            self._my = struct.unpack("<h", data[14:16])[0] * mag_ratio
            self._mz = struct.unpack("<h", data[16:18])[0] * mag_ratio

        elif opcode == OpCode.REPORT_ICM_RAW:
            ratio = 1.0 / 1000.0
            self._gx = struct.unpack("<h", data[0:2])[0] * ratio
            self._gy = struct.unpack("<h", data[2:4])[0] * ratio
            self._gz = struct.unpack("<h", data[4:6])[0] * ratio

            self._ax = struct.unpack("<h", data[6:8])[0] * ratio
            self._ay = struct.unpack("<h", data[8:10])[0] * ratio
            self._az = struct.unpack("<h", data[10:12])[0] * ratio

            self._mx = struct.unpack("<h", data[12:14])[0] * ratio
            self._my = struct.unpack("<h", data[14:16])[0] * ratio
            self._mz = struct.unpack("<h", data[16:18])[0] * ratio

        elif opcode == OpCode.REPORT_IMU_ATT:
            self._roll  = struct.unpack("<h", data[0:2])[0] / 10000.0
            self._pitch = struct.unpack("<h", data[2:4])[0] / 10000.0
            self._yaw   = struct.unpack("<h", data[4:6])[0] / 10000.0

        elif opcode == OpCode.REPORT_ENCODER:
            self._enc_m1 = struct.unpack("<i", data[0:4])[0]
            self._enc_m2 = struct.unpack("<i", data[4:8])[0]
            self._enc_m3 = struct.unpack("<i", data[8:12])[0]
            self._enc_m4 = struct.unpack("<i", data[12:16])[0]

        elif opcode == OpCode.VERSION:
            self._version_major = data[0]
            self._version_minor = data[1]
            self._version = self._version_major + self._version_minor / 10.0
            if self._debug:
                print("FW version:", self._version)

        elif opcode == OpCode.SET_MOTOR_PID:
            self._pid_index = data[0]
            self._kp_raw = struct.unpack("<h", data[1:3])[0]
            self._ki_raw = struct.unpack("<h", data[3:5])[0]
            self._kd_raw = struct.unpack("<h", data[5:7])[0]

        elif opcode == OpCode.AKM_DEF_ANGLE:
            _id = data[0]
            self._akm_default_angle = data[1]
            self._akm_angle_ready = True

        elif opcode == OpCode.SET_CAR_TYPE:
            self._car_type_read = data[0]

        else:
            # 이 예제에선 arm, servo 등은 생략 (필요하면 확장)
            if self._debug:
                print("Unhandled report:", opcode, list(data))

    # ---------------------- Request/Response ----------------------

    def _request_data(self, opcode: OpCode, param: int = 0) -> None:
        """MCU에 특정 데이터를 요청하는 프레임 전송."""
        b = self._build_frame(OpCode.REQUEST_DATA)
        b.add_u8(int(opcode) & 0xFF, param & 0xFF)
        self._send(b.build())

    # ---------------------- 기본 제어 API ----------------------

    def set_auto_report(self, enabled: bool, persist: bool = False) -> None:
        """MCU의 자동 보고 기능 on/off."""
        state = 1 if enabled else 0
        flag = 0x5F if persist else 0
        b = self._build_frame(OpCode.AUTO_REPORT)
        b.add_u8(state, flag)
        self._send(b.build())

    def beep(self, duration_ms: int) -> None:
        """지정한 시간(duration_ms) 동안 부저 ON (0=OFF, 1=계속)."""
        if duration_ms < 0:
            raise ValueError("duration_ms must be >= 0")
        b = self._build_frame(OpCode.BEEP)
        b.add_i16(duration_ms)
        self._send(b.build())

    # ---------------------- 모터 관련 ----------------------

    @staticmethod
    def _limit_motor(value: int) -> int:
        if value == 127:
            return 127
        if value > 100:
            return 100
        if value < -100:
            return -100
        return int(value)

    def set_wheel_pwm(self, m1: int, m2: int, m3: int, m4: int) -> None:
        """각 바퀴 PWM(-100~100) 직접 지정 (양수=전진)."""
        b = self._build_frame(OpCode.MOTOR_PWM)
        b.add_i8(
            self._limit_motor(self.MOTOR_POLARITY * m1),
            self._limit_motor(self.MOTOR_POLARITY * m2),
            self._limit_motor(self.MOTOR_POLARITY * m3),
            self._limit_motor(self.MOTOR_POLARITY * m4),
        )
        self._send(b.build())

    def command_direction(self, state: int, speed: int, use_adjust: bool = False) -> None:
        """
        단순 방향 제어:
        state: 0=STOP, 1=전진, 2=후진, 3=좌, 4=우, 5=좌회전, 6=우회전, 7=주차
        speed: -100~100
        """
        car_type_flag = self._car_type
        if use_adjust:
            car_type_flag |= self._CAR_ADJUST_FLAG

        speed_bytes = struct.pack("<h", int(speed))
        b = self._build_frame(OpCode.CAR_RUN)
        b.add_u8(car_type_flag, state & 0xFF)
        b.payload.extend(speed_bytes)
        self._send(b.build())

    def set_motion(self, vx: float, vy: float, wz: float) -> None:
        """
        기구학 기반 속도 명령:
        vx, vy [m/s], wz [rad/s] → 내부에서 1000배 스케일로 전송.
        """
        vx_i = int(vx * 1000)
        vy_i = int(vy * 1000)
        wz_i = int(wz * 1000)
        b = self._build_frame(OpCode.MOTION)
        b.add_u8(self._car_type)
        b.add_i16(vx_i, vy_i, wz_i)
        self._send(b.build())

    # ---------------------- PID ----------------------

    def set_motion_pid(self, kp: float, ki: float, kd: float, persist: bool = False) -> None:
        """속도 PID 파라미터 설정 (0~10.0 범위 권장)."""
        if not (0 <= kp <= 10 and 0 <= ki <= 10 and 0 <= kd <= 10):
            raise ValueError("kp/ki/kd must be in [0, 10.0]")

        state = 0x5F if persist else 0
        b = self._build_frame(OpCode.SET_MOTOR_PID)
        b.add_i16(int(kp * 1000), int(ki * 1000), int(kd * 1000))
        b.add_u8(state)
        self._send(b.build())
        if persist:
            time.sleep(0.1)

    def get_motion_pid(self) -> Tuple[float, float, float]:
        """MCU에 저장된 속도 PID 값 읽기."""
        self._kp_raw = self._ki_raw = self._kd_raw = 0
        self._pid_index = 0
        self._request_data(OpCode.SET_MOTOR_PID, 1)

        for _ in range(20):
            if self._pid_index > 0:
                return (
                    self._kp_raw / 1000.0,
                    self._ki_raw / 1000.0,
                    self._kd_raw / 1000.0,
                )
            time.sleep(0.001)
        return -1.0, -1.0, -1.0

    # ---------------------- car type / flash / state ----------------------

    def set_car_type(self, car_type: int) -> None:
        """하위 보드에 car type 저장."""
        self._car_type = car_type & 0xFF
        b = self._build_frame(OpCode.SET_CAR_TYPE)
        b.add_u8(self._car_type, 0x5F)  # 0x5F: flash 저장 플래그
        self._send(b.build())
        time.sleep(0.1)

    def get_car_type_from_mcu(self) -> int:
        """MCU에 저장된 car type 읽기."""
        self._car_type_read = 0
        self._request_data(OpCode.SET_CAR_TYPE)
        for _ in range(20):
            if self._car_type_read != 0:
                tmp = self._car_type_read
                self._car_type_read = 0
                return tmp
            time.sleep(0.001)
        return -1

    def reset_flash(self) -> None:
        """MCU flash에 저장된 값 초기화(공장값)."""
        b = self._build_frame(OpCode.RESET_FLASH)
        b.add_u8(0x5F)
        self._send(b.build())
        time.sleep(0.1)

    def reset_runtime_state(self) -> None:
        """차량 상태 초기화: 정지, LED/부저 OFF."""
        b = self._build_frame(OpCode.RESET_STATE)
        b.add_u8(0x5F)
        self._send(b.build())

    # ---------------------- 버전 ----------------------

    def get_firmware_version(self) -> float:
        """MCU 펌웨어 버전(Vx.y) 반환."""
        if self._version_major == 0:
            self._request_data(OpCode.VERSION)
            for _ in range(20):
                if self._version_major != 0:
                    return self._version
                time.sleep(0.001)
        return self._version

    # ---------------------- 센서 읽기 API ----------------------

    def get_accel(self) -> Tuple[float, float, float]:
        """가속도 (ax, ay, az) [m/s^2]."""
        return self._ax, self._ay, self._az

    def get_gyro(self) -> Tuple[float, float, float]:
        """각속도 (gx, gy, gz) [rad/s]."""
        return self._gx, self._gy, self._gz

    def get_mag(self) -> Tuple[float, float, float]:
        """자기장 (mx, my, mz) [임의 단위]."""
        return self._mx, self._my, self._mz

    def get_attitude(self, in_degrees: bool = True) -> Tuple[float, float, float]:
        """roll, pitch, yaw 반환 (기본: 도 단위)."""
        if in_degrees:
            r2d = 57.2957795
            return self._roll * r2d, self._pitch * r2d, self._yaw * r2d
        return self._roll, self._pitch, self._yaw

    def get_motion(self) -> Tuple[float, float, float]:
        """vx, vy, wz [m/s, m/s, rad/s]."""
        return self._vx, self._vy, self._vz

    def get_battery_voltage(self) -> float:
        """배터리 전압 [V]."""
        return self._battery_raw / 10.0

    def get_encoders(self) -> Tuple[int, int, int, int]:
        """4개 바퀴 엔코더 카운트."""
        return self._enc_m1, self._enc_m2, self._enc_m3, self._enc_m4

    # ---------------------- Ackermann 일부 예시 ----------------------

    def set_akm_default_angle(self, angle: int, persist: bool = False) -> None:
        """Ackermann 차량 기본 조향 각도 설정 (예: 60~120)."""
        if not (60 <= angle <= 120):
            return
        state = 0x5F if persist else 0
        b = self._build_frame(OpCode.AKM_DEF_ANGLE)
        b.add_u8(self._akm_servo_id, angle, state)
        self._send(b.build())
        if persist:
            time.sleep(0.1)

    def get_akm_default_angle(self) -> int:
        """MCU에서 Ackermann 기본 각도 읽기."""
        if not self._akm_angle_ready:
            self._request_data(OpCode.AKM_DEF_ANGLE, self._akm_servo_id)
            cnt = 0
            while not self._akm_angle_ready and cnt < 100:
                time.sleep(0.01)
                cnt += 1
            if not self._akm_angle_ready:
                return -1
        return self._akm_default_angle

    def set_akm_steering(self, angle_delta: int, control_car_speed: bool = False) -> None:
        """
        Ackermann 차량 조향 각도 설정 (center 기준 좌/우 ±45).
        control_car_speed=True면 모터 속도도 같이 조정.
        """
        if not (-45 <= angle_delta <= 45):
            return
        sid = self._akm_servo_id | (0x80 if control_car_speed else 0x00)
        b = self._build_frame(OpCode.AKM_STEER_ANGLE)
        b.add_u8(sid, angle_delta & 0xFF)
        self._send(b.build())

    # ---------------------- 종료 처리 ----------------------

    def close(self) -> None:
        """백그라운드 리더를 멈추고 시리얼 포트 닫기."""
        # 먼저 rx_loop 에 정지 신호(state=2) → join 으로 스레드가 read 를 끝내고
        # 빠져나가길 기다린 뒤 시리얼을 닫는다(닫힌 포트 read race 방지).
        self._uart_thread_state = 2
        t = self._rx_thread
        if t is not None and t.is_alive():
            t.join(timeout=1.0)
        if self._ser and self._ser.is_open:
            self._ser.close()
            print("TriboBase: serial closed")

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


# ============================ 예제 사용 ============================

if __name__ == "__main__":
    bot = TriboBase(port="/dev/ttyUSB0", car_type=TriboBase.CARTYPE_X3, debug=True)

    bot.start_background_reader()
    time.sleep(0.1)

    # 자동 리포트 켜기
    bot.set_auto_report(True, persist=False)

    # 비프 테스트
    bot.beep(50)

    # 버전 확인
    print("FW version:", bot.get_firmware_version())

    # 모터 & IMU 테스트 루프
    try:
        bot.set_motion(0.2, 0.0, 0.0)
        for _ in range(50):
            ax, ay, az = bot.get_accel()
            gx, gy, gz = bot.get_gyro()
            mx, my, mz = bot.get_mag()
            print("ACC:", ax, ay, az, "| GYR:", gx, gy, gz, "| MAG:", mx, my, mz)
            time.sleep(0.1)
        bot.set_motion(0.0, 0.0, 0.0)
    except KeyboardInterrupt:
        bot.set_motion(0.0, 0.0, 0.0)
        bot.close()

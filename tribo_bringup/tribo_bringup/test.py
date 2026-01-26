#!/usr/bin/env python3
# coding: utf-8

import struct
import unittest

from tribolib import TriboBase, OpCode


class DummyTribo(TriboBase):
    """
    하드웨어(시리얼 포트) 없이 파싱만 테스트하기 위한 더미 클래스.
    __init__에서 serial.Serial을 열지 않고, 상태 변수만 초기화한다.
    """

    def __init__(self):
        # TreeboBase.__init__를 호출하지 않고 필요한 필드만 직접 세팅
        self._debug = False
        self._delay = 0.0
        self._car_type = TreeboBase.CARTYPE_X3

        # IMU/속도/엔코더/배터리 상태
        self._ax = self._ay = self._az = 0.0
        self._gx = self._gy = self._gz = 0.0
        self._mx = self._my = self._mz = 0.0
        self._vx = self._vy = self._vz = 0.0
        self._roll = self._pitch = self._yaw = 0.0
        self._enc_m1 = self._enc_m2 = self._enc_m3 = self._enc_m4 = 0
        self._battery_raw = 0

        # 버전, PID, Ackermann 상태
        self._version_major = 0
        self._version_minor = 0
        self._version = -1.0

        self._pid_index = 0
        self._kp_raw = self._ki_raw = self._kd_raw = 0

        self._akm_default_angle = 100
        self._akm_angle_ready = False
        self._akm_servo_id = 0x01

        self._car_type_read = 0

        # arm 관련 (여기서는 사용 X)
        self._arm_ctrl_enable = True

        # 시리얼 및 스레드 관련은 테스트에서 사용하지 않으므로 빼도 됨
        self._ser = None
        self._rx_thread = None
        self._uart_thread_state = 0
    def get_motion_pid(self):
        """
        DummyTribo용 PID 조회:
        보드에 요청하지 않고, 이미 파싱된 raw 값을 float로 변환해서 반환.
        """
        if self._pid_index <= 0:
            return -1.0, -1.0, -1.0
        return (
            self._kp_raw / 1000.0,
            self._ki_raw / 1000.0,
            self._kd_raw / 1000.0,
        )

class TestTriboParsing(unittest.TestCase):

    def setUp(self):
        self.bot = DummyTribo()

    # ---------------- REPORT_SPEED ----------------

    def test_report_speed_and_battery(self):
        """
        REPORT_SPEED 패킷 파싱 테스트:
        vx, vy, vz, 배터리 전압이 올바르게 들어가는지 확인.
        """
        # 원하는 값 (단위: m/s, rad/s, V)
        vx = 0.5   # 0.5 m/s
        vy = -0.2  # -0.2 m/s
        vz = 1.0   # 1 rad/s라고 가정 (보드는 단순히 값만 보냄)
        battery_v = 11.9

        # 보드에서 사용하는 raw 값 (mm/s, 0.1V 단위)
        vx_raw = int(vx * 1000)        # 500
        vy_raw = int(vy * 1000)        # -200
        vz_raw = int(vz * 1000)        # 1000
        bat_raw = int(battery_v * 10)  # 119

        data = bytearray()
        data.extend(struct.pack("<h", vx_raw))
        data.extend(struct.pack("<h", vy_raw))
        data.extend(struct.pack("<h", vz_raw))
        data.append(bat_raw)

        self.bot._handle_report(int(OpCode.REPORT_SPEED), data)

        self.assertAlmostEqual(self.bot._vx, vx, places=3)
        self.assertAlmostEqual(self.bot._vy, vy, places=3)
        self.assertAlmostEqual(self.bot._vz, vz, places=3)
        self.assertEqual(self.bot._battery_raw, bat_raw)
        self.assertAlmostEqual(self.bot.get_battery_voltage(), battery_v, places=1)

    # ---------------- REPORT_MPU_RAW ----------------

    def test_report_mpu_raw(self):
        """
        REPORT_MPU_RAW 패킷 파싱 테스트:
        가속도/자이로/자기장 값이 올바르게 변환되는지 확인.
        """
        # 보드 코드 기준
        gyro_ratio = 1.0 / 3754.9   # gx = raw * gyro_ratio
        accel_ratio = 1.0 / 1671.84 # ax = raw * accel_ratio
        mag_ratio = 1.0             # mx = raw * mag_ratio

        # 목표 물리 값 (임의)
        gx_target = 0.1  # rad/s
        gy_target = -0.2
        gz_target = 0.3

        ax_target = 1.0  # m/s^2
        ay_target = -0.5
        az_target = 0.0

        mx_target = 100.0
        my_target = -50.0
        mz_target = 10.0

        # 역으로 raw 값 계산
        gx_raw = int(gx_target / gyro_ratio)
        gy_raw = int(-gy_target / gyro_ratio)  # 코드에서 gy는 부호 반전
        gz_raw = int(-gz_target / gyro_ratio)

        ax_raw = int(ax_target / accel_ratio)
        ay_raw = int(ay_target / accel_ratio)
        az_raw = int(az_target / accel_ratio)

        mx_raw = int(mx_target / mag_ratio)
        my_raw = int(my_target / mag_ratio)
        mz_raw = int(mz_target / mag_ratio)

        data = bytearray()
        data.extend(struct.pack("<h", gx_raw))
        data.extend(struct.pack("<h", gy_raw))
        data.extend(struct.pack("<h", gz_raw))
        data.extend(struct.pack("<h", ax_raw))
        data.extend(struct.pack("<h", ay_raw))
        data.extend(struct.pack("<h", az_raw))
        data.extend(struct.pack("<h", mx_raw))
        data.extend(struct.pack("<h", my_raw))
        data.extend(struct.pack("<h", mz_raw))

        self.bot._handle_report(int(OpCode.REPORT_MPU_RAW), data)

        self.assertAlmostEqual(self.bot._gx, gx_target, places=2)
        self.assertAlmostEqual(self.bot._gy, gy_target, places=2)
        self.assertAlmostEqual(self.bot._gz, gz_target, places=2)

        self.assertAlmostEqual(self.bot._ax, ax_target, places=2)
        self.assertAlmostEqual(self.bot._ay, ay_target, places=2)
        self.assertAlmostEqual(self.bot._az, az_target, places=2)

        self.assertAlmostEqual(self.bot._mx, mx_target, places=1)
        self.assertAlmostEqual(self.bot._my, my_target, places=1)
        self.assertAlmostEqual(self.bot._mz, mz_target, places=1)

    # ---------------- REPORT_IMU_ATT ----------------

    def test_report_imu_att(self):
        """
        REPORT_IMU_ATT 패킷 파싱 테스트:
        roll/pitch/yaw가 rad로 저장되고, get_attitude()에서 deg로 바뀌는지 확인.
        """
        # rad 단위 목표
        roll_rad = 0.1
        pitch_rad = -0.2
        yaw_rad = 0.3

        roll_raw = int(roll_rad * 10000)
        pitch_raw = int(pitch_rad * 10000)
        yaw_raw = int(yaw_rad * 10000)

        data = bytearray()
        data.extend(struct.pack("<h", roll_raw))
        data.extend(struct.pack("<h", pitch_raw))
        data.extend(struct.pack("<h", yaw_raw))

        self.bot._handle_report(int(OpCode.REPORT_IMU_ATT), data)

        self.assertAlmostEqual(self.bot._roll, roll_rad, places=4)
        self.assertAlmostEqual(self.bot._pitch, pitch_rad, places=4)
        self.assertAlmostEqual(self.bot._yaw, yaw_rad, places=4)

        # get_attitude(deg) 확인
        roll_deg, pitch_deg, yaw_deg = self.bot.get_attitude(in_degrees=True)
        self.assertAlmostEqual(roll_deg, roll_rad * 57.2958, places=2)
        self.assertAlmostEqual(pitch_deg, pitch_rad * 57.2958, places=2)
        self.assertAlmostEqual(yaw_deg, yaw_rad * 57.2958, places=2)

    # ---------------- REPORT_ENCODER ----------------

    def test_report_encoder(self):
        """
        REPORT_ENCODER 패킷 파싱 테스트:
        네 개 엔코더 카운트가 올바르게 들어가는지 확인.
        """
        m1 = 123456
        m2 = -234567
        m3 = 345678
        m4 = -456789

        data = bytearray()
        data.extend(struct.pack("<i", m1))
        data.extend(struct.pack("<i", m2))
        data.extend(struct.pack("<i", m3))
        data.extend(struct.pack("<i", m4))

        self.bot._handle_report(int(OpCode.REPORT_ENCODER), data)

        self.assertEqual(self.bot._enc_m1, m1)
        self.assertEqual(self.bot._enc_m2, m2)
        self.assertEqual(self.bot._enc_m3, m3)
        self.assertEqual(self.bot._enc_m4, m4)

        encoders = self.bot.get_encoders()
        self.assertEqual(encoders, (m1, m2, m3, m4))

    # ---------------- VERSION ----------------

    def test_version_parsing(self):
        """
        VERSION 패킷 파싱 테스트:
        major/minor에서 float 버전이 제대로 계산되는지 확인.
        """
        major = 1
        minor = 5  # → 1.5 로 해석

        data = bytearray([major, minor])

        self.bot._handle_report(int(OpCode.VERSION), data)

        self.assertEqual(self.bot._version_major, major)
        self.assertEqual(self.bot._version_minor, minor)
        self.assertAlmostEqual(self.bot._version, 1.5, places=2)

    # ---------------- SET_MOTOR_PID (readback) ----------------

    def test_motor_pid_parsing(self):
        """
        SET_MOTOR_PID 응답 패킷 파싱 테스트:
        내부 raw 값이 들어갔을 때 float로 다시 맞게 변환되는지 확인.
        """
        kp = 0.5
        ki = 0.1
        kd = 0.3

        kp_raw = int(kp * 1000)
        ki_raw = int(ki * 1000)
        kd_raw = int(kd * 1000)

        data = bytearray()
        data.append(1)  # pid_index
        data.extend(struct.pack("<h", kp_raw))
        data.extend(struct.pack("<h", ki_raw))
        data.extend(struct.pack("<h", kd_raw))

        self.bot._handle_report(int(OpCode.SET_MOTOR_PID), data)

        self.assertEqual(self.bot._pid_index, 1)
        self.assertEqual(self.bot._kp_raw, kp_raw)
        self.assertEqual(self.bot._ki_raw, ki_raw)
        self.assertEqual(self.bot._kd_raw, kd_raw)

        # get_motion_pid에서 다시 변환되는지 확인
        kps, kis, kds = self.bot.get_motion_pid()
        self.assertAlmostEqual(kps, kp, places=3)
        self.assertAlmostEqual(kis, ki, places=3)
        self.assertAlmostEqual(kds, kd, places=3)


if __name__ == "__main__":
    unittest.main()

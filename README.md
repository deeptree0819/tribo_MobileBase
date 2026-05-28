# Tribo MobileBase

Yahboom **Rosmaster STM32 ROS 확장보드** 기반 차동구동(differential drive) 모바일 로봇용 ROS 2 패키지.
새 로봇(Ubuntu 24.04 / ROS 2 Jazzy)에서 처음부터 세팅하는 절차를 정리한 문서입니다.

> 기존 환경은 **ROS 2 Jazzy + Ubuntu 24.04** 기준입니다. 다른 배포판을 쓴다면 아래 `jazzy`를 해당 배포판 이름으로 바꾸세요.

---

## 1. 패키지 구성

| 패키지 | 역할 |
|--------|------|
| `tribo_bringup`   | HW 인터페이스: `/cmd_vel` → 모터 PWM, 엔코더 퍼블리시 (`tribolib.py` 사용) |
| `tribo_odom`      | 엔코더 → `/odom` + TF (`odom`→`base_link`) |
| `tribo_description` | URDF(xacro) 로봇 모델 |
| `tribo_navigation`  | Nav2 / SLAM Toolbox 설정·런치 |
| `tribo_gazebo`    | Gazebo 시뮬레이션 |
| `sllidar_ros2`    | RPLIDAR 드라이버 (**git 서브모듈**) |

데이터 흐름: `bringup`(보드 시리얼) → `encoder_raw` → `odom_publisher`(`/odom`+TF) → `nav2`

---

## 2. 사전 요구사항

- Ubuntu 24.04
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) 설치 완료
- Yahboom Rosmaster STM32 확장보드 (USB-시리얼 CH340) 연결
- RPLIDAR (C1 등, USB-시리얼 CP2102N) 연결

---

## 3. 설치

### 3-1. 시스템 의존성

```bash
sudo apt update
sudo apt install -y \
  python3-serial \
  python3-colcon-common-extensions \
  git
```

> `python3-serial`(pyserial)은 `tribolib.py`가 보드와 통신하는 데 **유일하게 필요한 외부 파이썬 의존성**입니다. (Yahboom의 `Rosmaster_Lib`는 따로 설치할 필요 없음 — `tribolib.py`가 프로토콜을 자체 구현)

### 3-2. ROS 2 패키지 의존성

```bash
sudo apt install -y \
  ros-jazzy-geometry-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-tf2-ros \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox
```

시뮬레이션(`tribo_gazebo`)도 쓸 경우:

```bash
sudo apt install -y \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image
```

### 3-3. 워크스페이스 클론 (서브모듈 포함)

`sllidar_ros2`는 서브모듈이므로 `--recurse-submodules`가 필수입니다.

```bash
mkdir -p ~/tribo_ws/src
cd ~/tribo_ws/src
git clone --recurse-submodules https://github.com/deeptree0819/tribo_MobileBase.git tribo

# 이미 서브모듈 없이 클론했다면:
cd ~/tribo_ws/src/tribo
git submodule update --init --recursive
```

### 3-4. 빌드

```bash
cd ~/tribo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 4. 하드웨어 설정

### 4-1. 시리얼 포트 접근 권한

`dialout` 그룹에 사용자를 추가해야 보드/라이다 시리얼 포트(`/dev/ttyUSB*`)에 접근할 수 있습니다.

```bash
sudo usermod -aG dialout $USER
# 로그아웃 후 다시 로그인(또는 재부팅)해야 적용됨
```

CH340 / CP2102 드라이버는 최신 커널에 기본 내장돼 있어 보통 별도 설치가 필요 없습니다.
연결 확인:

```bash
ls -l /dev/serial/by-id/
# 예시:
#   usb-1a86_USB_Serial-if00-port0                → 보드(ttyUSB0)
#   usb-Silicon_Labs_CP2102N_..._if00-port0       → 라이다(ttyUSB1)
```

### 4-2. 보드 포트

`tribo_bringup/config/bringup.yaml`의 `port`는 번호 변동이 없는 **by-id 고정 경로**를 사용합니다:

```yaml
port: "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
```

보드 모델/USB-시리얼 칩이 다르면 위 `by-id` 경로를 실제 값으로 수정하세요.

### 4-3. 라이다 포트 ⚠️ (새 로봇에서 주의)

`sllidar_ros2/launch/sllidar_c1_launch.py`의 `serial_port` 기본값은 **특정 라이다 1대의 고유 시리얼 번호**가 박힌 by-id 경로입니다(CP2102N 칩의 일련번호 포함). **새 로봇의 라이다는 번호가 다르므로** 다음 중 하나로 맞춰야 합니다:

```bash
# (A) 런치 시 인자로 직접 지정
ros2 launch sllidar_ros2 sllidar_c1_launch.py \
  serial_port:=/dev/serial/by-id/<새-라이다-by-id>

# (B) 또는 launch 파일의 serial_port 기본값을 자기 로봇 값으로 수정
```

> 이 서브모듈 변경은 로봇마다 다르므로 보통 커밋하지 않고 로컬에만 둡니다.

---

## 5. ROS_DOMAIN_ID 설정 (PC ↔ 로봇 통신)

PC와 로봇이 서로의 토픽을 보려면 **같은 `ROS_DOMAIN_ID`** 를 써야 합니다. `~/.bashrc` 끝에 추가:

```bash
export ROS_DOMAIN_ID=20          # PC·로봇 동일 값으로
source /opt/ros/jazzy/setup.bash
source ~/tribo_ws/install/setup.bash
```

> alias만 정의해 두고 매번 수동 실행하면 노드별로 도메인이 어긋날 수 있으니, 위처럼 **export로 고정**하는 것을 권장합니다.

---

## 6. 실행

### 6-1. 실제 로봇 bringup (보드 + 오도메트리 + 라이다)

```bash
ros2 launch tribo_bringup bringup.launch.py
```

주요 런치 인자(기본값): `use_odom:=true`, `use_lidar:=true`, `use_description:=true`

모터 구동만 (라이다·odom 제외):

```bash
ros2 launch tribo_bringup bringup.launch.py use_lidar:=false use_odom:=false
```

bringup + odom만 묶은 코어 런치:

```bash
ros2 launch tribo_bringup core.launch.py
```

### 6-2. 동작 테스트

```bash
# 전진 명령 (안전을 위해 바퀴를 들고 테스트 권장)
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"
```

> bringup에는 **watchdog**이 있어 `cmd_vel`이 `cmd_timeout`(기본 0.5초)초 동안 끊기면 모터를 자동 정지합니다.

### 6-3. SLAM 매핑 / 내비게이션

```bash
# 지도 작성 (SLAM Toolbox)
ros2 launch tribo_navigation map_building.launch.py

# 내비게이션 (Nav2)
ros2 launch tribo_navigation navigation_launch.xml
```

### 6-4. 시뮬레이션 (Gazebo)

```bash
ros2 launch tribo_gazebo launch_sim.launch.xml
```

---

## 7. 주요 파라미터 파일

| 파일 | 설명 |
|------|------|
| `tribo_bringup/config/bringup.yaml`    | 시리얼 포트, 모터별 gain, PWM 최소듀티, cmd 안전(deadzone/timeout) |
| `tribo_bringup/config/robot_geom.yaml` | 공유 기구 파라미터 (track_width, wheel_radius, ticks_per_rev) |
| `tribo_odom/config/odom.yaml`          | 오도메트리 파라미터 |
| `tribo_navigation/config/nav2_params.yaml` | Nav2 설정 |

---

## 8. 문제 해결

| 증상 | 확인 |
|------|------|
| `could not open port /dev/ttyUSB0` | `dialout` 그룹 추가 후 재로그인 했는지, 보드가 연결됐는지 |
| `ModuleNotFoundError: serial` | `sudo apt install python3-serial` |
| PC에서 publish해도 로봇이 안 움직임 | PC·로봇 `ROS_DOMAIN_ID` 일치 여부 |
| 라이다 `/scan` 안 나옴 | 4-3의 라이다 `serial_port` 경로가 실제 장치와 맞는지 |
| `sllidar_ros2` 빌드 누락 | `git submodule update --init --recursive` 후 재빌드 |

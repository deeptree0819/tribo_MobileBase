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

### 6-3. 맵 만들기 (SLAM)

SLAM Toolbox로 지도를 작성합니다. (`ros-jazzy-slam-toolbox` 패키지 필요 — 3-2의 의존성 목록에 포함)

**① 로봇에서: bringup 실행** (모터·오도메트리·라이다·URDF 전체)

```bash
# 로봇 (dtrp@192.168.210.14)
ros2 launch tribo_bringup bringup.launch.py
```

**② SLAM 실행** (로봇 또는 PC, 어느 쪽이든 OK — 같은 `ROS_DOMAIN_ID`만 맞으면 됨)

```bash
ros2 launch tribo_navigation map_building.launch.py
```

이 런치는 내부적으로 다음을 띄웁니다:
- `laser_filters/scan_to_scan_filter_chain` (`/scan` → `/scan_filtered`)
- `slam_toolbox` online sync (`config/slam_toolbox_mapping.yaml`)

**③ 텔레옵으로 주행하며 매핑**

```bash
# 별도 터미널에서 (PC 권장)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> 너무 빠르거나 급회전하면 스캔 매칭이 깨집니다. 천천히, 겹치는 구간을 만들면서 한 바퀴 돌리세요.

**④ 맵 저장**

`map_saver_cli`는 **실행한 머신의 현재 경로**에 `<이름>.yaml` + `<이름>.pgm` 두 파일을 저장합니다. Nav2가 결국 **로봇에서** map을 로드하므로, 가능하면 **로봇에서 저장**하는 게 가장 간단합니다.

```bash
# 로봇에서 저장 → /home/dtrp/my_map.{yaml,pgm} 생성됨
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

PC에서 저장한 경우, 로봇으로 복사해야 합니다:

```bash
# PC에서 저장했다면
ros2 run nav2_map_server map_saver_cli -f ~/my_map
scp ~/my_map.yaml ~/my_map.pgm dtrp@192.168.210.14:/home/dtrp/
```

> **최종 상태**: 로봇 측에 `/home/dtrp/my_map.yaml` + `/home/dtrp/my_map.pgm` 두 파일이 있어야 다음 단계(Navigation)가 동작합니다.

### 6-4. Navigation 실행 (Nav2)

저장한 맵 위에서 AMCL 자기위치추정 + Nav2 플래너/컨트롤러로 자율 주행합니다.

**① 로봇에서: bringup 실행**

```bash
# 로봇
ros2 launch tribo_bringup bringup.launch.py
```

**② 로봇에서: Nav2 전체 스택 실행** (localization + navigation 합본)

```bash
# 로봇 — map 경로는 "로봇 입장의 절대경로"여야 함
ros2 launch tribo_navigation bringup_launch.xml \
  map:=/home/dtrp/my_map.yaml \
  set_initial_pose:=false
```

- `set_initial_pose:=false`로 띄우면 초기 위치를 자동으로 발행하지 않으므로, 다음 단계에서 RViz "2D Pose Estimate"로 직접 찍어줘야 `map`→`odom` TF가 살아납니다.
- 처음부터 원점에서 시작한다는 게 확실하면 `set_initial_pose:=true`(기본값) + `initial_pose_x/y/yaw`로 자동 발행도 가능합니다.
- 위 launch는 **로봇에 연결된 LCD에 RViz2를 전체화면으로 자동 표시**합니다(`use_rviz:=true` 기본). RViz가 필요 없으면 `use_rviz:=false`로 끄세요. LCD 표시는 GNOME 데스크톱 세션의 Xwayland(`:0`)에 X 앱을 띄우는 방식이라, **로봇 LCD에 사용자가 한 번 로그인되어 GNOME 세션이 살아있어야** RViz 창이 화면에 뜹니다(자동 로그인이 꺼져 있다면 부팅 후 LCD에서 한 번 로그인 필요).

**③ (선택) PC에서도 RViz 따로 보기**

LCD 화면이 작거나 별도 모니터로 보고 싶을 때만. 같은 토픽을 PC의 큰 화면에서 동시에 볼 수 있습니다(로봇 LCD의 RViz와 무관).

```bash
# PC
ros2 launch tribo_navigation nav2_view.launch.xml
```

**④ RViz 조작**
1. 상단 툴바 **"2D Pose Estimate"** 클릭 → 맵 상의 실제 로봇 위치/방향을 클릭+드래그로 지정 (한 번만 하면 됨)
2. 상단 툴바 **"Nav2 Goal"** 클릭 → 목표 위치 클릭+드래그 → 자동 주행 시작

**⑤ 동작 검증**

```bash
# 로봇에서 (또는 PC에서, 도메인만 같으면 됨)
ros2 lifecycle get /map_server     # → active [3]
ros2 lifecycle get /amcl           # → active [3]
ros2 run tf2_ros tf2_echo map odom # → 주기적으로 transform 출력되면 OK
ros2 topic echo /amcl_pose --once  # 현재 추정 위치
```

> **⚠️ 함정 — Nav2 단독 런치만 띄우면 안 됨**
>
> `tribo_navigation/launch/navigation_launch.xml`은 컨트롤러/플래너/BT/스무더 등 **주행 관련 노드만** 띄우는 부분 런치입니다. 단독 실행하면 `map_server`·`amcl`이 안 떠서 `map` 프레임이 생기지 않고, `tf2_echo map odom`이 영원히 대기합니다. **반드시 `bringup_launch.xml`(= localization + navigation 합본)을 사용하세요.**
>
> **⚠️ 함정 — `map:=` 경로는 launch가 실행되는 머신 기준**
>
> 위 예시는 로봇에서 `ros2 launch`를 실행하므로 `map:=/home/dtrp/my_map.yaml`(로봇 홈) 입니다. 만약 PC에서 launch를 실행한다면 PC 입장 경로(`/home/deeptree/...`)로 줘야 합니다. PC의 `/home/deeptree/...`를 로봇 launch에 넘기면 `map_server` configure가 "파일을 못 찾는다"며 실패합니다.

### 6-5. 시뮬레이션 (Gazebo)

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

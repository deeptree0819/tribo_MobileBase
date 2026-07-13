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

## 3. 원격 접속 (SSH) 설정 — 헤드리스 라즈베리파이

라즈베리파이 기반 로봇은 보통 모니터 없이 PC에서 SSH로 접속해 작업합니다. **새로 설치한 OS는 SSH 서버(sshd)가 없거나 비활성 상태**일 수 있어, 처음 한 번은 직접 켜줘야 합니다.

### 3-1. 증상 진단 (PC에서)

PC에서 접속이 안 될 때, 먼저 원인을 구분합니다.

```bash
ping -c 3 <robot-ip>           # 호스트가 살아있는지
nc -vz <robot-ip> 22           # 22번 포트(sshd) 상태
```

| `nc` 결과 | 의미 | 조치 |
|-----------|------|------|
| `succeeded` | sshd 정상 | 바로 `ssh` 접속 (3-3) |
| `Connection refused` | 호스트는 살아있으나 **sshd 미기동/미설치** | 3-2로 |
| `timed out` | 방화벽 차단 또는 IP/네트워크 문제 | IP 재확인 / `sudo ufw allow ssh` |

> `ping`은 되는데 `nc ... 22`가 **Connection refused**면 거의 항상 라즈베리파이의 SSH가 꺼져 있는 것입니다(네트워크·IP는 정상).

### 3-2. SSH 서버 설치·활성화 (라즈베리파이 본체에서)

모니터+키보드를 연결하거나 SD카드 콘솔에서 실행합니다.

```bash
# 이미 설치돼 있고 꺼져만 있다면 이 줄만으로 충분
sudo systemctl enable --now ssh

# "Unit ssh.service not found"처럼 sshd 자체가 없다면 설치부터
sudo apt update
sudo apt install -y openssh-server
sudo systemctl enable --now ssh

sudo systemctl status ssh      # active (running) 확인
```

> 헤드리스(모니터 없음)로 처음부터 켜려면: SD카드를 PC에 꽂아 boot 파티션에 **빈 `ssh` 파일**(확장자 없음)을 만들면 부팅 시 자동 활성화됩니다 — `touch /media/$USER/bootfs/ssh` (파티션명은 `ls /media/$USER/`로 확인).

### 3-3. 접속 확인 (PC에서)

```bash
nc -vz <robot-ip> 22           # 이제 succeeded!
ssh <robot-user>@<robot-ip>    # 예: ssh tribo@192.168.210.20
```

> 로봇마다 **계정명이 다를 수 있습니다.** 라즈베리파이에서 `whoami`로 확인하세요. `tribossh` 같은 alias를 쓴다면 `type tribossh`로 가리키는 `user@host`가 해당 로봇과 맞는지 점검하세요(IP만 바꾸고 옛 계정을 가리키면 인증 단계에서 실패).
>
> 여기까지는 IP로 접속하지만, 키 등록(3-4) 후에는 **3-4-1처럼 호스트명 별칭(`tribo-robot`)으로 고정**하는 것을 권장합니다. 이 문서의 이후 예시는 모두 그 별칭을 씁니다.

### 3-4. SSH 키 등록 (비밀번호 없이 접속)

매번 비밀번호를 입력하지 않도록 PC의 공개키를 로봇에 등록합니다. 자동화 스크립트나 `scp`, 원격 launch에도 필수입니다.

```bash
# PC에 키가 없다면 먼저 생성 (이미 있으면 생략)
ls ~/.ssh/id_ed25519.pub || ssh-keygen -t ed25519

# 공개키를 로봇에 등록 — 이때 한 번만 비밀번호 입력
ssh-copy-id <robot-user>@<robot-ip>      # 예: ssh-copy-id tribo@192.168.210.20

# 확인 — 이제 비밀번호 없이 접속돼야 함
ssh <robot-user>@<robot-ip> 'echo OK'
```

### 3-4-1. IP 대신 호스트명으로 고정하기 (권장)

로봇 IP는 **DHCP로 재부팅·재접속 때마다 바뀔 수 있습니다.** 실제로 이 프로젝트에서도 `.14` → `.16` → `.20` 으로 두 번 바뀌었고, 그때마다 IP를 하드코딩해 둔 스크립트·문서·alias가 전부 깨졌습니다. 더 나쁜 건, 비워진 옛 IP를 **다른 기기가 가져가면** 접속 시 `timed out`이 아니라 `Connection refused`가 떠서 "로봇이 꺼졌다"고 오진하기 쉽다는 점입니다.

그래서 IP는 어디에도 적지 말고, **`~/.ssh/config` 한 곳에서 호스트명으로** 관리합니다. 라즈베리파이/우분투는 mDNS(avahi)가 기본 동작하므로 `<hostname>.local` 이 같은 네트워크에서 자동으로 해석됩니다.

```bash
# PC ~/.ssh/config
Host tribo-robot
  HostName tribo-robot.local     # 로봇의 hostname + .local (IP를 적지 않는다)
  User tribo
  IdentityFile ~/.ssh/id_ed25519
  StrictHostKeyChecking accept-new
```

```bash
# 확인 — 현재 IP가 무엇이든 알아서 찾아감
getent hosts tribo-robot.local   # 현재 IP 확인용
ssh tribo-robot 'hostname'       # → tribo-robot
```

이제 IP가 바뀌어도 **고칠 곳이 없습니다.** 이후 모든 명령·스크립트는 `user@ip` 대신 `tribo-robot` 별칭만 씁니다. PC `~/.bashrc`에 alias를 두면 더 짧아집니다:

```bash
alias tribo="ssh tribo-robot"    # IP는 ~/.ssh/config 의 tribo-robot 한 곳에서만 관리
```

> 접속이 안 되면 `getent hosts tribo-robot.local`로 현재 IP부터 확인하세요. 아무것도 안 나오면 로봇이 꺼져 있거나 다른 공유기(AP)에 붙은 것입니다. mDNS가 막힌 네트워크라면 `HostName`에 IP를 직접 적되, **그 한 줄만** 갱신하면 되도록 나머지는 별칭을 유지하세요.

### 3-5. 호스트키 변경 경고 — 같은 IP에 새 로봇을 올렸을 때 (PC에서)

기존 로봇과 **같은 IP를 재사용하는 새 로봇**(재설치/보드 교체 포함)에 접속하면, 새 OS가 새 호스트키를 갖고 있어 PC가 이렇게 막습니다.

```
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@    WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!     @
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
...
Offending ED25519 key in /home/<user>/.ssh/known_hosts:41
Host key verification failed.
```

이건 MITM 공격이 아니라 **로봇이 바뀌었으니 당연한 경고**입니다(PC의 `known_hosts`에 옛 로봇의 키가 남아 있는 것). 해당 IP의 옛 키만 지우면 됩니다.

```bash
# PC에서 — 옛 호스트키 제거 (백업은 known_hosts.old 로 자동 보관됨)
ssh-keygen -f ~/.ssh/known_hosts -R <robot-ip>      # 예: -R 192.168.210.20

# 3-4-1의 호스트명 별칭을 쓴다면 known_hosts에도 호스트명으로 기록되므로 그쪽도 제거
ssh-keygen -f ~/.ssh/known_hosts -R tribo-robot.local

# 다시 접속하면 새 키 등록 여부를 물어봄 → yes
ssh <robot-user>@<robot-ip>
# The authenticity of host ... can't be established.
# Are you sure you want to continue connecting (yes/no/[fingerprint])?  → yes
```

> 접속이 뜨면 로봇에서 `whoami`/`hostname`으로 **계정·호스트명이 맞는 로봇인지** 한 번 확인하세요(옛 로봇과 IP만 같고 실제로 다른 장비일 수 있음). 이후 `ssh-copy-id`(3-4)로 공개키를 다시 등록하면 비밀번호 없이 접속됩니다.

---

## 4. 설치

> **📍 대상: PC와 로봇 양쪽에서 각각 수행합니다.** 두 머신 모두 자기 `~/tribo_ws`를 갖고 빌드합니다 — 로봇은 실제 bringup·주행용, PC는 시뮬레이션·RViz·개발용. (하드웨어 설정인 5장만 **로봇 전용**입니다.)
>
> | 소절 | PC | 로봇 | 비고 |
> |------|:--:|:----:|------|
> | 4-1 시스템 의존성 | ✅ | ✅ | `python3-serial`은 실제로는 로봇에서만 쓰이나, 설치해 둬도 무해 |
> | 4-2 ROS 2 패키지 의존성 | ✅ | ✅ | 시뮬(gazebo) 블록은 **PC만** |
> | 4-3 워크스페이스 clone | ✅ | ✅ | 각 머신에 독립적으로 clone |
> | 4-4 빌드 | ✅ | ✅ | 각 머신에서 `colcon build` |

### 4-1. 시스템 의존성 &nbsp;·&nbsp; 🖥️ PC + 🤖 로봇

```bash
sudo apt update
sudo apt install -y \
  python3-serial \
  python3-colcon-common-extensions \
  git
```

> `python3-serial`(pyserial)은 `tribolib.py`가 보드와 통신하는 데 **유일하게 필요한 외부 파이썬 의존성**입니다. (Yahboom의 `Rosmaster_Lib`는 따로 설치할 필요 없음 — `tribolib.py`가 프로토콜을 자체 구현)
>
> `python3-colcon-common-extensions`가 `Unable to locate package`로 안 잡히면 **ROS 2 apt 저장소가 아직 추가되지 않은 것**입니다(이 패키지는 packages.ros.org에 있음). 2장의 ROS 2 Jazzy 설치를 먼저 끝내면 잡힙니다.

### 4-2. ROS 2 패키지 의존성 &nbsp;·&nbsp; 🖥️ PC + 🤖 로봇

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

시뮬레이션(`tribo_gazebo`)도 쓸 경우 &nbsp;·&nbsp; **🖥️ PC 전용** (로봇에선 불필요):

```bash
sudo apt install -y \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image
```

### 4-3. 워크스페이스 클론 (서브모듈 포함) &nbsp;·&nbsp; 🖥️ PC + 🤖 로봇 (각각)

`sllidar_ros2`는 서브모듈이므로 `--recurse-submodules`가 필수입니다.

```bash
mkdir -p ~/tribo_ws/src
cd ~/tribo_ws/src
git clone --recurse-submodules https://github.com/deeptree0819/tribo_MobileBase.git tribo

# 이미 서브모듈 없이 클론했다면:
cd ~/tribo_ws/src/tribo
git submodule update --init --recursive
```

### 4-4. 빌드 &nbsp;·&nbsp; 🖥️ PC + 🤖 로봇 (각각)

```bash
cd ~/tribo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 5. 하드웨어 설정 &nbsp;·&nbsp; 🤖 로봇 전용

### 5-1. 시리얼 포트 접근 권한

`dialout` 그룹에 사용자를 추가해야 보드/라이다 시리얼 포트(`/dev/ttyUSB*`)에 접근할 수 있습니다. 추가하지 않으면 bringup이 `could not open port` 로 실패합니다.

```bash
# 로봇에서 (SSH로 접속한 상태). sudo 비밀번호를 한 번 입력
sudo usermod -aG dialout $USER

# 그룹 변경은 새 로그인 세션부터 적용 → 재로그인(또는 재부팅) 필수
exit
ssh <robot-user>@<robot-ip>     # 다시 접속
# 또는: sudo reboot

# 확인 — 목록에 dialout 이 보이면 성공
groups
# tribo adm dialout cdrom sudo ...
```

> `/dev/ttyUSB0` 권한은 `crw-rw---- root dialout`이라, dialout 그룹에 들기 전에는 같은 사용자라도 포트를 열 수 없습니다.

CH340 / CP2102 드라이버는 최신 커널에 기본 내장돼 있어 보통 별도 설치가 필요 없습니다.
연결 확인:

```bash
ls -l /dev/serial/by-id/
# 예시:
#   usb-1a86_USB_Serial-if00-port0                → 보드(ttyUSB0)
#   usb-Silicon_Labs_CP2102N_..._if00-port0       → 라이다(ttyUSB1)
```

### 5-2. USB 장치 고정 이름 (udev) — 권장

USB 포트를 다른 소켓에 꽂거나 부팅 순서가 바뀌면 `/dev/ttyUSB0`↔`ttyUSB1`이 뒤바뀌어 **보드와 라이다가 서로 포트를 뺏는** 문제(보드 `multiple access on port`, 라이다 `OPERATION_TIMEOUT`)가 생길 수 있습니다. udev 규칙으로 칩 종류(VID:PID) 기반 고정 심볼릭을 만들어 원천 차단합니다.

repo에 규칙 파일이 포함돼 있습니다: `tribo_bringup/udev/99-tribo-serial.rules`

```bash
# 로봇에서 (sudo 비밀번호 1회)
sudo cp ~/tribo_ws/src/tribo/tribo_bringup/udev/99-tribo-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# 확인 — 두 심볼릭이 보이면 성공
ls -l /dev/tribo_base /dev/tribo_lidar
#  /dev/tribo_base  -> ../../ttyUSB0
#  /dev/tribo_lidar -> ../../ttyUSB1
```

| 장치 | 매칭 기준 | 고정 이름 |
|------|-----------|-----------|
| 보드 (CH340)     | VID:PID `1a86:7523` | `/dev/tribo_base`  |
| 라이다 (CP2102N) | VID:PID `10c4:ea60` | `/dev/tribo_lidar` |

> - 보드/라이다 칩이 위와 다르면 규칙 파일의 VID:PID를 자기 장치 값(`udevadm info -q property -n /dev/ttyUSB0`의 `ID_VENDOR_ID`/`ID_MODEL_ID`)으로 바꾸세요.
> - 같은 칩(예: CP2102N 2개)을 동시에 꽂는 경우에만 VID:PID로 구분이 안 됩니다. 그땐 규칙 파일 주석대로 라이다 줄에 `ATTRS{serial}=="<고유시리얼>"`을 추가하세요.

### 5-3. 보드·라이다 포트 해석 (자동)

`bringup.launch.py`가 시작할 때 포트를 **자동 해석**합니다 — udev 심볼릭(`/dev/tribo_base`, `/dev/tribo_lidar`)이 있으면 그것을, 없으면 `/dev/serial/by-id/` 경로로 폴백합니다. 따라서 **5-2의 udev 설치를 건너뛰어도** by-id로 동작하고, ttyUSB 번호 변동에는 항상 안전합니다.

- **보드**: `bringup.yaml`에는 `port`를 두지 않고 launch가 해석한 값을 사용합니다.
- **라이다**: `bringup.launch.py`가 해석한 `serial_port`를 sllidar 런치에 넘깁니다 — `sllidar_c1_launch.py`를 직접 고칠 필요가 없습니다.

> 칩이 다른 보드/라이다를 쓰면 `bringup.launch.py` 상단의 `_resolve_port(...)` 매칭 문자열과 `udev/99-tribo-serial.rules`의 VID:PID를 함께 바꾸세요.

### 5-4. CPU 거버너 = performance ⚠️ (Pi5 + Nav2 필수)

라즈베리파이 기본 CPU 거버너는 `ondemand`라 부하에 따라 클럭을 **천천히** 올립니다. Nav2 12개 노드가 한꺼번에 뜨는 **버스트 시작** 때 클럭(2.4GHz)이 못 따라와서:

- `controller_server`가 costmap 로딩에 너무 오래 걸려 → `lifecycle_manager: get_state service is not available! Aborting bringup` 으로 **nav 전체가 abort**
- `ekf_node: Failed to meet update rate!` 스톨 다발

거버너를 `performance`(전 코어 최대 클럭 고정)로 바꾸면 해결됩니다.

```bash
# 즉시 적용 (재부팅 전까지)
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# 확인 — performance / 2400000(2.4GHz) 나와야 함
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq
```

재부팅해도 유지하려면 repo의 systemd 서비스를 설치:

```bash
sudo cp ~/tribo_ws/src/tribo/tribo_bringup/system/cpu-performance.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now cpu-performance.service
systemctl is-enabled cpu-performance.service                 # enabled 면 OK
systemctl is-active  cpu-performance.service                 # active 면 OK

# 재부팅 후 검증 — 4코어 전부 performance 나와야 함
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

> performance 거버너는 발열·소비전력이 늘어납니다. Pi5는 액티브 쿨러가 있으면 문제없습니다. Nav2를 안 쓰고 bringup만 돌릴 땐 `ondemand`로도 충분합니다.

---

## 6. ROS_DOMAIN_ID 설정 (PC ↔ 로봇 통신)

PC와 로봇이 서로의 토픽을 보려면 **같은 `ROS_DOMAIN_ID`** 를 써야 합니다. `~/.bashrc` 끝에 추가:

```bash
export ROS_DOMAIN_ID=20          # PC·로봇 동일 값으로
source /opt/ros/jazzy/setup.bash
source ~/tribo_ws/install/setup.bash
```

> alias만 정의해 두고 매번 수동 실행하면 노드별로 도메인이 어긋날 수 있으니, 위처럼 **export로 고정**하는 것을 권장합니다.

---

## 7. 실행

### 7-1. 실제 로봇 bringup (보드 + 오도메트리 + 라이다)

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

### 7-2. 동작 테스트

```bash
# 전진 명령 (안전을 위해 바퀴를 들고 테스트 권장)
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"
```

> bringup에는 **watchdog**이 있어 `cmd_vel`이 `cmd_timeout`(기본 0.5초)초 동안 끊기면 모터를 자동 정지합니다.

### 7-3. 맵 만들기 (SLAM)

SLAM Toolbox로 지도를 작성합니다. (`ros-jazzy-slam-toolbox` 패키지 필요 — 4-2의 의존성 목록에 포함)

**① 로봇에서: bringup 실행** (모터·오도메트리·라이다·URDF 전체)

```bash
# 로봇 (ssh tribo-robot)
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
# 로봇에서 저장 → /home/tribo/my_map.{yaml,pgm} 생성됨
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

PC에서 저장한 경우, 로봇으로 복사해야 합니다:

```bash
# PC에서 저장했다면
ros2 run nav2_map_server map_saver_cli -f ~/my_map
scp ~/my_map.yaml ~/my_map.pgm tribo-robot:~/
```

> **최종 상태**: 로봇 측 홈(`~`, 예: `/home/tribo/`)에 `my_map.yaml` + `my_map.pgm` 두 파일이 있어야 다음 단계(Navigation)가 동작합니다.

### 7-4. Navigation 실행 (Nav2)

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
  map:=/home/tribo/my_map.yaml \
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
> 위 예시는 로봇에서 `ros2 launch`를 실행하므로 `map:=/home/tribo/my_map.yaml`(로봇 홈) 입니다. 만약 PC에서 launch를 실행한다면 PC 입장 경로(`/home/deeptree/...`)로 줘야 합니다. PC의 `/home/deeptree/...`를 로봇 launch에 넘기면 `map_server` configure가 "파일을 못 찾는다"며 실패합니다.

### 7-5. 시뮬레이션 (Gazebo) — SLAM/Nav 실습

실로봇 없이 PC에서 SLAM·Nav2를 실습할 수 있도록 Gazebo(gz-sim) 환경을 제공합니다.

**구성**
- **로봇 모델**: 2단 트롤리 외형(하단 구동 베이스 + 모터 4 + 상단 프로파일 프레임 + 선반 플레이트 + LCD). 4륜 차동구동(skid-steer).
- **센서**: 2D 라이다(`/scan`), IMU(`/imu`), Depth 카메라(`/camera/image`, `/camera/depth_image`, `/camera/points`, `/camera/camera_info`)
- **월드** (`tribo_gazebo/worlds/tribo_world.world`): 8×6 m 실내 — 방 3개 + 복도 + 문 3개 + 장애물 5개(루프 클로저·장애물 회피 연습용). 외부 모델 의존성 없는 self-contained.
- **odom 보정 완료**: 4륜 스키드 회전 슬립 때문에 `gazebo_control.xacro`의 `wheel_separation`을 물리 트랙(0.50)이 아닌 **유효 트랙 0.65**로 설정. IMU·ground-truth와 비교해 회전·직진 모두 ±2% 일치 검증함.

**① 시뮬 실행**

```bash
ros2 launch tribo_gazebo launch_sim.launch.xml
```
→ 실내 맵 + 로봇(복도 서쪽 `-3, 0`)이 스폰됩니다. (빈 월드는 `launch_sim_empty.launch.xml`)

**② SLAM 맵 작성** (새 터미널)

```bash
ros2 launch tribo_navigation map_building.launch.py use_sim_time:=true use_scan_filter:=false
```

**③ 로봇 운전하며 맵 작성** (새 터미널) — 복도·방을 돌며 루프 닫기

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**④ 맵 저장**

```bash
ros2 run nav2_map_server map_saver_cli -f ~/tribo_map
```

**⑤ Nav2 자율주행** (SLAM 종료 후)

```bash
ros2 launch tribo_navigation bringup_launch.xml \
  map:=$HOME/tribo_map.yaml use_sim_time:=true use_rviz:=false use_scan_filter:=false

# 다른 터미널 — RViz (Nav2 Goal 로 목표 지정)
ros2 launch tribo_navigation nav2_view.launch.xml
```

> **시뮬 주의점**
> - 시뮬에서는 **항상 `use_sim_time:=true`** (안 주면 TF 시간 불일치로 SLAM/Nav 깨짐).
> - `laser_filters` 미설치 시 **`use_scan_filter:=false`**. 실로봇과 동일하게 필터까지 쓰려면 `sudo apt install ros-jazzy-laser-filters` 후 옵션 생략.
> - SLAM과 Nav2는 **동시에 띄우지 말 것** (맵 작성 → 저장 → 종료 → Nav2 순서).

> **⚠️ URDF/xacro 편집 함정 — 주석에 "콜론+공백" 금지**
>
> `tribo_description/urdf/*.xacro` 주석에 `단위: m` 같은 **콜론+공백(`: `)** 패턴을 넣으면, robot_description이 YAML로 파싱되며 `yaml.safe_load() failed`로 robot_state_publisher가 죽습니다. `단위 m`, `보정식 - ...` 처럼 콜론을 빼세요.

#### 시뮬 회전 odom 재보정 (필요 시)

스키드 스티어 회전 슬립이 바뀌면 `gazebo_control.xacro`의 `wheel_separation`을 다시 맞춥니다. 제자리 회전 명령을 주고 odom(바퀴)과 IMU(실제) 각속도를 비교:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.6}}"
ros2 topic echo /odom --field twist.twist.angular.z   # RViz 가 쓰는 값
ros2 topic echo /imu  --field angular_velocity.z       # Gazebo 실제 회전
# 새 값 = 현재값 * (odom 각속도 / imu 각속도)
```

---

## 8. 주요 파라미터 파일

| 파일 | 설명 |
|------|------|
| `tribo_bringup/config/bringup.yaml`    | 모터별 gain, PWM 최소듀티, cmd 안전(deadzone/timeout) — 시리얼 `port`는 launch가 해석(5-3) |
| `tribo_bringup/config/motor_calib.yaml` | **로봇별** 모터 gain 오버라이드 (자동 캘리브 결과, 8-2). **gitignore** — 커밋 안 함. 있으면 launch가 bringup.yaml 뒤에 로드해 덮어씀 |
| `tribo_bringup/udev/99-tribo-serial.rules` | USB 장치 고정 이름 udev 규칙 (`/dev/tribo_base`, `/dev/tribo_lidar`) |
| `tribo_bringup/config/robot_geom.yaml` | 공유 기구 파라미터 (track_width, wheel_radius, ticks_per_rev) |
| `tribo_odom/config/odom.yaml`          | 오도메트리 파라미터 |
| `tribo_navigation/config/nav2_params.yaml` | Nav2 설정 |

### 8-1. 회전 odom 캘리브레이션 (실로봇)

4륜 스키드는 제자리/급회전 시 바퀴가 옆으로 헛돌아 휠 odom 회전이 실제보다 과대 적분됩니다. `rotation_calib` 노드가 제자리 회전을 여러 번 시키며 **휠 odom / IMU 자이로(실제) / EKF 융합**의 회전각을 비교해 슬립 정도와 보정값을 산출합니다.

```bash
# 로봇에서 bringup 실행 후, 별도 터미널에서
ros2 run tribo_odom rotation_calib --ros-args \
  -p num_spins:=4 -p spin_duration:=5.0 -p current_track:=0.873
```

출력 해석:
- `wheel/IMU` ≈ 1 이고 변동이 작으면 → 유효 트랙이 맞음. 크면 → `track_true` 평균값을 `bringup.launch.py`의 `_odom_common_params` `track_width`에 반영.
- 슬립 변동(CV)이 크면(>15%) 고정 트랙으로는 한계 → **EKF가 회전을 IMU 자이로로 추정**하도록 설정(현재 기본값). `ekf.yaml`에서 `odom0` yaw/vyaw=false, `imu0` vyaw=true.
- `EKF/IMU` ≈ 1 이면 내비가 쓰는 `/odom` 회전이 실제와 일치(양호).

> 회전 방향이 IMU와 엔코더가 반대로 나오면 `bringup.yaml`의 `invert_imu_yaw`를 토글하세요. 결과는 출력만 하며 설정을 자동 수정하지 않습니다.

### 8-2. 모터 게인 자동 캘리브레이션 (실로봇) &nbsp;·&nbsp; 🤖 로봇 전용

로봇마다 4개 모터(m1=FL, m2=RL, m3=RR, m4=FR)의 개체 편차가 있어, 같은 PWM에도 바퀴 속도가 달라 직진이 틀어집니다. `motor_calib` 노드가 **전진 시 4모터의 엔코더 tick rate를 측정 → 가장 느린 모터에 맞춰 나머지 gain을 낮춤**을 반복해, 불균형이 임계치(기본 5%) 미만으로 **수렴할 때까지** 자동 보정합니다.

**결과는 로봇별 파일 `config/motor_calib.yaml`에 저장**되고(gitignore — 값이 로봇마다 다르므로 커밋하지 않음), `bringup.launch.py`가 이 파일이 있으면 `bringup.yaml` 뒤에 오버라이드로 로드합니다. 소스를 고쳐도 install 복사본은 별개라(install config 미반영 함정), 스크립트가 매 반복 `colcon build`로 install에 반영합니다.

```bash
# ⚠️ 반드시 바퀴를 들고(로봇을 받침대에 올려) 실행 — 모터가 실제로 회전합니다.
export ROS_DOMAIN_ID=20
source /opt/ros/jazzy/setup.bash && source ~/tribo_ws/install/setup.bash
bash ~/tribo_ws/src/tribo/tribo_bringup/scripts/motor_calib_converge.sh
# 반복 상한/허용오차 조정: MAX_ITER=8 TOL=0.05 를 앞에 붙여 실행
```

스크립트는 매 반복마다 `bringup 기동 → /encoder_raw 대기 → 캘리브 시퀀스(모터 회전) → gain 기록 → 재빌드 → 재시작`을 수행하고, 끝에 **자동 진단**을 출력합니다:

| VERDICT | 의미 | 조치 |
|---------|------|------|
| `PASS` | 불균형 < TOL 수렴 | 완료. 바닥에서 직진성 확인 후 커밋 |
| `NOT_CONVERGED` | 아직 임계 초과(문제 아님) | 재실행하면 **저장된 gain에서 이어서** 수렴(모터 비선형 때문에 여러 번 필요). 반영은 재부팅 후에도 유지됨 |
| `FAIL_STUCK m{X}` | 모터 X가 거의 안 돎 | `pwm_min_percent` 상향 또는 배선/기계 저항 점검 (gain으로 해결 불가) |
| `FAIL_SIGN m{X}` | 모터 X 회전 방향 반대 | `bringup.yaml`의 `invert_m{X}` 토글 후 재실행 |

수렴 후 확인:
```bash
ros2 param get /tribo_bringup gain_m1     # motor_calib.yaml 값이 나오면 반영됨
```

> - **바퀴를 든 채로는 회전 관련 추천(`turn_scale`, `gain_*_rev_factor`)이 무의미**합니다 — 로봇이 실제로 안 돌아 `yaw_rate≈0`이라 값이 터무니없이 나옵니다. 직진 gain(`gain_m*`)만 신뢰하고 회전 보정은 바닥에서 8-1로 하세요.
> - 전진 시 엔코더가 음수로 찍히는 로봇에서는 레거시 로그에 `[직진] 좌/우 부호` 경고가 뜰 수 있으나, 자동 진단 `VERDICT`가 `FAIL_SIGN`이 아니면 **정상**(거짓 경고)입니다.
> - "가장 느린 모터 기준"이라 빠른 모터가 많이 눌려 **최고 직진 속도가 느린 모터 수준으로 제한**됩니다. 균형 대신 속도를 더 살리려면 기준을 평균으로 바꾸는 옵션을 추가할 수 있습니다.

---

## 9. 문제 해결

| 증상 | 확인 |
|------|------|
| SSH 접속 `Connection refused` | ① 로봇에서 `sudo systemctl status ssh` 확인, 없으면 `openssh-server` 설치 (3-2). ② **하드코딩한 옛 IP로 접속하고 있지 않은지 확인** — DHCP로 IP가 바뀐 뒤 그 IP를 *다른 기기*가 가져가면 `timed out`이 아니라 `refused`가 뜬다. `getent hosts tribo-robot.local`로 현재 IP 확인 후, 호스트명 별칭으로 전환 (3-4-1) |
| SSH `REMOTE HOST IDENTIFICATION HAS CHANGED` / `Host key verification failed` | 같은 IP에 새 로봇을 올린 경우. PC에서 `ssh-keygen -R <robot-ip>`로 옛 호스트키 제거 후 재접속 (3-5) |
| `could not open port ...` | `dialout` 그룹 추가 후 재로그인 했는지(5-1), 보드가 연결됐는지 |
| `ModuleNotFoundError: serial` | `sudo apt install python3-serial` |
| PC에서 publish해도 로봇이 안 움직임 | PC·로봇 `ROS_DOMAIN_ID` 일치 여부 |
| 보드·라이다가 서로 포트를 뺏음 (`multiple access` / 라이다 `OPERATION_TIMEOUT`) | udev 규칙 설치(5-2). 미설치 시에도 launch가 by-id로 해석하지만, udev 설치를 권장 |
| bringup 중복 실행 시 `multiple access on port` | `bringup.launch.py`가 시작 시 이전 인스턴스를 자동 정리함. 끄려면 `TRIBO_AUTOCLEAN=0` |
| 라이다 `/scan` 안 나옴 | `ls /dev/tribo_lidar`(또는 by-id) 존재 여부, 라이다 전원/USB 연결 |
| `sllidar_ros2` 빌드 누락 | `git submodule update --init --recursive` 후 재빌드 |
| Nav2 `get_state service is not available! Aborting bringup` / `ekf_node: Failed to meet update rate!` | CPU 거버너를 `performance`로(5-4). Pi5에서 거의 항상 이 원인. RViz는 PC에서(`use_rviz:=false`) |
| nav/bringup을 Ctrl-C로 껐는데 노드가 안 죽고 남음 | `ros2 launch`는 Ctrl-C 한 번만 처리(이후 무시), sllidar는 종료 느림. 재실행 전 `pkill -9 -f 'ros-args'`로 잔여 노드 정리 (bringup auto-clean은 nav 노드는 안 건드림) |
| `odom_publisher: No package metadata was found for tribo-odom` | entry point 추가 후 증분 빌드 깨짐. `rm -rf build/tribo_odom install/tribo_odom src/tribo/tribo_odom/tribo_odom.egg-info && colcon build --packages-select tribo_odom --symlink-install` |

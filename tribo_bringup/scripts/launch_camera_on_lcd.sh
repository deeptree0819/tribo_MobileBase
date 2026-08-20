#!/usr/bin/env bash
# 로봇에 연결된 LCD(GNOME Wayland 세션의 Xwayland)에 카메라 영상을 전체화면으로 띄운다.
#
# tribo_navigation/scripts/launch_rviz_on_lcd.sh 와 같은 방식이다.
# - DISPLAY 는 Xwayland 디스플레이 번호(보통 :0)
# - XAUTHORITY 는 mutter 가 매 부팅마다 임의 접미사로 새로 만드는 파일이므로
#   경로를 하드코딩하면 재부팅 후 깨진다. 호출 시점에 글롭으로 최신 파일을 찾는다.
#
# 왜 rqt_image_view 를 안 쓰나: 창 장식과 툴바가 1024x600 화면의 상당 부분을
# 먹고, mutter 가 SSH 에서 온 리사이즈 요청을 무시해서 크기를 맞출 수도 없다.
# lcd_camera_view.py 는 OpenCV 전체화면 창에 영상만 그린다.
#
# Usage:
#   launch_camera_on_lcd.sh [토픽] [display] [xauthority_glob]
#
# Defaults:
#   토픽             = /camera/camera/color/image_raw
#   display          = :0
#   xauthority_glob  = /run/user/<uid>/.mutter-Xwaylandauth.*
#
# 종료: LCD 에서 q 또는 ESC. SSH 로 띄웠으면 pkill -f lcd_camera_view

set -u

TOPIC="${1:-/camera/camera/color/image_raw}"
DISP="${2:-:0}"
XAUTH_GLOB="${3:-/run/user/$(id -u)/.mutter-Xwaylandauth.*}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VIEWER="${SCRIPT_DIR}/lcd_camera_view.py"

# 이 스크립트는 셸에서 직접 실행하는 것을 전제로 한다(launch_rviz_on_lcd.sh 와 달리
# ros2 launch 가 호출하지 않는다). SSH 비대화형 세션에는 ROS 환경이 없어서
# rclpy import 가 실패하므로 여기서 직접 로드한다.
#
# NOTE: 소싱하는 동안에는 nounset 을 반드시 꺼야 한다. ROS/colcon 의 setup.bash 는
#       미설정 변수를 참조하므로 set -u 상태에서 소싱하면
#       "AMENT_TRACE_SETUP_FILES: unbound variable" 로 죽는다.
#       ROS 계열 setup 스크립트 전반에서 반복되는 함정이다.
set +u
if [ -z "${ROS_DISTRO:-}" ]; then
  for d in /opt/ros/*/setup.bash; do
    [ -f "$d" ] && . "$d" && break
  done
fi

# 워크스페이스가 빌드돼 있으면 함께 로드한다(없어도 이 뷰어는 동작한다).
[ -f "${HOME}/tribo_ws/install/setup.bash" ] && . "${HOME}/tribo_ws/install/setup.bash"
set -u

# ROS_DOMAIN_ID 는 기체마다 다르므로 여기서 정하지 않는다. 환경값을 그대로 쓰되,
# 값이 어긋나면 "카메라가 안 켜졌나?" 로 오해하기 쉬우므로 반드시 눈에 보이게 찍는다.
echo "[launch_camera_on_lcd] ROS_DISTRO=${ROS_DISTRO:-없음} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-미설정(0)}"

if [ ! -f "${VIEWER}" ]; then
  echo "[launch_camera_on_lcd] ERROR: ${VIEWER} 가 없습니다." >&2
  exit 2
fi

# shellcheck disable=SC2086
XAUTH_FILE=$(ls -t ${XAUTH_GLOB} 2>/dev/null | head -n 1)

if [ -z "${XAUTH_FILE}" ] && [ -f "${HOME}/.Xauthority" ]; then
  XAUTH_FILE="${HOME}/.Xauthority"
fi

if [ -z "${XAUTH_FILE}" ]; then
  echo "[launch_camera_on_lcd] WARN: XAUTHORITY 후보를 찾지 못했습니다(${XAUTH_GLOB})." >&2
  echo "                      LCD 에 로그인된 GNOME 세션이 있는지 확인하세요." >&2
fi

echo "[launch_camera_on_lcd] DISPLAY=${DISP} XAUTHORITY=${XAUTH_FILE}"
echo "[launch_camera_on_lcd] topic = ${TOPIC}"

export DISPLAY="${DISP}"
export XAUTHORITY="${XAUTH_FILE}"

# 이전 뷰어가 남아 있으면 화면을 두고 다투므로 정리한다.
# 대괄호는 자기 자신 매칭을 피하려는 것 — 이 스크립트의 명령줄에 패턴 문자열이
# 들어 있으면 pkill 이 스스로를 죽인다(실제로 겪음).
pkill -f "lcd_camera_view[.]py" 2>/dev/null
sleep 1

python3 "${VIEWER}" "${TOPIC}" &
VIEWER_PID=$!

# 전체화면을 창 관리자에게 강제로 요청한다.
#
# 왜 필요한가: OpenCV 의 WND_PROP_FULLSCREEN 은 이 환경(GNOME Wayland + mutter,
# Xwayland 경유)에서 무시된다 — 창이 400x263 으로 뜨는 것을 실측했다.
# xdotool 의 windowsize 도 mutter 가 무시한다. EWMH 표준 요청인
# wmctrl -b add,fullscreen 만 확실하게 먹는다.
if command -v wmctrl >/dev/null 2>&1; then
  for _ in $(seq 1 25); do
    sleep 1
    if wmctrl -l 2>/dev/null | grep -q tribo_lcd_camera; then
      wmctrl -r tribo_lcd_camera -b add,fullscreen 2>/dev/null
      break
    fi
  done
else
  echo "[launch_camera_on_lcd] WARN: wmctrl 이 없어 전체화면을 강제하지 못합니다." >&2
  echo "                      sudo apt install -y wmctrl" >&2
fi

wait "${VIEWER_PID}"

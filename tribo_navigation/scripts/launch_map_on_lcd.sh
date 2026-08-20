#!/usr/bin/env bash
# 매핑 중인 맵을 로봇 LCD 에 자동 맞춤으로 띄운다.
#
# tribo_bringup/scripts/launch_camera_on_lcd.sh 와 같은 구조다.
# - DISPLAY 는 Xwayland 디스플레이 번호(보통 :0)
# - XAUTHORITY 는 mutter 가 매 부팅마다 임의 접미사로 새로 만들기 때문에
#   경로를 하드코딩하면 재부팅 후 깨진다. 호출 시점에 글롭으로 최신 파일을 찾는다.
#
# Usage:
#   launch_map_on_lcd.sh [맵토픽] [display] [xauthority_glob]
#
# Defaults:
#   맵토픽           = /map
#   display          = :0
#   xauthority_glob  = /run/user/<uid>/.mutter-Xwaylandauth.*
#
# 종료: LCD 에서 q 또는 ESC. SSH 에서는 pkill -f "lcd_map_view[.]py"

set -u

TOPIC="${1:-/map}"
DISP="${2:-:0}"
XAUTH_GLOB="${3:-/run/user/$(id -u)/.mutter-Xwaylandauth.*}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VIEWER="${SCRIPT_DIR}/lcd_map_view.py"

if [ ! -f "${VIEWER}" ]; then
  echo "[launch_map_on_lcd] ERROR: ${VIEWER} 가 없습니다." >&2
  exit 2
fi

# NOTE: 소싱하는 동안에는 nounset 을 반드시 꺼야 한다. ROS/colcon 의 setup.bash 는
#       미설정 변수를 참조하므로 set -u 상태에서 소싱하면
#       "AMENT_TRACE_SETUP_FILES: unbound variable" 로 죽는다.
#       scripts/find_tribos.sh 주석에 적힌 것과 같은 함정이다.
set +u
if [ -z "${ROS_DISTRO:-}" ]; then
  for d in /opt/ros/*/setup.bash; do
    [ -f "$d" ] && . "$d" && break
  done
fi
[ -f "${HOME}/tribo_ws/install/setup.bash" ] && . "${HOME}/tribo_ws/install/setup.bash"
set -u

echo "[launch_map_on_lcd] ROS_DISTRO=${ROS_DISTRO:-없음} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-미설정(0)}"

# shellcheck disable=SC2086
XAUTH_FILE=$(ls -t ${XAUTH_GLOB} 2>/dev/null | head -n 1)

if [ -z "${XAUTH_FILE}" ] && [ -f "${HOME}/.Xauthority" ]; then
  XAUTH_FILE="${HOME}/.Xauthority"
fi

if [ -z "${XAUTH_FILE}" ]; then
  echo "[launch_map_on_lcd] WARN: XAUTHORITY 후보를 찾지 못했습니다(${XAUTH_GLOB})." >&2
  echo "                    LCD 에 로그인된 GNOME 세션이 있는지 확인하세요." >&2
fi

echo "[launch_map_on_lcd] DISPLAY=${DISP} XAUTHORITY=${XAUTH_FILE}"
echo "[launch_map_on_lcd] topic = ${TOPIC}"

export DISPLAY="${DISP}"
export XAUTHORITY="${XAUTH_FILE}"

# 대괄호는 자기 자신 매칭을 피하려는 것 — 이 스크립트의 명령줄에 패턴 문자열이
# 들어 있으면 pkill 이 스스로를 죽인다(실제로 겪음).
pkill -f "lcd_map_view[.]py" 2>/dev/null
sleep 1

python3 "${VIEWER}" "${TOPIC}" &
VIEWER_PID=$!

# 전체화면을 창 관리자에게 강제로 요청한다.
# OpenCV 의 WND_PROP_FULLSCREEN 은 GNOME Wayland(Xwayland 경유)에서 무시된다.
# xdotool 의 windowsize 도 mutter 가 무시한다. EWMH 요청인 wmctrl 만 확실히 먹는다.
if command -v wmctrl >/dev/null 2>&1; then
  for _ in $(seq 1 25); do
    sleep 1
    if wmctrl -l 2>/dev/null | grep -q tribo_lcd_map; then
      wmctrl -r tribo_lcd_map -b add,fullscreen 2>/dev/null
      break
    fi
  done
else
  echo "[launch_map_on_lcd] WARN: wmctrl 이 없어 전체화면을 강제하지 못합니다." >&2
  echo "                    sudo apt install -y wmctrl" >&2
fi

wait "${VIEWER_PID}"

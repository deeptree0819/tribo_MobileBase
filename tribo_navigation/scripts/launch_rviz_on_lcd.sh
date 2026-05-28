#!/usr/bin/env bash
# 로봇에 연결된 LCD(GNOME Wayland 세션의 Xwayland)에 RViz2를 전체화면으로 띄운다.
#
# SSH 비대화형 세션에서 ros2 launch가 이 스크립트를 호출하는 시나리오를 가정.
# - DISPLAY 는 Xwayland 디스플레이 번호(보통 :0)
# - XAUTHORITY 는 mutter가 매 부팅마다 새로 만드는 random suffix 파일이므로
#   호출 시점에 글롭으로 가장 최신 파일을 찾아서 잡아준다.
#
# Usage:
#   launch_rviz_on_lcd.sh <rviz_config_path> [display] [xauthority_glob]
#
# Defaults:
#   display          = :0
#   xauthority_glob  = /run/user/<uid>/.mutter-Xwaylandauth.*

set -u

RVIZ_CONFIG="${1:-}"
DISP="${2:-:0}"
XAUTH_GLOB="${3:-/run/user/$(id -u)/.mutter-Xwaylandauth.*}"

if [ -z "${RVIZ_CONFIG}" ]; then
  echo "[launch_rviz_on_lcd] ERROR: rviz_config 인자가 비었습니다." >&2
  exit 2
fi

# shellcheck disable=SC2086
XAUTH_FILE=$(ls -t ${XAUTH_GLOB} 2>/dev/null | head -n 1)

if [ -z "${XAUTH_FILE}" ] && [ -f "${HOME}/.Xauthority" ]; then
  XAUTH_FILE="${HOME}/.Xauthority"
fi

if [ -z "${XAUTH_FILE}" ]; then
  echo "[launch_rviz_on_lcd] WARN: XAUTHORITY 후보를 찾지 못했습니다(${XAUTH_GLOB})." >&2
  echo "                    LCD에 로그인된 GNOME 세션이 있는지 확인하세요." >&2
fi

echo "[launch_rviz_on_lcd] DISPLAY=${DISP} XAUTHORITY=${XAUTH_FILE}"
echo "[launch_rviz_on_lcd] rviz config = ${RVIZ_CONFIG}"

export DISPLAY="${DISP}"
export XAUTHORITY="${XAUTH_FILE}"

exec rviz2 -d "${RVIZ_CONFIG}" --fullscreen

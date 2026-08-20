#!/usr/bin/env bash
# 로봇에 연결된 LCD(GNOME Wayland 세션의 Xwayland)에 RViz2를 전체화면으로 띄운다.
#
# SSH 비대화형 세션에서 ros2 launch가 이 스크립트를 호출하는 시나리오를 가정.
# - DISPLAY 는 Xwayland 디스플레이 번호(보통 :0)
# - XAUTHORITY 는 mutter가 매 부팅마다 새로 만드는 random suffix 파일이므로
#   호출 시점에 글롭으로 가장 최신 파일을 찾아서 잡아준다.
#
# map_yaml 을 주면 그 맵이 화면에 꼭 맞도록 설정을 새로 만들어 쓴다.
# 안 주면 넘겨받은 설정을 그대로 쓴다(기존 동작).
#
# Usage:
#   launch_rviz_on_lcd.sh <rviz_config_path> [display] [xauthority_glob] [map_yaml]
#
# Defaults:
#   display          = :0
#   xauthority_glob  = /run/user/<uid>/.mutter-Xwaylandauth.*
#   map_yaml         = (없음 — 맞춤 없이 원본 설정 사용)

set -u

RVIZ_CONFIG="${1:-}"
DISP="${2:-:0}"
XAUTH_GLOB="${3:-/run/user/$(id -u)/.mutter-Xwaylandauth.*}"
MAP_YAML="${4:-}"

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

export DISPLAY="${DISP}"
export XAUTHORITY="${XAUTH_FILE}"

# 맵 크기에 맞춰 배율·중심을 다시 계산한다.
#
# 왜 필요한가: navigation.rviz 의 Scale 은 고정값(89.86)이라 맵이 바뀌면 일부만
# 보인다. 실측에서 17.4x13.85 m 맵의 40% 정도만 화면에 들어왔다. 로봇에는 마우스가
# 없어 실행 중 휠로 맞출 수도 없다.
#
# 생성 실패는 치명적이지 않다 — 원본 설정으로 그냥 띄운다.
if [ -n "${MAP_YAML}" ] && [ -f "${MAP_YAML}" ]; then
  SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
  FITTER="${SCRIPT_DIR}/fit_rviz_to_map.py"
  FITTED="/tmp/tribo_nav_fitted.rviz"
  if [ -f "${FITTER}" ]; then
    if python3 "${FITTER}" "${MAP_YAML}" --base "${RVIZ_CONFIG}" -o "${FITTED}"; then
      RVIZ_CONFIG="${FITTED}"
    else
      echo "[launch_rviz_on_lcd] WARN: 맵 맞춤 실패 — 원본 설정으로 진행합니다." >&2
    fi
  fi
fi

echo "[launch_rviz_on_lcd] DISPLAY=${DISP} XAUTHORITY=${XAUTH_FILE}"
echo "[launch_rviz_on_lcd] rviz config = ${RVIZ_CONFIG}"

exec rviz2 -d "${RVIZ_CONFIG}" --fullscreen

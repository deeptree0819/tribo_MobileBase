#!/usr/bin/env bash
# =============================================================================
#  motor_calib_converge.sh — 방식 A 오케스트레이션
#  측정 → gain 기록(config/motor_calib.yaml) → bringup 재시작 을 직진 tick rate
#  불균형이 임계치(converge_tol) 미만으로 수렴할 때까지 자동 반복한다.
#
#  ┌───────────────────────────────────────────────────────────────────────┐
#  │  ⚠️  바퀴를 들고(로봇을 들어올려) 실행하세요. 모터가 실제로 회전합니다.  │
#  │  ⚠️  LIFT THE WHEELS OFF THE GROUND — the motors WILL spin.            │
#  └───────────────────────────────────────────────────────────────────────┘
#
#  사용법:
#    bash scripts/motor_calib_converge.sh
#  환경변수로 조정 가능:
#    MAX_ITER (기본 5)  TOL (기본 0.05)  ENC_TIMEOUT (기본 20s)
#    ROS_DOMAIN_ID (기본 20)
# =============================================================================
set -uo pipefail

# ---- 설정 ----
ROS_DISTRO_SETUP="/opt/ros/jazzy/setup.bash"
WS="${WS:-$HOME/tribo_ws}"
MAX_ITER="${MAX_ITER:-5}"
TOL="${TOL:-0.05}"
ENC_TIMEOUT="${ENC_TIMEOUT:-20}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"

# calib yaml = 이 스크립트 기준 ../config/motor_calib.yaml (소스 트리)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CALIB_YAML="$(cd "$SCRIPT_DIR/.." && pwd)/config/motor_calib.yaml"

BRINGUP_LOG="/tmp/tribo_calib_bringup.log"
BRINGUP_PID=""

# ---- 배너 ----
cat <<'BANNER'
=============================================================================
  ⚠️  WHEELS UP / 바퀴를 들고 실행하세요 — 모터가 실제로 회전합니다.
=============================================================================
BANNER
echo "  WS=$WS  MAX_ITER=$MAX_ITER  TOL=$TOL  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  CALIB_YAML=$CALIB_YAML"
echo

# ---- ROS 환경 ----
# ROS setup.bash 는 nounset(set -u) 하에서 미설정 변수(AMENT_TRACE_SETUP_FILES 등)를
# 참조하다 죽으므로, source 구간만 -u 를 잠시 끈다.
set +u
# shellcheck disable=SC1090
source "$ROS_DISTRO_SETUP"
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

# ---- 안전 정리 (종료/인터럽트 시 cmd_vel=0 + bringup kill) ----
stop_bringup() {
  if [[ -n "$BRINGUP_PID" ]]; then
    # setsid 로 새 세션 리더가 되었으므로 프로세스 그룹 전체에 시그널
    kill -TERM -- "-$BRINGUP_PID" 2>/dev/null || true
    sleep 2
    kill -KILL -- "-$BRINGUP_PID" 2>/dev/null || true
    BRINGUP_PID=""
  fi
  # 백스톱: 남은 노드 정리
  pkill -f 'tribo_bringup.bringup' 2>/dev/null || true
  pkill -f 'bringup.launch.py' 2>/dev/null || true
}

_CLEANED=0
cleanup() {
  # 재진입 방지: Ctrl-C 연타로 cleanup 이 중첩 실행되며 매달리는 것을 막는다.
  [[ "$_CLEANED" == 1 ]] && return
  _CLEANED=1
  trap - EXIT INT TERM
  echo
  echo "[cleanup] cmd_vel=0 발행 + bringup 정리"
  # pub 이 구독자/데몬 문제로 매달리지 않도록 timeout 으로 바운드
  timeout 5 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 || true
  stop_bringup
}
trap cleanup EXIT INT TERM

# ---- /encoder_raw 대기 ----
wait_for_encoder() {
  local deadline=$((SECONDS + ENC_TIMEOUT))
  while (( SECONDS < deadline )); do
    if timeout 3 ros2 topic echo --once /encoder_raw >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

# motor_calib.yaml 에서 gain_m1~4 를 읽어 표로 출력 (yaml 파싱은 python3)
print_final_gains() {
  python3 - "$CALIB_YAML" <<'PY'
import sys, yaml
path = sys.argv[1]
try:
    with open(path) as f:
        d = yaml.safe_load(f) or {}
    p = (d.get("tribo_bringup") or {}).get("ros__parameters") or {}
    for i in (1, 2, 3, 4):
        v = p.get(f"gain_m{i}", "N/A")
        print(f"    gain_m{i} = {v}")
except Exception as e:  # noqa: BLE001
    print(f"    (yaml 파싱 실패: {e})")
PY
}

CONVERGED=0
FINAL_STATE=""        # converged | FAIL_STUCK | FAIL_SIGN | max_iter
LAST_VERDICT=""
LAST_IMB=""
LAST_ITER=0

for (( iter=1; iter<=MAX_ITER; iter++ )); do
  LAST_ITER=$iter
  echo "============================================================"
  echo "[iter $iter/$MAX_ITER] bringup 기동 (use_lidar:=false use_odom:=true)"
  echo "============================================================"

  # bringup 을 새 세션(setsid)으로 백그라운드 기동 -> PID=세션리더=PGID
  setsid bash -c "exec ros2 launch tribo_bringup bringup.launch.py \
      use_lidar:=false use_odom:=true" >"$BRINGUP_LOG" 2>&1 &
  BRINGUP_PID=$!

  echo "[iter $iter] /encoder_raw 대기 (<= ${ENC_TIMEOUT}s) ..."
  if ! wait_for_encoder; then
    echo "[iter $iter] ERROR: /encoder_raw 타임아웃. bringup 로그: $BRINGUP_LOG"
    tail -n 20 "$BRINGUP_LOG" || true
    stop_bringup
    exit 1
  fi
  echo "[iter $iter] /encoder_raw OK. 캘리브 시퀀스 실행 (모터 회전!)"

  # 캘리브 노드 실행 (write_yaml). 출력에서 CONVERGED/NOT_CONVERGED 캡처.
  CALIB_OUT="$(ros2 run tribo_bringup motor_calib --ros-args \
      -p write_yaml:=true \
      -p converge_tol:="$TOL" \
      -p calib_yaml_path:="$CALIB_YAML" 2>&1)" || true
  echo "$CALIB_OUT"

  # bringup 정리 (다음 iter 는 새 gain 으로 재빌드 후 재기동)
  stop_bringup
  sleep 1

  # 머신 판정 라인(VERDICT:) 캡처
  VLINE="$(echo "$CALIB_OUT" | grep -oE 'VERDICT: (PASS|FAIL_STUCK|FAIL_SIGN|NOT_CONVERGED)' | tail -1)"
  LAST_VERDICT="${VLINE#VERDICT: }"

  # 수렴 판정 캡처 (NOT_CONVERGED 를 먼저 검사 — CONVERGED 부분문자열 포함)
  if echo "$CALIB_OUT" | grep -q 'NOT_CONVERGED imbalance='; then
    LAST_IMB="$(echo "$CALIB_OUT" | grep -oE 'NOT_CONVERGED imbalance=[0-9.]+' | tail -1 | sed 's/.*imbalance=//')"
    echo "[iter $iter] imbalance=$LAST_IMB verdict=${LAST_VERDICT:-?} -> 아직 미수렴."
  elif echo "$CALIB_OUT" | grep -q 'CONVERGED imbalance='; then
    LAST_IMB="$(echo "$CALIB_OUT" | grep -oE 'CONVERGED imbalance=[0-9.]+' | tail -1 | sed 's/.*imbalance=//')"
    echo "[iter $iter] imbalance=$LAST_IMB verdict=${LAST_VERDICT:-?} -> 수렴 후보!"
    CONVERGED=1
  else
    echo "[iter $iter] WARN: CONVERGED/NOT_CONVERGED 라인을 못 찾음. 캘리브 출력 확인 필요."
  fi

  # 하드 페일(STUCK/SIGN)은 gain 재조정으로 해결 불가 → 즉시 중단
  if [[ "$LAST_VERDICT" == "FAIL_STUCK" || "$LAST_VERDICT" == "FAIL_SIGN" ]]; then
    FINAL_STATE="$LAST_VERDICT"
    echo "[iter $iter] ❌ $LAST_VERDICT 감지 -> 반복 중단 (재빌드 생략)."
    break
  fi

  # 새 motor_calib.yaml 을 install share 로 반영 (install 은 소스와 분리 복사본)
  echo "[iter $iter] colcon build --packages-select tribo_bringup ..."
  ( cd "$WS" && colcon build --packages-select tribo_bringup ) \
    || { echo "[iter $iter] ERROR: colcon build 실패"; exit 1; }
  # 빌드로 setup.bash 재생성 -> 재-source (nounset 잠시 해제)
  set +u
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  set -u

  # 현재 gain 값 echo
  echo "[iter $iter] 현재 motor_calib.yaml gain:"
  grep -E 'gain_m[1-4]' "$CALIB_YAML" 2>/dev/null | sed 's/^/    /' || true

  if (( CONVERGED == 1 )); then
    FINAL_STATE="converged"
    echo
    echo "=== 수렴 완료 (iter $iter). 종료합니다. ==="
    break
  fi
done

# max_iter 도달까지 어떤 종료 상태도 못 잡았으면 미수렴
if [[ -z "$FINAL_STATE" ]]; then
  FINAL_STATE="max_iter"
fi

# ---- 최종 종합 리포트 ----
echo
echo "============================================================"
echo "  최종 요약"
echo "============================================================"
case "$FINAL_STATE" in
  converged)
    echo "✅ 최종: ${LAST_ITER}회 만에 수렴(PASS)"
    echo "최종 gain (config/motor_calib.yaml):"
    print_final_gains
    ;;
  FAIL_STUCK)
    echo "❌ 최종: FAIL_STUCK — 한 개 이상 모터가 거의 안 돎 (iter ${LAST_ITER})"
    echo "  마지막 진단 블록: 위 [iter ${LAST_ITER}] 출력의 '========== 진단(자동) ==========' 참고"
    echo "  권장 조치: pwm_min_percent 상향 또는 배선/기계 저항 점검"
    echo "현재 gain (config/motor_calib.yaml):"
    print_final_gains
    ;;
  FAIL_SIGN)
    echo "❌ 최종: FAIL_SIGN — 모터 회전 방향 반대 (iter ${LAST_ITER})"
    echo "  마지막 진단 블록: 위 [iter ${LAST_ITER}] 출력의 '========== 진단(자동) ==========' 참고"
    echo "  권장 조치: bringup.yaml invert_m{X} 토글 후 재실행 (X=진단 블록의 SIGN_FLIP 모터)"
    echo "현재 gain (config/motor_calib.yaml):"
    print_final_gains
    ;;
  max_iter)
    echo "⚠️ 최종: max_iter(${MAX_ITER}) 도달, 미수렴"
    echo "  마지막 imbalance=${LAST_IMB:-N/A} (tol=$TOL)"
    echo "  비선형/포화 가능성 — pwm 포화 여부, gain 범위, tol 완화를 검토하세요."
    echo "현재 gain (config/motor_calib.yaml):"
    print_final_gains
    ;;
esac

echo
echo "검증: bringup 을 (모터 명령 없이) 띄운 뒤"
echo "  ros2 param get /tribo_bringup gain_m3"

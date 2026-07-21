#!/usr/bin/env bash
# 같은 WiFi(서브넷)에 붙어 있는 tribo 기체를 찾아 IP·serial 을 출력한다.
#
# 왜 필요한가: 양산 기체는 hostname 이 전부 "tribo-robot" 으로 같고 IP 는 DHCP 라
# 계속 바뀐다. mDNS(tribo-robot.local) 도 이 망에선 안 잡힌다. 그래서
# "SSH 키로 붙어지는가" + "/proc/cpuinfo 의 Pi serial" 로 기체를 특정한다.
#
# 사용법:
#   ./scripts/find_tribos.sh              # 현재 서브넷 자동 탐지
#   ./scripts/find_tribos.sh 172.16.200   # 서브넷 직접 지정
#
# NOTE: 일부러 `set -u`(nounset)를 쓰지 않는다. 실수로 이 파일을 실행이 아니라
#       source 하면 그 옵션이 호출한 셸에 남고, 이후 colcon 의 setup.bash 가
#       "COLCON_CURRENT_PREFIX: unbound variable" 로 깨진다(2026-07-20 실제 발생).
#       변수는 아래처럼 ${VAR:-} 로 개별 방어한다.

USER_NAME="${TRIBO_USER:-tribo}"

# 서브넷 결정 (인자 > 현재 IP 에서 유추)
if [ $# -ge 1 ]; then
  SUBNET="$1"
else
  MYIP=$(ip -4 addr show scope global 2>/dev/null | grep -oE 'inet [0-9.]+' | awk '{print $2}' | head -1)
  if [ -z "${MYIP:-}" ]; then
    echo "현재 IP 를 못 찾았다. 서브넷을 직접 넘겨라: $0 172.16.200" >&2
    exit 1
  fi
  SUBNET="${MYIP%.*}"
fi

echo "탐색 중: ${SUBNET}.0/24 (계정 ${USER_NAME}) ..."

# 22번 열린 호스트만 추림 (전체 SSH 시도보다 훨씬 빠름)
HOSTS=$(nmap -n -Pn -p22 --open -oG - "${SUBNET}.0/24" 2>/dev/null \
        | awk '/22\/open/{print $2}')

if [ -z "${HOSTS:-}" ]; then
  echo "SSH 열린 호스트가 없다."
  exit 0
fi

FOUND=0
for h in $HOSTS; do
  # BatchMode: 비밀번호 프롬프트 없이 키로만 시도 → 우리 기체만 통과
  info=$(timeout 7 ssh -o StrictHostKeyChecking=accept-new -o BatchMode=yes \
           -o ConnectTimeout=5 "${USER_NAME}@${h}" \
           'echo "$(grep -i ^Serial /proc/cpuinfo | awk "{print \$3}") | $(uptime -p)"' 2>/dev/null)
  if [ -n "${info:-}" ]; then
    printf '  %-16s serial=%s\n' "$h" "$info"
    FOUND=$((FOUND+1))
  fi
done

if [ "$FOUND" -eq 0 ]; then
  echo "tribo 를 못 찾았다. 로봇 전원/WiFi 연결을 확인할 것."
else
  echo "총 ${FOUND} 대."
  echo "접속하려면 ~/.ssh/config 의 'Host tribo-robot' HostName 을 위 IP 로 바꾼다."
fi

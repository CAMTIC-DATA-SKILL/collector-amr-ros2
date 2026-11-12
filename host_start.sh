#!/usr/bin/env bash
# 로봇 PC에서 collector 를 띄운다.
# ZMQ_PUB_BIND=true 이고 컨테이너 네트워크가 host 가 아니면, 호스트 포트를 컨테이너 PUB 으로 넘긴다.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${AMR_CONTAINER:-camtic_dev}"
PORT=5570
PUB_BIND=true
ENDPOINT=""

if [[ -f "${ROOT}/.env" ]]; then
  ENDPOINT="$(grep -E '^[[:space:]]*ZMQ_PUB_ENDPOINT=' "${ROOT}/.env" | tail -1 | cut -d= -f2- | tr -d ' "'\''\r' || true)"
  parsed="${ENDPOINT##*:}"
  if [[ "${parsed}" =~ ^[0-9]+$ ]]; then
    PORT="${parsed}"
  fi
  bind_val="$(grep -E '^[[:space:]]*ZMQ_PUB_BIND=' "${ROOT}/.env" | tail -1 | cut -d= -f2- | tr -d ' "'\''\r' || true)"
  if [[ "${bind_val}" == "false" || "${bind_val}" == "0" ]]; then
    PUB_BIND=false
  fi
fi

echo "container=${CONTAINER} pub_bind=${PUB_BIND} endpoint=${ENDPOINT:-<default>} pub_port=${PORT}"

ready=""
for _ in $(seq 1 30); do
  ready="$(docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null || true)"
  if [[ "${ready}" == "true" ]]; then
    break
  fi
  echo "컨테이너 ${CONTAINER} 대기 중"
  sleep 2
done

if [[ "${ready}" != "true" ]]; then
  echo "컨테이너 ${CONTAINER} 가 실행 중이 아닙니다." >&2
  exit 1
fi

MODE="$(docker inspect -f '{{.HostConfig.NetworkMode}}' "${CONTAINER}")"
echo "network=${MODE}"

RELAY_PID=""
DOCKER_PID=""
cleanup() {
  if [[ -n "${DOCKER_PID}" ]]; then
    kill "${DOCKER_PID}" 2>/dev/null || true
  fi
  if [[ -n "${RELAY_PID}" ]]; then
    kill "${RELAY_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'cleanup; exit 143' INT TERM

if [[ "${PUB_BIND}" == "false" ]]; then
  echo "PUB 은 미들웨어로 connect 한다. 호스트 포트 relay 는 열지 않는다."
elif [[ "${MODE}" != "host" ]]; then
  CIP="$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{"\n"}}{{end}}' "${CONTAINER}" | awk 'NF { print; exit }')"
  if [[ -z "${CIP}" ]]; then
    echo "컨테이너 IP 를 찾지 못했습니다. network=${MODE}" >&2
    echo "엣지에서 ${PORT} 으로 접속할 수 없습니다. 컨테이너를 host 네트워크로 두거나 포트 출판이 필요합니다." >&2
    exit 1
  fi
  echo "relay 0.0.0.0:${PORT} -> ${CIP}:${PORT}"
  python3 "${ROOT}/script/tcp_relay.py" "${PORT}" "${CIP}" "${PORT}" &
  RELAY_PID=$!
  sleep 0.3
  if ! kill -0 "${RELAY_PID}" 2>/dev/null; then
    echo "TCP relay 가 바로 종료되었습니다. 호스트 ${PORT} 포트가 이미 쓰이는지 확인하세요." >&2
    exit 1
  fi
fi

docker exec -i "${CONTAINER}" bash "${ROOT}/run.sh" &
DOCKER_PID=$!
set +e
wait "${DOCKER_PID}"
status=$?
set -e
DOCKER_PID=""
exit "${status}"

#!/usr/bin/env bash
# camtic_dev 안에서 ROS 2 환경을 맞춘 뒤 collector 를 실행한다.
# 로봇 PC에서 실행하면 camtic_dev 로 다시 들어간다. 엣지 포트 개방은 host_start.sh 가 한다.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${AMR_CONTAINER:-camtic_dev}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  if [[ "${COLLECTOR_AMR_INNER:-}" == "1" ]]; then
    echo "컨테이너 안에 /opt/ros/humble/setup.bash 가 없습니다. camtic_dev 가 맞는지 확인하세요." >&2
    exit 1
  fi
  if ! command -v docker >/dev/null 2>&1; then
    echo "ROS 2 Humble 이 없습니다. 로봇 PC에서 docker 로 camtic_dev 에 들어가 실행하세요." >&2
    exit 1
  fi
  echo "컨테이너 ${CONTAINER} 안에서 실행합니다."
  exec docker exec -e COLLECTOR_AMR_INNER=1 -i "${CONTAINER}" bash "${ROOT}/run.sh"
fi

export PYTHONUNBUFFERED=1

source_if() {
  local file="$1"
  if [[ -f "${file}" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${file}"
    set -u
  fi
}

set +u
source_if /opt/ros/humble/setup.bash
if [[ -f /home/camtic/camtic_ws/install/setup.bash ]]; then
  source_if /home/camtic/camtic_ws/install/setup.bash
elif [[ -f /root/camtic_ws/install/setup.bash ]]; then
  source_if /root/camtic_ws/install/setup.bash
fi

# bashrc 는 비대화형에서 return 하므로 export 줄만 가져온다. $() 와 따옴표 명령은 실행하지 않는다.
while IFS= read -r line; do
  [[ "${line}" == *'$'* || "${line}" == *'`'* || "${line}" == *';'* ]] && continue
  if [[ "${line}" =~ ^[[:space:]]*export[[:space:]]+(ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION|CYCLONEDDS_URI|FASTRTPS_DEFAULT_PROFILES_FILE)=\"?[A-Za-z0-9_.:/=-]*\"?[[:space:]]*$ ]]; then
    # shellcheck disable=SC2163
    eval "${line}"
  fi
done < <(grep -hE '^[[:space:]]*export[[:space:]]+' /root/.bashrc /root/.profile /etc/bash.bashrc /etc/profile.d/*.sh 2>/dev/null || true)

adopt_peer_env() {
  local pid=""
  pid="$(ps -eo pid=,cmd= | awk '/[s]cout_base_node/ { print $1; exit }')"
  if [[ -z "${pid}" ]]; then
    pid="$(ps -eo pid=,cmd= | awk '/[s]lam_toolbox|[c]omponent_container/ { print $1; exit }')"
  fi
  if [[ -z "${pid}" || ! -r "/proc/${pid}/environ" ]]; then
    return 1
  fi
  echo "ROS 환경을 실행 중인 프로세스 ${pid} 에 맞춥니다."
  local line key
  local allow='|ROS_DOMAIN_ID|ROS_DISTRO|ROS_VERSION|ROS_PYTHON_VERSION|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION|CYCLONEDDS_URI|FASTRTPS_DEFAULT_PROFILES_FILE|AMENT_PREFIX_PATH|CMAKE_PREFIX_PATH|COLCON_PREFIX_PATH|PYTHONPATH|LD_LIBRARY_PATH|ROS_PACKAGE_PATH|'
  while IFS= read -r line; do
    key="${line%%=*}"
    case "${allow}" in
      *"|${key}|"*) export "${line}" ;;
    esac
  done < <(tr '\0' '\n' < "/proc/${pid}/environ")
}

adopt_peer_env || true

apply_dotenv_ros() {
  local key value
  [[ -f "${ROOT}/.env" ]] || return 0
  for key in ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE; do
    value="$(grep -E "^[[:space:]]*${key}=" "${ROOT}/.env" | tail -1 | cut -d= -f2- | tr -d '"' | tr -d "'" | tr -d '\r' || true)"
    if [[ -n "${value}" ]]; then
      export "${key}=${value}"
    fi
  done
}

if [[ ! -f "${ROOT}/.env" && -f "${ROOT}/.env.example" ]]; then
  cp "${ROOT}/.env.example" "${ROOT}/.env"
  echo ".env 가 없어 .env.example 을 복사했습니다."
fi
apply_dotenv_ros
set -u

echo "ROS_DISTRO=${ROS_DISTRO:-} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<default>} CYCLONEDDS_URI=${CYCLONEDDS_URI:-<unset>}"

if ! python3 -c 'import rclpy' >/dev/null 2>&1; then
  echo "rclpy 를 불러오지 못했습니다. PYTHONPATH=${PYTHONPATH:-}" >&2
  exit 1
fi

if ! python3 -c 'import zmq, dotenv, pydantic' >/dev/null 2>&1; then
  echo "pyzmq, python-dotenv, pydantic 를 설치합니다."
  if ! python3 -m pip install -r "${ROOT}/requirements.txt"; then
    python3 -m pip install --break-system-packages -r "${ROOT}/requirements.txt"
  fi
fi
if ! python3 -c 'import zmq, dotenv, pydantic' >/dev/null 2>&1; then
  echo "설치 후에도 pyzmq/dotenv/pydantic 을 불러오지 못했습니다. python3=$(command -v python3)" >&2
  python3 -c 'import zmq, dotenv, pydantic'
  exit 1
fi

cd "${ROOT}"
exec python3 "${ROOT}/main.py"

#!/usr/bin/env bash
# 로봇 PC(camtic@, 컨테이너 밖)에서 systemd 유닛을 등록한다.
# 사용: sudo bash service_add.sh
set -euo pipefail

SERVICE_NAME="collector-amr-ros2"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_DST="/etc/systemd/system/${SERVICE_NAME}.service"

if [[ "${EUID}" -ne 0 ]]; then
  echo "root 권한이 필요합니다. 예: sudo bash service_add.sh" >&2
  exit 1
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  echo "지금 camtic_dev 안입니다. exit 로 로봇 PC에 나온 뒤 sudo bash service_add.sh" >&2
  exit 1
fi

case "${ROOT}" in
  /home/camtic/*) ;;
  *)
    echo "저장소는 /home/camtic 아래에 있어야 컨테이너와 같은 경로로 보입니다." >&2
    echo "현재 경로: ${ROOT}" >&2
    exit 1
    ;;
esac

if ! command -v docker >/dev/null 2>&1; then
  echo "docker 가 없습니다. 로봇 PC에서 실행하세요." >&2
  exit 1
fi

if [[ ! -f "${ROOT}/.env" ]]; then
  echo "경고: ${ROOT}/.env 가 없습니다. .env.example 을 복사해 두세요. run.sh 가 없으면 복사합니다." >&2
fi

chmod +x "${ROOT}/run.sh" "${ROOT}/host_start.sh"

cat > "${UNIT_DST}" <<EOF
[Unit]
Description=AMR ROS 2 collector (camtic_dev -> ZeroMQ)
After=docker.service network-online.target
Wants=docker.service network-online.target

[Service]
Type=simple
WorkingDirectory=${ROOT}
ExecStart=/bin/bash ${ROOT}/host_start.sh
Restart=always
RestartSec=5
TimeoutStopSec=20
KillMode=control-group

StandardOutput=journal
StandardError=journal
SyslogIdentifier=${SERVICE_NAME}

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}.service"
systemctl restart "${SERVICE_NAME}.service"
systemctl --no-pager --full status "${SERVICE_NAME}.service" || true

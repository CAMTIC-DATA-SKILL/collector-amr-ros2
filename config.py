"""모든 환경 변수는 여기서만 읽는다. 저장소 루트의 .env 를 먼저 불러온다 (.env.example 참고).

    from config import settings
"""
import os
import re
from dataclasses import dataclass
from pathlib import Path

from dotenv import load_dotenv

from core.zeromq_client.config import ZmqConfig

BASE_DIR = Path(__file__).resolve().parent
load_dotenv(BASE_DIR / ".env")

_DEVICE_KEY = re.compile(r"[A-Za-z0-9_-]+")


def _env_str(key: str, default: str) -> str:
    value = os.getenv(key)
    return value.strip() if value is not None and value.strip() else default


def _env_int(key: str, default: int) -> int:
    return int(_env_str(key, str(default)))


def _env_float(key: str, default: float) -> float:
    return float(_env_str(key, str(default)))


def _env_bool(key: str, default: bool) -> bool:
    return _env_str(key, str(default)).lower() in ("1", "true", "yes", "on")


def _topic(key: str, default: str) -> str:
    value = _env_str(key, default)
    if value and not value.startswith("/"):
        raise ValueError(f"{key} 는 / 로 시작해야 한다: {value!r}")
    return value


@dataclass(slots=True)
class Settings:
    log_level: str
    device_key: str
    gateway_address: str
    poll_interval: float
    stale_sec: float
    scout_status_topic: str
    odom_topic: str
    amcl_pose_topic: str
    scan_topic: str
    zmq: ZmqConfig


def load_settings() -> Settings:
    device_key = _env_str("DEVICE_KEY", "amr-scout-1")
    if not _DEVICE_KEY.fullmatch(device_key):
        raise ValueError(f"DEVICE_KEY 는 영숫자, '_', '-' 만 쓸 수 있다: {device_key!r}")
    poll_interval = _env_float("POLL_INTERVAL_SEC", 1.0)
    stale_sec = _env_float("STALE_SEC", 3.0)
    if poll_interval <= 0 or stale_sec <= 0:
        raise ValueError("POLL_INTERVAL_SEC 와 STALE_SEC 는 0보다 커야 한다")
    zmq = ZmqConfig()
    return Settings(
        log_level=_env_str("LOG_LEVEL", "INFO").upper(),
        device_key=device_key,
        gateway_address=_env_str("GATEWAY_ADDRESS", "gateway"),
        poll_interval=poll_interval,
        stale_sec=stale_sec,
        scout_status_topic=_topic("SCOUT_STATUS_TOPIC", "/scout_status"),
        odom_topic=_topic("ODOM_TOPIC", "/wheel_odom"),
        amcl_pose_topic=_topic("AMCL_POSE_TOPIC", "/amcl_pose"),
        scan_topic=_topic("SCAN_TOPIC", "/scan"),
        zmq=ZmqConfig(
            pub_endpoint=_env_str("ZMQ_PUB_ENDPOINT", zmq.pub_endpoint),
            sub_endpoint=_env_str("ZMQ_SUB_ENDPOINT", zmq.sub_endpoint),
            pub_bind=_env_bool("ZMQ_PUB_BIND", zmq.pub_bind),
            sub_bind=_env_bool("ZMQ_SUB_BIND", zmq.sub_bind),
            recv_timeout_ms=_env_int("ZMQ_RECV_TIMEOUT_MS", zmq.recv_timeout_ms),
            linger_ms=_env_int("ZMQ_LINGER_MS", zmq.linger_ms),
        ),
    )


settings = load_settings()

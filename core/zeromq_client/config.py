from dataclasses import dataclass


@dataclass(slots=True)
class ZmqConfig:
    """게이트웨이 IPC 용 ZeroMQ 엔드포인트. 환경 변수 값은 루트 config.py 에서 채운다."""

    pub_endpoint: str = "tcp://0.0.0.0:5570"
    sub_endpoint: str = ""
    pub_bind: bool = True
    sub_bind: bool = False
    recv_timeout_ms: int = 100
    linger_ms: int = 0

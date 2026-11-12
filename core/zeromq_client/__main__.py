"""게이트웨이 역할로 collector PUB 을 구독해 출력한다.

예) python -m core.zeromq_client
    python -m core.zeromq_client --endpoint tcp://192.168.1.211:5570 --device-key amr-scout-1
"""
import argparse
import signal
import threading

from config import settings
from model.protocol_model import MsgTypeEnum, ProtocolHeaderDTO

from .sub_client import ZmqSubClient
from .topics import COLLECTOR_KIND, PUB_TOPIC_ROOT


def connect_endpoint(endpoint: str) -> str:
    """PUB 이 0.0.0.0 에 bind 돼 있으면 구독은 127.0.0.1 로 붙는다."""
    return endpoint.replace("://0.0.0.0:", "://127.0.0.1:").replace("://*:", "://127.0.0.1:")


def main() -> None:
    parser = argparse.ArgumentParser(description="ZeroMQ gateway-side echo")
    parser.add_argument("--endpoint", default="", help="collector PUB endpoint (connect). 비우면 .env")
    parser.add_argument("--device-key", default="", help="비우면 collector.robot. 전체")
    args = parser.parse_args()

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    endpoint = connect_endpoint(args.endpoint or settings.zmq.pub_endpoint)
    prefix = f"{PUB_TOPIC_ROOT}.{COLLECTOR_KIND}."
    if args.device_key:
        prefix = f"{prefix}{args.device_key}."
    sub = ZmqSubClient(endpoint, topics=[prefix], recv_timeout_ms=200)
    sub.connect()
    try:
        print(f"listening connect:{endpoint} topic={prefix!r} (Ctrl+C 종료)", flush=True)
        while not stop.is_set():
            header: ProtocolHeaderDTO | None = sub.recv()
            if header is None:
                continue
            if header.msg_type not in (MsgTypeEnum.DATA, MsgTypeEnum.HEALTH, MsgTypeEnum.ACK):
                continue
            print(f"\n[{header.collector_address}.{header.msg_type.value}]", flush=True)
            print(header.model_dump_json(indent=2), flush=True)
    finally:
        sub.close()


if __name__ == "__main__":
    main()

"""프로젝트 진입점. ROS 2 토픽을 읽어 ZeroMQ 로 보낸다.

    ./run.sh

수집은 메인 스레드에서 ROS 를 돌리고, cmd SUB 이 켜져 있으면 별도 스레드에서 ack 한다.
cmd 는 로봇으로 넘기지 않고 E-4102 로 거절한다.
"""
import logging
import os
import signal
import threading
import time
from uuid import uuid4

from pydantic import BaseModel

from config import settings
from core.zeromq_client import ZeroMqClient, ZmqClientError
from model import (
    AckCode,
    MsgAckDTO,
    MsgAckStatusEnum,
    MsgCmdREnum,
    MsgCmdWEnum,
    MsgDataDTO,
    MsgHealthDTO,
    MsgSampleDTO,
    MsgTypeEnum,
    ProtocolHeaderDTO,
    Sample,
)

logger = logging.getLogger("collector")


def now_ms() -> int:
    return int(time.time() * 1000)


def emit(zmq: ZeroMqClient, msg_type: MsgTypeEnum, body: BaseModel, *, gateway_address: str | None = None) -> None:
    header = ProtocolHeaderDTO(
        msg_id=uuid4(),
        gateway_address=gateway_address or settings.gateway_address,
        collector_address=settings.device_key,
        msg_type=msg_type,
        msg_body=body.model_dump(mode="json"),
        timestamp_ms=now_ms(),
    )
    try:
        payload = zmq.send(header)
    except ZmqClientError as exc:
        logger.warning("[%s] zmq pub failed: %s", msg_type.value.upper(), exc)
        return
    if msg_type is MsgTypeEnum.DATA and _has_long_array(body):
        logger.info("[%s] %s", msg_type.value.upper(), _data_brief(body))
        logger.debug("[%s] %s", msg_type.value.upper(), payload)
        return
    logger.info("[%s] %s", msg_type.value.upper(), payload)


def _has_long_array(body: BaseModel) -> bool:
    samples = getattr(body, "samples", None)
    if not isinstance(samples, list):
        return False
    return any(isinstance(getattr(sample, "value", None), list) and len(sample.value) > 16 for sample in samples)


def _data_brief(body: BaseModel) -> str:
    samples = getattr(body, "samples", [])
    parts: list[str] = []
    for sample in samples:
        value = getattr(sample, "value", None)
        addr = getattr(sample, "addr", "")
        if isinstance(value, list):
            parts.append(f"{addr}=n{len(value)}")
        elif addr in {"battery_voltage", "error_code", "odom_x", "odom_y", "odom_yaw", "amcl_x", "amcl_y"}:
            parts.append(f"{addr}={value}")
    return " ".join(parts) if parts else f"samples={len(samples)}"


def emit_health(zmq: ZeroMqClient, *, device_ok: bool, reason: str | None) -> None:
    body = MsgHealthDTO(
        device_key=settings.device_key,
        ipc_ok=True,
        device_ok=device_ok,
        reason=reason,
        observed_ms=now_ms(),
    )
    emit(zmq, MsgTypeEnum.HEALTH, body)


def emit_data(zmq: ZeroMqClient, samples: list[Sample]) -> None:
    body = MsgDataDTO(
        device_key=settings.device_key,
        sample_ms=now_ms(),
        samples=[MsgSampleDTO.model_validate(sample.model_dump()) for sample in samples],
        sockets=[],
    )
    emit(zmq, MsgTypeEnum.DATA, body)


def handle_command(zmq: ZeroMqClient, header: ProtocolHeaderDTO) -> None:
    """cmd_r / cmd_w 는 로봇으로 넘기지 않고 rejected(E-4102) ack 로 응답한다."""
    logger.info("[SUB %s] %s", header.msg_type.value.upper(), header.model_dump_json())
    if header.msg_type is MsgTypeEnum.CMD_R:
        actions = MsgCmdREnum
    elif header.msg_type is MsgTypeEnum.CMD_W:
        actions = MsgCmdWEnum
    else:
        return
    raw_action = header.msg_body.get("action")
    try:
        action = actions(raw_action)
    except ValueError:
        logger.warning("%s action 판별 불가, ack 생략: %r", header.msg_type.value, raw_action)
        return
    body = MsgAckDTO(
        ref_msg_id=header.msg_id,
        device_key=settings.device_key,
        action=action,
        status=MsgAckStatusEnum.REJECTED,
        code=AckCode.CMD_UNSUPPORTED_ACTION,
        reason=AckCode.CMD_UNSUPPORTED_ACTION.name,
    )
    emit(zmq, MsgTypeEnum.ACK, body, gateway_address=header.gateway_address)


def serve_commands(stop: threading.Event, zmq: ZeroMqClient) -> None:
    while not stop.is_set():
        try:
            header = zmq.recv()
        except ZmqClientError as exc:
            logger.warning("zmq sub failed: %s", exc)
            stop.wait(0.2)
            continue
        if header is None or header.collector_address != settings.device_key:
            continue
        handle_command(zmq, header)


def _log_ros_env() -> None:
    for key in (
        "ROS_DISTRO",
        "ROS_DOMAIN_ID",
        "ROS_LOCALHOST_ONLY",
        "RMW_IMPLEMENTATION",
        "CYCLONEDDS_URI",
    ):
        logger.info("%s=%s", key, os.environ.get(key, ""))


def collect(stop: threading.Event, zmq: ZeroMqClient) -> None:
    try:
        from core.ros_bridge import run_ros
    except ImportError as exc:
        raise SystemExit(
            "ROS 2 Python 을 불러오지 못했습니다. camtic_dev 안에서 ./run.sh 로 실행하세요. "
            f"ROS_DISTRO={os.environ.get('ROS_DISTRO', '')!r} ({exc})"
        ) from exc

    device_ok = False
    last_reason: str | None = None
    last_addrs: tuple[str, ...] | None = None
    last_wait_log = 0.0

    def on_cycle(samples: list[Sample], ok: bool, reason: str | None, wait_status: str) -> None:
        nonlocal device_ok, last_reason, last_addrs, last_wait_log
        if ok != device_ok or reason != last_reason:
            logger.info("[HEALTH] device_ok=%s reason=%s", ok, reason)
        emit_health(zmq, device_ok=ok, reason=reason)
        device_ok = ok
        last_reason = reason
        if not ok:
            now = time.monotonic()
            if now - last_wait_log >= 10.0:
                logger.info("토픽 대기: %s", wait_status)
                last_wait_log = now
            return
        addrs = tuple(sample.addr for sample in samples)
        if addrs != last_addrs:
            logger.info("sample addrs: %s", ", ".join(addrs))
            last_addrs = addrs
        emit_data(zmq, samples)

    run_ros(stop, on_cycle)


def main() -> None:
    logging.basicConfig(
        level=settings.log_level,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    logging.getLogger("rclpy").setLevel(logging.WARNING)
    _log_ros_env()
    endpoint = settings.zmq.pub_endpoint
    if settings.zmq.pub_bind and ("127.0.0.1" in endpoint or "localhost" in endpoint):
        logger.warning(
            "ZMQ_PUB_ENDPOINT=%s 에 bind 하면 이 기계 안에서만 보인다. 엣지에서 붙으려면 tcp://0.0.0.0:5570",
            endpoint,
        )
    elif not settings.zmq.pub_bind and "0.0.0.0" in endpoint:
        logger.warning(
            "ZMQ_PUB_ENDPOINT=%s 는 connect 할 주소가 아니다. 미들웨어 주소(예: tcp://192.168.1.111:5556)를 적으세요.",
            endpoint,
        )
    logger.info(
        "device_key=%s poll=%ss stale=%ss topics=%s %s %s %s",
        settings.device_key,
        settings.poll_interval,
        settings.stale_sec,
        settings.scout_status_topic,
        settings.odom_topic,
        settings.amcl_pose_topic,
        settings.scan_topic,
    )

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGHUP, lambda *_: stop.set())

    zmq = ZeroMqClient(settings.zmq)
    try:
        zmq.connect()
    except ZmqClientError as exc:
        logger.error("%s", exc)
        if settings.zmq.pub_bind:
            logger.error("ZMQ_PUB_ENDPOINT 포트가 이미 쓰이면 .env 에서 번호를 바꾸세요.")
        else:
            logger.error("ZMQ_PUB_ENDPOINT 가 미들웨어 SUB 주소인지 확인하세요.")
        raise SystemExit(1) from exc

    cmd_thread = None
    if settings.zmq.sub_endpoint:
        cmd_thread = threading.Thread(target=serve_commands, args=(stop, zmq), name="cmd", daemon=True)
        cmd_thread.start()
    else:
        logger.info("ZMQ_SUB_ENDPOINT 가 비어 cmd 는 구독하지 않는다")

    try:
        collect(stop, zmq)
    finally:
        stop.set()
        if cmd_thread is not None:
            cmd_thread.join(timeout=1.0)
        zmq.close()


if __name__ == "__main__":
    main()

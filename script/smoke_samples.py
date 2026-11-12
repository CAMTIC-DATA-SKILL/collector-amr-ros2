"""ROS 없이 샘플 변환과 ZeroMQ 프레임을 확인한다.

    python script/smoke_samples.py
"""
import json
import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from core.samples import (
    Snapshot,
    assemble_samples,
    flatten_scalars,
    samples_from_odometry,
    samples_from_scan,
    yaw_from_quaternion,
)
from core.zeromq_client.config import ZmqConfig
from core.zeromq_client.sub_client import ZmqSubClient
from core.zeromq_client.topics import topic_for
from core.zeromq_client.zmq_client import ZeroMqClient
from model.protocol_model import MsgDataDTO, MsgSampleDTO, MsgTypeEnum, ProtocolHeaderDTO
from model.sample_model import PointType, Sample


class _Header:
    __slots__ = ("_frame_id",)

    def __init__(self) -> None:
        self._frame_id = "base_link"


class _Motor:
    __slots__ = ("_temperature",)

    def __init__(self, temperature: float) -> None:
        self._temperature = temperature


class _Status:
    __slots__ = ("_header", "_battery_voltage", "_error_code", "_ranges", "_actuator_states")

    def __init__(self) -> None:
        self._header = _Header()
        self._battery_voltage = 24.5
        self._error_code = 0
        self._ranges = [0.1] * 100
        self._actuator_states = [_Motor(30.0), _Motor(31.5)]


def _odom():
    half = math.sqrt(2.0) / 2.0
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1, y=-2, z=0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=half, w=half),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=0.2, y=0.0, z=0.0),
                angular=SimpleNamespace(x=0.0, y=0.0, z=-0.4),
            )
        ),
    )


def _by_addr(samples: list[Sample]) -> dict[str, Sample]:
    return {sample.addr: sample for sample in samples}


def check_yaw() -> None:
    half = math.sqrt(2.0) / 2.0
    yaw = yaw_from_quaternion(0.0, 0.0, half, half)
    if not math.isclose(yaw, math.pi / 2.0, abs_tol=1e-9):
        raise AssertionError(yaw)


def check_odom() -> None:
    found = _by_addr(samples_from_odometry(_odom()))
    if set(found) != {"odom_x", "odom_y", "odom_yaw", "odom_vx", "odom_wz"}:
        raise AssertionError(sorted(found))
    if found["odom_x"].value != 1.0 or found["odom_y"].value != -2.0:
        raise AssertionError((found["odom_x"].value, found["odom_y"].value))
    if not math.isclose(float(found["odom_yaw"].value), math.pi / 2.0, abs_tol=1e-9):
        raise AssertionError(found["odom_yaw"].value)
    if found["odom_x"].type is not PointType.F64:
        raise AssertionError(found["odom_x"].type)


def check_status_skips_arrays() -> None:
    found = _by_addr(flatten_scalars(_Status()))
    expected = {
        "battery_voltage",
        "error_code",
        "actuator_states_0_temperature",
        "actuator_states_1_temperature",
    }
    if set(found) != expected:
        raise AssertionError(sorted(found))
    if found["error_code"].type is not PointType.I64 or found["error_code"].value != 0:
        raise AssertionError(found["error_code"])
    if "ranges_0" in found or "frame_id" in found:
        raise AssertionError(sorted(found))


def check_assemble() -> None:
    now = 100.0
    odom = Snapshot(mono=now, samples=tuple(samples_from_odometry(_odom())))
    samples, ok, reason = assemble_samples(
        now=now, stale_sec=3.0, odom=odom, scout=None, amcl=None, scan=None
    )
    found = _by_addr(samples)
    if not ok or reason is not None:
        raise AssertionError((ok, reason))
    if found["wheel_odom_ok"].value is not True or found["scan_ok"].value is not False:
        raise AssertionError((found["wheel_odom_ok"].value, found["scan_ok"].value))
    if found["amcl_ok"].value is not False or found["scout_status_ok"].value is not False:
        raise AssertionError("status bits")
    stale, stale_ok, stale_reason = assemble_samples(
        now=now, stale_sec=3.0, odom=Snapshot(mono=now - 5.0, samples=odom.samples), scout=None, amcl=None, scan=None
    )
    if stale or stale_ok or stale_reason != "E-1003":
        raise AssertionError((stale, stale_ok, stale_reason))


def check_scan() -> None:
    msg = SimpleNamespace(
        angle_min=-1.0,
        angle_max=1.0,
        angle_increment=0.5,
        time_increment=0.0,
        scan_time=0.1,
        range_min=0.05,
        range_max=12.0,
        ranges=[1.0, float("inf"), 3.5],
        intensities=[0.0, 4.0],
    )
    found = _by_addr(samples_from_scan(msg))
    if found["scan_ranges"].type.value != "f64[]":
        raise AssertionError(found["scan_ranges"].type)
    if list(found["scan_ranges"].value) != [1.0, 12.0, 3.5]:
        raise AssertionError(found["scan_ranges"].value)
    if list(found["scan_intensities"].value) != [0.0, 4.0]:
        raise AssertionError(found["scan_intensities"].value)
    if found["scan_angle_min"].value != -1.0 or found["scan_range_max"].value != 12.0:
        raise AssertionError("scan scalars")


def check_wire() -> None:
    sample = Sample(addr="odom_x", name="odom_x", type=PointType.F64, value=1.25, error=None)
    body = MsgDataDTO(
        device_key="amr-scout-1",
        sample_ms=1,
        samples=[MsgSampleDTO.model_validate(sample.model_dump())],
        sockets=[],
    )
    header = ProtocolHeaderDTO(
        msg_id=uuid4(),
        gateway_address="gateway",
        collector_address="amr-scout-1",
        msg_type=MsgTypeEnum.DATA,
        msg_body=body.model_dump(mode="json"),
        timestamp_ms=1,
    )
    payload = json.loads(header.model_dump_json())
    if payload["msg_type"] != "data" or payload["collector_address"] != "amr-scout-1":
        raise AssertionError(payload)
    body_payload = payload["msg_body"]
    if body_payload["sockets"] != [] or body_payload["samples"][0]["addr"] != "odom_x":
        raise AssertionError(body_payload)
    if body_payload["samples"][0]["type"] != "f64" or body_payload["samples"][0]["value"] != 1.25:
        raise AssertionError(body_payload["samples"][0])
    topic = topic_for("amr-scout-1", MsgTypeEnum.DATA)
    if topic != "collector.robot.amr-scout-1.data" or len(topic.split(".")) != 4:
        raise AssertionError(topic)


def check_zmq() -> None:
    endpoint = "tcp://127.0.0.1:45770"
    client = ZeroMqClient(ZmqConfig(pub_endpoint=endpoint, sub_endpoint="", pub_bind=True))
    sub = ZmqSubClient(endpoint, topics=["collector.robot.amr-scout-1."], recv_timeout_ms=200)
    try:
        client.connect()
        sub.connect()
        time.sleep(0.3)
        header = ProtocolHeaderDTO(
            msg_id=uuid4(),
            gateway_address="gateway",
            collector_address="amr-scout-1",
            msg_type=MsgTypeEnum.HEALTH,
            msg_body={"device_key": "amr-scout-1", "ipc_ok": True, "device_ok": True, "reason": None, "observed_ms": 1},
            timestamp_ms=1,
        )
        got = None
        for _ in range(5):
            client.send(header)
            got = sub.recv()
            if got is not None:
                break
        if got is None or got.msg_type is not MsgTypeEnum.HEALTH or got.collector_address != "amr-scout-1":
            raise AssertionError(got)
    finally:
        sub.close()
        client.close()


def main() -> None:
    checks = [check_yaw, check_odom, check_status_skips_arrays, check_scan, check_assemble, check_wire, check_zmq]
    failed = []
    for check in checks:
        try:
            check()
        except Exception as exc:
            failed.append(check.__name__)
            print(f"[FAIL] {check.__name__}: {exc}", flush=True)
        else:
            print(f"[PASS] {check.__name__}", flush=True)
    if failed:
        print(f"== FAIL {len(failed)}/{len(checks)}: {', '.join(failed)} ==", flush=True)
        raise SystemExit(1)
    print(f"== PASS {len(checks)}/{len(checks)} ==", flush=True)


if __name__ == "__main__":
    main()

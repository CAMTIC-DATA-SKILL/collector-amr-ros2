"""게이트웨이 ↔ collector 프로토콜 DTO.

ZeroMQ multipart [topic, ProtocolHeaderDTO JSON].
토픽은 collector.robot.{device_key}.{data|health|ack}.
샘플 조회 키는 addr 이고 sockets 는 항상 빈 목록이다.
"""
from enum import Enum
from uuid import UUID

from pydantic import BaseModel, Field

from .sample_model import PointType, Value


class MsgTypeEnum(str, Enum):
    CMD_R = "cmd_r"
    CMD_W = "cmd_w"
    ACK = "ack"
    HEALTH = "health"
    DATA = "data"


class MsgCmdREnum(str, Enum):
    SET_SCAN = "SET_SCAN"
    SET_DEVICE = "SET_DEVICE"
    READ_ONCE = "READ_ONCE"
    STOP = "STOP"


class MsgCmdWEnum(str, Enum):
    CONTROL_COMMAND = "CONTROL_COMMAND"
    INSERT = "INSERT"


class MsgAckStatusEnum(str, Enum):
    OK = "ok"
    REJECTED = "rejected"
    ERROR = "error"


class AckCode(str, Enum):
    OK = "S-0000"
    CMD_BODY_INVALID = "E-4101"
    CMD_UNSUPPORTED_ACTION = "E-4102"


class ProtocolHeaderDTO(BaseModel):
    msg_id: UUID
    gateway_address: str
    collector_address: str
    msg_type: MsgTypeEnum
    msg_body: dict
    timestamp_ms: int


class MsgAckDTO(BaseModel):
    ref_msg_id: UUID
    device_key: str
    action: MsgCmdREnum | MsgCmdWEnum
    status: MsgAckStatusEnum
    code: AckCode = AckCode.OK
    reason: str | None = None
    applied_ms: int | None = None


class MsgHealthDTO(BaseModel):
    device_key: str
    ipc_ok: bool
    device_ok: bool
    reason: str | None = None
    observed_ms: int


class MsgSampleDTO(BaseModel):
    addr: str
    name: str
    type: PointType
    value: Value = None
    error: str | None = None


class MsgDataDTO(BaseModel):
    device_key: str
    sample_ms: int
    samples: list[MsgSampleDTO]
    sockets: list[dict] = Field(default_factory=list)

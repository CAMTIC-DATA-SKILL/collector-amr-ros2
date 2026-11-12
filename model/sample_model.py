"""DATA 샘플 한 건. 조회 키는 addr 이다."""
from enum import Enum

from pydantic import BaseModel, ConfigDict


class PointType(str, Enum):
    BOOL = "bool"
    I64 = "i64"
    F64 = "f64"
    STR = "str"


Value = bool | int | float | str | None


class Sample(BaseModel):
    """실패하면 value 는 None 이고 error 에 코드가 들어간다. 이 collector 는 정상 값만 싣는다."""

    model_config = ConfigDict(frozen=True)

    addr: str
    name: str
    type: PointType
    value: Value = None
    error: str | None = None

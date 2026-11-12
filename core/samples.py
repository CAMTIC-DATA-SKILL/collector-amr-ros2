"""ROS 메시지에서 스칼라 샘플을 고른다. rclpy 에 의존하지 않는다."""
from __future__ import annotations

import math
import numbers
import re
from dataclasses import dataclass

from model.sample_model import PointType, Sample

# 라이다·공분산·포인트클라우드는 미들웨어가 배열로 받지 않는다.
_SKIP_FIELDS = frozenset({"header", "covariance", "ranges", "intensities", "points", "data"})
_MAX_EXPAND = 8
_MAX_STR = 200
_ADDR_UNSAFE = re.compile(r"[^A-Za-z0-9_]+")

WAIT_REASON = "E-1003"


@dataclass(frozen=True)
class Snapshot:
    mono: float
    samples: tuple[Sample, ...]


def clean_addr(name: str) -> str:
    return _ADDR_UNSAFE.sub("_", name).strip("_")


def _public_name(slot: str) -> str:
    if slot.startswith("_") and not slot.startswith("__"):
        return slot[1:]
    return slot


def _is_message(value: object) -> bool:
    slots = getattr(value, "__slots__", None)
    return isinstance(slots, tuple) and bool(slots) and not isinstance(value, type)


def _is_sequence(value: object) -> bool:
    if isinstance(value, (str, bytes, bytearray, dict)) or _is_message(value):
        return False
    if isinstance(value, (list, tuple)):
        return True
    return type(value).__name__ in {"array", "ndarray"}


def _scalar(value: object) -> tuple[PointType, bool | int | float | str] | None:
    if isinstance(value, bool):
        return PointType.BOOL, value
    if isinstance(value, str):
        if not value or len(value) > _MAX_STR:
            return None
        return PointType.STR, value
    if isinstance(value, numbers.Integral):
        return PointType.I64, int(value)
    if isinstance(value, numbers.Real):
        number = float(value)
        if not math.isfinite(number):
            return None
        return PointType.F64, number
    return None


def _point(addr: str, kind: PointType, value: bool | int | float | str) -> Sample | None:
    addr = clean_addr(addr)
    if not addr:
        return None
    return Sample(addr=addr, name=addr, type=kind, value=value, error=None)


def flatten_scalars(msg: object, prefix: str = "", depth: int = 0) -> list[Sample]:
    """메시지 슬롯의 스칼라만 샘플로 만든다. 길이 8 을 넘는 배열은 버린다."""
    if depth > 1:
        return []
    slots = getattr(msg, "__slots__", None)
    if not isinstance(slots, tuple):
        return []
    samples: list[Sample] = []
    for slot in slots:
        name = _public_name(str(slot))
        if not name or name in _SKIP_FIELDS or name.startswith("_"):
            continue
        value = getattr(msg, slot)
        scalar = _scalar(value)
        if scalar is not None:
            point = _point(f"{prefix}{name}", *scalar)
            if point is not None:
                samples.append(point)
            continue
        if _is_sequence(value):
            if len(value) == 0 or len(value) > _MAX_EXPAND:  # type: ignore[arg-type]
                continue
            for index, item in enumerate(value):  # type: ignore[arg-type]
                item_scalar = _scalar(item)
                if item_scalar is not None:
                    point = _point(f"{prefix}{name}_{index}", *item_scalar)
                    if point is not None:
                        samples.append(point)
                    continue
                if _is_message(item):
                    samples.extend(flatten_scalars(item, f"{prefix}{name}_{index}_", depth + 1))
            continue
        if _is_message(value):
            samples.extend(flatten_scalars(value, f"{prefix}{name}_", depth + 1))
    return _dedupe(samples)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def _f64(addr: str, value: object) -> Sample | None:
    """Odometry 좌표는 float 이지만, 정수로 와도 실수 샘플로 보낸다."""
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        return None
    number = float(value)
    if not math.isfinite(number):
        return None
    return _point(addr, PointType.F64, number)


def _pose_message(msg: object) -> object | None:
    """Odometry / PoseWithCovarianceStamped / PoseStamped 에서 geometry Pose 를 꺼낸다."""
    pose = getattr(msg, "pose", None)
    if pose is None:
        return None
    inner = getattr(pose, "pose", None)
    return inner if inner is not None else pose


def samples_from_odometry(msg: object) -> list[Sample]:
    pose = _pose_message(msg)
    twist = getattr(getattr(msg, "twist", None), "twist", None)
    if pose is None or twist is None:
        return []
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    linear = getattr(twist, "linear", None)
    angular = getattr(twist, "angular", None)
    if None in (position, orientation, linear, angular):
        return []
    yaw = yaw_from_quaternion(
        float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)
    )
    points = [
        _f64("odom_x", position.x),
        _f64("odom_y", position.y),
        _f64("odom_yaw", yaw),
        _f64("odom_vx", linear.x),
        _f64("odom_wz", angular.z),
    ]
    if any(point is None for point in points):
        return []
    return _dedupe([point for point in points if point is not None])


def samples_from_pose(msg: object, prefix: str) -> list[Sample]:
    pose = _pose_message(msg)
    if pose is None:
        return []
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    if position is None or orientation is None:
        return []
    yaw = yaw_from_quaternion(
        float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)
    )
    points = [
        _f64(f"{prefix}_x", position.x),
        _f64(f"{prefix}_y", position.y),
        _f64(f"{prefix}_yaw", yaw),
    ]
    if any(point is None for point in points):
        return []
    return _dedupe([point for point in points if point is not None])


def _fresh(snapshot: Snapshot | None, now: float, stale_sec: float) -> bool:
    return snapshot is not None and (now - snapshot.mono) <= stale_sec


def _bit(addr: str, value: bool) -> Sample:
    return Sample(addr=addr, name=addr, type=PointType.BOOL, value=value, error=None)


def assemble_samples(
    *,
    now: float,
    stale_sec: float,
    odom: Snapshot | None,
    scout: Snapshot | None,
    amcl: Snapshot | None,
    scan_mono: float | None,
) -> tuple[list[Sample], bool, str | None]:
    """코어 토픽이 살아 있으면 스칼라 샘플을 만든다. 아니면 DATA 없이 E-1003."""
    odom_ok = _fresh(odom, now, stale_sec)
    scout_ok = _fresh(scout, now, stale_sec)
    if not (odom_ok or scout_ok):
        return [], False, WAIT_REASON
    amcl_ok = _fresh(amcl, now, stale_sec)
    scan_ok = scan_mono is not None and (now - scan_mono) <= stale_sec
    samples: list[Sample] = []
    if odom_ok and odom is not None:
        samples.extend(odom.samples)
    if scout_ok and scout is not None:
        samples.extend(scout.samples)
    if amcl_ok and amcl is not None:
        samples.extend(amcl.samples)
    samples.extend(
        [
            _bit("wheel_odom_ok", odom_ok),
            _bit("scout_status_ok", scout_ok),
            _bit("amcl_ok", amcl_ok),
            _bit("scan_ok", scan_ok),
        ]
    )
    return _dedupe(samples), True, None


def _dedupe(samples: list[Sample]) -> list[Sample]:
    """같은 addr 은 나중 값이 이긴다. 상태 비트는 측정값 뒤에 붙여 덮어쓴다."""
    order: list[str] = []
    by_addr: dict[str, Sample] = {}
    for sample in samples:
        if sample.addr not in by_addr:
            order.append(sample.addr)
        by_addr[sample.addr] = sample
    return [by_addr[addr] for addr in order]

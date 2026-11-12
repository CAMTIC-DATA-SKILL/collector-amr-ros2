"""camtic_dev 안의 ROS 2 토픽을 주기 샘플로 만든다.

구독 QoS 는 발행자와 같게 맞춘다. 읽지 못하면 BEST_EFFORT + VOLATILE 로 구독한다.
"""
from __future__ import annotations

import logging
import signal
import threading
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from config import settings
from core.samples import (
    Snapshot,
    assemble_samples,
    flatten_scalars,
    samples_from_odometry,
    samples_from_pose,
    samples_from_scan,
)

logger = logging.getLogger("collector")


def _qos_like(info: object) -> QoSProfile:
    """발행자와 같은 reliability/durability 로 구독한다. 다르면 토픽이 안 들어온다.

    속성을 읽지 못하면 센서용 BEST_EFFORT + VOLATILE 로 떨어진다.
    이 조합은 reliable 발행과도 호환된다.
    """
    fallback = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    offered = getattr(info, "qos_profile", None)
    if offered is None:
        return fallback
    try:
        return QoSProfile(
            reliability=offered.reliability,
            durability=offered.durability,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
    except Exception:
        logger.warning("publisher QoS 를 복사하지 못해 BEST_EFFORT 로 구독합니다")
        return fallback


@dataclass(frozen=True)
class _Spec:
    key: str
    topic: str


class AmrNode(Node):
    def __init__(self) -> None:
        super().__init__("collector_amr_ros2")
        self._specs = (
            _Spec("scout", settings.scout_status_topic),
            _Spec("odom", settings.odom_topic),
            _Spec("amcl", settings.amcl_pose_topic),
            _Spec("scan", settings.scan_topic),
        )
        self._subs: list[object] = []
        self._subscribed: set[str] = set()
        self._type_failed: set[str] = set()
        self._subscribe_failed: set[str] = set()
        self._empty_warned: set[str] = set()
        self._slots_logged: set[str] = set()
        self._odom: Snapshot | None = None
        self._scout: Snapshot | None = None
        self._amcl: Snapshot | None = None
        self._scan: Snapshot | None = None

    def refresh_subscriptions(self) -> None:
        for spec in self._specs:
            if (
                not spec.topic
                or spec.topic in self._subscribed
                or spec.topic in self._type_failed
                or spec.topic in self._subscribe_failed
            ):
                continue
            try:
                infos = self.get_publishers_info_by_topic(spec.topic)
            except Exception:
                logger.exception("publisher 조회 실패: %s", spec.topic)
                continue
            if not infos:
                continue
            type_name = infos[0].topic_type
            if not type_name:
                continue
            try:
                msg_type = get_message(type_name)
            except Exception as exc:
                self._type_failed.add(spec.topic)
                logger.error(
                    "%s 타입 %s 을 불러오지 못했습니다. "
                    "run.sh 가 찍은 ROS_DOMAIN_ID / RMW_IMPLEMENTATION 과 "
                    "대화형 셸의 값이 같은지 확인하세요. (%s)",
                    spec.topic,
                    type_name,
                    exc,
                )
                continue
            try:
                qos = _qos_like(infos[0])
                subscription = self.create_subscription(
                    msg_type,
                    spec.topic,
                    lambda msg, key=spec.key: self._on_msg(key, msg),
                    qos,
                )
            except Exception:
                logger.exception("구독 실패: %s", spec.topic)
                self._subscribe_failed.add(spec.topic)
                continue
            self._subs.append(subscription)
            self._subscribed.add(spec.topic)
            logger.info(
                "subscribe %s type=%s publishers=%d reliability=%s durability=%s",
                spec.topic,
                type_name,
                len(infos),
                getattr(qos.reliability, "name", qos.reliability),
                getattr(qos.durability, "name", qos.durability),
            )

    def _on_msg(self, key: str, msg: object) -> None:
        now = time.monotonic()
        try:
            if key == "scan":
                samples = samples_from_scan(msg)
                if samples:
                    self._scan = Snapshot(mono=now, samples=tuple(samples))
                return
            if key == "odom":
                samples = samples_from_odometry(msg)
            elif key == "amcl":
                samples = samples_from_pose(msg, "amcl")
            elif key == "scout":
                self._log_slots_once(msg)
                samples = flatten_scalars(msg)
            else:
                return
        except Exception:
            logger.exception("메시지 해석 실패: %s", key)
            return
        if not samples:
            if key not in self._empty_warned:
                self._empty_warned.add(key)
                logger.error(
                    "%s 메시지에서 값을 읽지 못했습니다. slots=%s",
                    key,
                    getattr(msg, "__slots__", None),
                )
            return
        snapshot = Snapshot(mono=now, samples=tuple(samples))
        if key == "odom":
            self._odom = snapshot
        elif key == "amcl":
            self._amcl = snapshot
        elif key == "scout":
            self._scout = snapshot

    def _log_slots_once(self, msg: object) -> None:
        if "scout" in self._slots_logged:
            return
        self._slots_logged.add("scout")
        slots = getattr(msg, "__slots__", ())
        logger.info("scout_status slots: %s", ", ".join(str(slot) for slot in slots))

    def assemble(self, now: float) -> tuple[list, bool, str | None]:
        return assemble_samples(
            now=now,
            stale_sec=settings.stale_sec,
            odom=self._odom,
            scout=self._scout,
            amcl=self._amcl,
            scan=self._scan,
        )

    def _mono_for(self, key: str) -> float | None:
        snapshot = {"odom": self._odom, "scout": self._scout, "amcl": self._amcl, "scan": self._scan}.get(key)
        return None if snapshot is None else snapshot.mono

    def wait_status(self, now: float) -> str:
        parts: list[str] = []
        for spec in self._specs:
            if not spec.topic:
                continue
            if spec.topic in self._type_failed:
                state = "type_error"
            elif spec.topic in self._subscribe_failed:
                state = "subscribe_error"
            elif spec.topic not in self._subscribed:
                state = "no_publisher"
            else:
                mono = self._mono_for(spec.key)
                state = "no_message" if mono is None else f"age_{now - mono:.1f}s"
            parts.append(f"{spec.topic}={state}")
        return ", ".join(parts)


def run_ros(stop: threading.Event, on_cycle) -> None:
    """on_cycle(samples, device_ok, reason, wait_status) 를 POLL_INTERVAL_SEC 마다 호출한다."""
    init_kwargs = {}
    try:
        from rclpy.signals import SignalHandlerOptions

        init_kwargs["signal_handler_options"] = SignalHandlerOptions.NO
    except Exception:
        logger.warning("SignalHandlerOptions 를 쓰지 않고 rclpy.init 한다")
    try:
        rclpy.init(**init_kwargs)
    except Exception:
        logger.exception("rclpy.init 실패")
        raise
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGHUP, lambda *_: stop.set())
    node = AmrNode()
    try:
        while not stop.is_set() and rclpy.ok():
            deadline = time.monotonic() + settings.poll_interval
            while not stop.is_set() and rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
                node.refresh_subscriptions()
            if stop.is_set() or not rclpy.ok():
                break
            now = time.monotonic()
            samples, ok, reason = node.assemble(now)
            on_cycle(samples, ok, reason, node.wait_status(now))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

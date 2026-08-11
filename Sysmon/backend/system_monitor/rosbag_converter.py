"""rosbag2 메시지를 System Monitor의 쥐몰이 Replay JSON으로 변환한다.

ROS 메시지를 읽는 부분과 시간별 프레임을 만드는 부분을 분리했다. 덕분에 ROS 2가
없는 개발 PC에서도 순수 변환 로직은 단위 테스트할 수 있고, 실제 bag을 읽을 때만
``rosbag2_py``와 ``rclpy``가 필요하다.
"""

from __future__ import annotations

import copy
import json
import math
from dataclasses import asdict, dataclass
from itertools import groupby
from pathlib import Path
from typing import Any, Iterable


class ConversionError(RuntimeError):
    """입력 bag이나 필수 좌표가 부족할 때 사용자에게 보여줄 오류다."""


@dataclass(frozen=True)
class SampleEvent:
    """bag의 비동기 ROS 메시지 한 건을 공통 필드 변경으로 단순화한 값."""

    timestamp: float
    field: str
    value: Any
    frame_id: str | None = None


@dataclass(frozen=True)
class TopicConfig:
    """main 토픽 이름을 한곳에서 바꿀 수 있게 모은 설정."""

    driver_id: str = "robot4"
    blocker_id: str = "robot6"
    driver_odom: str = "/robot4/odom"
    blocker_odom: str = "/robot6/odom"
    target_event: str | None = "/fleet/event"
    target_pose: str | None = None
    driver_goal: str | None = "/robot4/target_pose"
    blocker_goal: str | None = "/robot6/target_pose"
    fleet_status: str | None = "/fleet/status"
    state: str | None = "/herding/state"
    capture_progress: str | None = "/herding/capture_progress"
    success: str | None = "/herding/success"

    def normalized(self) -> "TopicConfig":
        """선택 토픽의 빈 문자열을 None으로, 토픽 이름은 절대 이름으로 만든다."""

        values = asdict(self)
        for name, value in values.items():
            if name in {"driver_id", "blocker_id"}:
                values[name] = str(value).strip("/")
            elif value:
                values[name] = f"/{str(value).strip('/')}"
            else:
                values[name] = None
        return TopicConfig(**values)

    def relevant_topics(self) -> set[str]:
        config = self.normalized()
        return {
            value
            for name, value in asdict(config).items()
            if name not in {"driver_id", "blocker_id"} and value
        }


_POINT_FIELDS = {"target", "driver", "blocker", "driver_goal", "blocker_goal"}
_FSM_STATE_MAP = {
    "IDLE": "IDLE",
    "SEARCH": "SEARCH",
    "SEARCHING": "SEARCH",
    "PATROLLING": "SEARCH",
    "TRACK": "TRACK",
    "TRACKING": "TRACK",
    "HERD": "HERD",
    "HERDING": "HERD",
    "NAVIGATING": "HERD",
    "CORNER": "CORNER",
    "LOST": "LOST",
    "TARGET_LOST": "LOST",
    "CAPTURED": "CAPTURED",
}


def _point(value: Any, field: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        raise ConversionError(f"{field} 좌표는 [x, y] 형식이어야 합니다.")
    x, y = float(value[0]), float(value[1])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ConversionError(f"{field} 좌표에 유효하지 않은 숫자가 있습니다.")
    return [x, y]


def _fsm_state(value: Any) -> str:
    raw = str(value or "SEARCH").strip().upper()
    return _FSM_STATE_MAP.get(raw, raw if raw else "SEARCH")


def _message_frame_id(message: Any) -> str | None:
    frame_id = getattr(getattr(message, "header", None), "frame_id", None)
    return str(frame_id).strip("/") if frame_id else None


def _pose_xy(message: Any, *, odometry: bool) -> list[float]:
    pose = message.pose.pose if odometry else message.pose
    return [float(pose.position.x), float(pose.position.y)]


def events_from_message(
    topic: str,
    message: Any,
    timestamp: float,
    topics: TopicConfig,
) -> list[SampleEvent]:
    """한 ROS 메시지를 0개 이상의 공통 SampleEvent로 바꾼다.

    현재 main의 colon 구분 String 계약과, 향후 추가할 수 있는 전용 상태·진행률
    토픽을 모두 지원한다.
    """

    topics = topics.normalized()
    topic = f"/{topic.strip('/')}"
    frame_id = _message_frame_id(message)
    if topic == topics.driver_odom:
        return [SampleEvent(timestamp, "driver", _pose_xy(message, odometry=True), frame_id)]
    if topic == topics.blocker_odom:
        return [SampleEvent(timestamp, "blocker", _pose_xy(message, odometry=True), frame_id)]
    if topics.target_pose and topic == topics.target_pose:
        return [SampleEvent(timestamp, "target", _pose_xy(message, odometry=False), frame_id)]
    if topics.driver_goal and topic == topics.driver_goal:
        return [SampleEvent(timestamp, "driver_goal", _pose_xy(message, odometry=False), frame_id)]
    if topics.blocker_goal and topic == topics.blocker_goal:
        return [SampleEvent(timestamp, "blocker_goal", _pose_xy(message, odometry=False), frame_id)]

    if topics.target_event and topic == topics.target_event:
        parts = str(getattr(message, "data", "")).split(":")
        if len(parts) != 3:
            return []
        name = parts[0].strip().lower()
        try:
            point = [float(parts[1]), float(parts[2])]
        except ValueError:
            return []
        if name == "rat_detected":
            return [SampleEvent(timestamp, "target", point, "map")]
        if name in {"rat_captured", "captured"}:
            return [
                SampleEvent(timestamp, "target", point, "map"),
                SampleEvent(timestamp, "state", "CAPTURED"),
                SampleEvent(timestamp, "capture_progress", 1.0),
                SampleEvent(timestamp, "success", True),
            ]
        return []

    if topics.fleet_status and topic == topics.fleet_status:
        parts = str(getattr(message, "data", "")).split(":")
        if len(parts) >= 2 and parts[0].strip("/") == topics.driver_id:
            return [SampleEvent(timestamp, "state", _fsm_state(parts[1]))]
        return []
    if topics.state and topic == topics.state:
        return [SampleEvent(timestamp, "state", _fsm_state(getattr(message, "data", "")))]
    if topics.capture_progress and topic == topics.capture_progress:
        return [SampleEvent(timestamp, "capture_progress", float(message.data))]
    if topics.success and topic == topics.success:
        success = bool(message.data)
        events = [SampleEvent(timestamp, "success", success)]
        if success:
            events.extend(
                [
                    SampleEvent(timestamp, "state", "CAPTURED"),
                    SampleEvent(timestamp, "capture_progress", 1.0),
                ]
            )
        return events
    return []


def build_trial(
    events: Iterable[SampleEvent],
    *,
    sample_period: float = 0.1,
    model: str = "rosbag_recording",
    goal_name: str | None = None,
    seed: int | None = None,
) -> dict[str, Any]:
    """시간순 이벤트를 일정 간격의 Replay 프레임 한 시험으로 만든다."""

    if sample_period <= 0:
        raise ConversionError("sample_period는 0보다 커야 합니다.")
    ordered = sorted(events, key=lambda item: item.timestamp)
    if not ordered:
        raise ConversionError("변환할 쥐몰이 메시지가 없습니다.")

    current: dict[str, Any] = {
        "state": "SEARCH",
        "capture_progress": 0.0,
        "success": False,
    }
    frames: list[dict[str, Any]] = []
    start_time: float | None = None
    next_sample: float | None = None
    last_event_time = ordered[-1].timestamp
    epsilon = sample_period / 1000

    def ready() -> bool:
        return all(name in current for name in ("target", "driver", "blocker"))

    def snapshot(absolute_time: float) -> dict[str, Any]:
        target = _point(current["target"], "target")
        driver = _point(current["driver"], "driver")
        blocker = _point(current["blocker"], "blocker")
        driver_goal = _point(current.get("driver_goal", driver), "driver_goal")
        blocker_goal = _point(current.get("blocker_goal", blocker), "blocker_goal")
        state = _fsm_state(current.get("state"))
        progress = max(0.0, min(1.0, float(current.get("capture_progress", 0.0))))
        if state == "CAPTURED" or current.get("success"):
            progress = 1.0
        return {
            "t": round(absolute_time - float(start_time), 3),
            "target": target,
            "driver": driver,
            "blocker": blocker,
            "driver_goal": driver_goal,
            "blocker_goal": blocker_goal,
            "state": state,
            "discovered": state in {"TRACK", "HERD", "CORNER", "LOST", "CAPTURED"},
            "panic": False,
            "driver_panic": False,
            "dist": round(math.dist(target, driver), 3),
            "capture_progress": round(progress, 4),
        }

    for timestamp, grouped_events in groupby(ordered, key=lambda item: item.timestamp):
        if start_time is not None:
            while next_sample is not None and next_sample < timestamp - epsilon:
                frames.append(snapshot(next_sample))
                next_sample += sample_period

        for event in grouped_events:
            if event.field in _POINT_FIELDS:
                current[event.field] = _point(event.value, event.field)
            elif event.field == "state":
                current[event.field] = _fsm_state(event.value)
                if current[event.field] == "CAPTURED":
                    current["success"] = True
            elif event.field == "capture_progress":
                current[event.field] = float(event.value)
            elif event.field == "success":
                current[event.field] = bool(event.value)

        if start_time is None and ready():
            start_time = timestamp
            next_sample = timestamp
        while (
            start_time is not None
            and next_sample is not None
            and next_sample <= timestamp + epsilon
        ):
            frames.append(snapshot(next_sample))
            next_sample += sample_period

    if start_time is None:
        missing = [name for name in ("target", "driver", "blocker") if name not in current]
        raise ConversionError(
            "프레임 생성에 필요한 좌표가 없습니다: " + ", ".join(missing)
        )
    if not frames or frames[-1]["t"] < last_event_time - start_time - epsilon:
        frames.append(snapshot(last_event_time))

    discovered_frames = [frame for frame in frames if frame["discovered"]]
    success = bool(current.get("success")) or any(
        frame["state"] == "CAPTURED" for frame in frames
    )
    blocker_distances = [
        math.dist(frame["target"], frame["blocker"]) for frame in discovered_frames
    ]
    return {
        "model": model,
        "seed": seed,
        "success": success,
        "goal_name": goal_name,
        "mouse_spawn": frames[0]["target"],
        "discovery_time": discovered_frames[0]["t"] if discovered_frames else None,
        "duration": frames[-1]["t"],
        "frames": frames,
        "min_blocker_dist_after_discovery": (
            round(min(blocker_distances), 3) if blocker_distances else None
        ),
        "discovered": bool(discovered_frames),
    }


def build_replay_document(
    trial: dict[str, Any],
    *,
    base_document: dict[str, Any] | None = None,
    append_existing_trials: bool = False,
) -> dict[str, Any]:
    """새 시험과 선택적인 기존 지도 정보를 합쳐 최종 Replay JSON을 만든다."""

    base = copy.deepcopy(base_document or {})
    layout_keys = (
        "photo_frame",
        "map_image",
        "traps",
        "capture_radius",
        "panic_distance",
        "sensor_range",
    )
    document = {key: base[key] for key in layout_keys if key in base}
    existing = copy.deepcopy(base.get("trials") or []) if append_existing_trials else []
    document["trials"] = [*existing, copy.deepcopy(trial)]
    return document


def _storage_identifier(bag_path: Path) -> str:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.exists():
        return "sqlite3"
    try:
        import yaml

        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
        info = metadata.get("rosbag2_bagfile_information") or {}
        return str(info.get("storage_identifier") or "sqlite3")
    except (OSError, ValueError):
        return "sqlite3"


def read_rosbag_events(
    bag_path: Path,
    topics: TopicConfig,
    *,
    map_frame: str = "map",
) -> tuple[list[SampleEvent], list[str], dict[str, str]]:
    """rosbag2 디렉터리를 열어 변환에 필요한 토픽만 역직렬화한다."""

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ModuleNotFoundError as error:
        raise ConversionError(
            "ROS 2 Python 모듈을 찾을 수 없습니다. 먼저 "
            "'source /opt/ros/humble/setup.bash'와 프로젝트 install/setup.bash를 실행하세요."
        ) from error

    bag_path = Path(bag_path)
    if not bag_path.is_dir():
        raise ConversionError("rosbag 경로는 metadata.yaml이 있는 디렉터리여야 합니다.")

    topics = topics.normalized()
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(
                uri=str(bag_path), storage_id=_storage_identifier(bag_path)
            ),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )
    except Exception as error:
        raise ConversionError(f"rosbag을 열 수 없습니다: {bag_path} ({error})") from error
    type_by_topic = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    required = {topics.driver_odom, topics.blocker_odom}
    target_candidates = {value for value in (topics.target_event, topics.target_pose) if value}
    if not target_candidates:
        raise ConversionError("--target-event 또는 --target-pose 중 하나는 필요합니다.")
    missing = sorted(topic for topic in required if topic not in type_by_topic)
    if missing:
        raise ConversionError("rosbag에 필수 토픽이 없습니다: " + ", ".join(missing))
    if not target_candidates.intersection(type_by_topic):
        raise ConversionError(
            "rosbag에 쥐 위치 토픽이 없습니다: " + ", ".join(sorted(target_candidates))
        )

    relevant = topics.relevant_topics().intersection(type_by_topic)
    try:
        message_classes = {
            topic: get_message(type_by_topic[topic]) for topic in relevant
        }
    except (AttributeError, ModuleNotFoundError, ValueError) as error:
        raise ConversionError(
            "bag 메시지 타입을 불러올 수 없습니다. 해당 ROS 메시지 패키지를 source했는지 확인하세요."
        ) from error
    events: list[SampleEvent] = []
    source_frames: set[str] = set()
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic not in relevant:
            continue
        try:
            message = deserialize_message(serialized, message_classes[topic])
        except Exception as error:
            raise ConversionError(
                f"ROS 메시지를 읽지 못했습니다: {topic} ({type_by_topic[topic]})"
            ) from error
        decoded = events_from_message(topic, message, timestamp_ns / 1_000_000_000, topics)
        events.extend(decoded)
        source_frames.update(event.frame_id for event in decoded if event.frame_id)

    warnings: list[str] = []
    mismatched = sorted(frame for frame in source_frames if frame != map_frame.strip("/"))
    if mismatched:
        warnings.append(
            "map 좌표계와 다른 frame이 포함돼 있습니다: " + ", ".join(mismatched)
        )
    present_fields = {event.field for event in events}
    if "driver_goal" not in present_fields:
        warnings.append("Driver 목표 토픽이 없어 실제 Driver 위치를 목표값으로 사용합니다.")
    if "blocker_goal" not in present_fields:
        warnings.append("Blocker 목표 토픽이 없어 실제 Blocker 위치를 목표값으로 사용합니다.")
    if "capture_progress" not in present_fields:
        warnings.append("포획 진행률 토픽이 없어 기본값 0%를 사용합니다.")
    if "state" not in present_fields:
        warnings.append("FSM 상태 토픽이 없어 기본 상태 SEARCH를 사용합니다.")
    return events, warnings, type_by_topic

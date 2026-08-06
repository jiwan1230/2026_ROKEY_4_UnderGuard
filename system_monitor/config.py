"""환경 변수를 검증된 애플리케이션 설정 객체로 변환한다."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path


def _csv(name: str, default: str) -> list[str]:
    value = os.getenv(name, default)
    return [item.strip() for item in value.split(",") if item.strip()]


@dataclass(frozen=True)
class RobotConfig:
    """로봇 한 대의 namespace, 역할 및 ROS 토픽 생성 규칙."""

    namespace: str
    role: str

    @property
    def robot_id(self) -> str:
        return self.namespace.strip("/")

    def topic(self, name: str) -> str:
        """상대 이름, 절대 이름 또는 namespace 템플릿을 ROS 토픽으로 만든다.

        예: ``odom`` → ``/robot4/odom``, ``/fleet/map`` → 그대로 유지,
        ``/{namespace}/odom_filtered`` → 로봇 namespace를 치환한다.
        """

        is_template = "{namespace}" in name or "{robot_id}" in name
        rendered = name.format(namespace=self.robot_id, robot_id=self.robot_id)
        if not rendered.strip("/"):
            raise ValueError("ROS 토픽 이름은 비어 있을 수 없습니다.")
        if rendered.startswith("/") or is_template:
            return f"/{rendered.lstrip('/')}"
        return f"/{self.robot_id}/{rendered.lstrip('/')}"


@dataclass(frozen=True)
class RosInterfaceConfig:
    """main Fleet 계약과 선택적 센서 입력 경계를 한곳에 모은다."""

    fleet_status_topic: str = "/fleet/status"
    fleet_event_topic: str = "/fleet/event"
    fleet_command_topic: str = "/fleet/command"
    webcam_detections_topic: str = "webcam/detections"
    oakd_detections_topic: str = "oakd/detections"
    odometry_topic: str = "odom"
    battery_topic: str = "battery_state"
    map_frame: str = "map"


@dataclass(frozen=True)
class Settings:
    """서버, 인증, 저장소, 실행 모드를 묶은 런타임 설정."""

    mode: str
    secret_key: str
    database_path: Path
    robots: tuple[RobotConfig, ...]
    offline_timeout_sec: float
    poll_interval_ms: int
    target_loss_timeout_sec: float = 1.5
    low_battery_threshold: float = 15.0
    map_yaml_path: Path = Path("../minipjt/mini_turtle4/resource/my_map.yaml")
    ros_interface: RosInterfaceConfig = field(default_factory=RosInterfaceConfig)


def load_settings() -> Settings:
    """환경 변수에서 실행 설정을 읽는다.

    입력: ``MONITOR_MODE``, ``ROBOT_NAMESPACES`` 등 환경 변수다.
    출력: 두 로봇 설정과 서버/DB 정보를 담은 ``Settings`` 객체다.
    사용: 일반 실행에서는 ``create_app()``이 자동으로 호출한다.
    """

    namespaces = _csv("ROBOT_NAMESPACES", "robot4,robot6")
    roles = _csv("ROBOT_ROLES", ",".join("SCOUT" for _ in namespaces))
    if len(roles) < len(namespaces):
        roles.extend(["UNASSIGNED"] * (len(namespaces) - len(roles)))

    robots = tuple(
        RobotConfig(namespace=namespace, role=roles[index])
        for index, namespace in enumerate(namespaces)
    )
    if not robots:
        raise ValueError("ROBOT_NAMESPACES에는 최소 한 대가 필요합니다.")

    return Settings(
        mode=os.getenv("MONITOR_MODE", "mock").strip().lower(),
        secret_key=os.getenv("SECRET_KEY", "dev-secret-change-me"),
        database_path=Path(os.getenv("DATABASE_PATH", "system_monitor.db")),
        robots=robots,
        # main robot_agent의 fleet status 기본 주기(10초)보다 길어야 정상 로봇이
        # 다음 상태 보고 전에 Offline으로 깜빡이지 않는다.
        offline_timeout_sec=float(os.getenv("OFFLINE_TIMEOUT_SEC", "15.0")),
        poll_interval_ms=int(os.getenv("POLL_INTERVAL_MS", "1000")),
        target_loss_timeout_sec=float(os.getenv("TARGET_LOSS_TIMEOUT_SEC", "1.5")),
        low_battery_threshold=float(os.getenv("LOW_BATTERY_THRESHOLD", "15.0")),
        map_yaml_path=Path(
            os.getenv(
                "MAP_YAML_PATH",
                "../minipjt/mini_turtle4/resource/my_map.yaml",
            )
        ),
        ros_interface=RosInterfaceConfig(
            fleet_status_topic=os.getenv("ROS_FLEET_STATUS_TOPIC", "/fleet/status"),
            fleet_event_topic=os.getenv("ROS_FLEET_EVENT_TOPIC", "/fleet/event"),
            fleet_command_topic=os.getenv("ROS_FLEET_COMMAND_TOPIC", "/fleet/command"),
            webcam_detections_topic=os.getenv(
                "ROS_WEBCAM_DETECTIONS_TOPIC", "webcam/detections"
            ),
            oakd_detections_topic=os.getenv(
                "ROS_OAKD_DETECTIONS_TOPIC", "oakd/detections"
            ),
            odometry_topic=os.getenv("ROS_ODOMETRY_TOPIC", "odom"),
            battery_topic=os.getenv("ROS_BATTERY_TOPIC", "battery_state"),
            map_frame=os.getenv("ROS_MAP_FRAME", "map").strip("/") or "map",
        ),
    )

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
    # /fleet/event(name:x:y)에는 없는 robot_id·confidence가 실려 오는 상세 탐지
    # 토픽이다. 이게 붙으면 "최근 활동 로봇으로 추측"하지 않고 실제 값을 쓴다.
    fleet_detection_topic: str = "/fleet/detection"
    # db_node가 여는 기록 조회 서비스(읽기 전용). UI는 MySQL에 직접 붙지 않는다.
    db_query_service: str = "/db/query"
    webcam_detections_topic: str = "webcam/detections"
    oakd_detections_topic: str = "oakd/detections"
    odometry_topic: str = "odom"
    battery_topic: str = "battery_state"
    # OAK-D 원본 압축 RGB 토픽(로봇 namespace 하위). 예전에는 camera_node가
    # synced/rgb로 재발행했지만 detector_node에 통합·삭제되어 그 토픽은 더 이상
    # 발행되지 않는다 — 원본을 직접 구독한다.
    camera_frame_topic: str = "oakd/rgb/image_raw/compressed"
    map_frame: str = "map"


@dataclass(frozen=True)
class Settings:
    """실시간 관제 서버, 로봇, 맵과 ROS 실행 설정."""

    mode: str
    robots: tuple[RobotConfig, ...]
    offline_timeout_sec: float
    poll_interval_ms: int
    target_loss_timeout_sec: float = 1.5
    low_battery_threshold: float = 15.0
    # app.py가 Path.cwd() 기준으로 상대경로를 푸므로(run_mock.sh/run_ros.sh가
    # cd하는 Sysmon/backend/ 기준), Desktop/minipjt까지 3단계 위로 올라간다.
    # room_map.yaml은 main의 실제 로봇 실행 설정(robot_bringup.launch.py)과
    # 쥐몰이 알고리즘이 참조하는 현재 맵이다(2026-08-06 실측). my_map.yaml은
    # 이전 mini 프로젝트 때 쓰던 좌표계가 다른 옛 맵이라 대체했다.
    map_yaml_path: Path = Path("../../../minipjt/mini_turtle4/resource/room_map.yaml")
    ros_interface: RosInterfaceConfig = field(default_factory=RosInterfaceConfig)
    # MONITOR_MODE=replay 전용 — herding_controller_dual 검증 시뮬레이션이 남긴
    # 궤적(replay_manager.DEFAULT_FRAMES_PATH) 중 몇 번째 trial을 얼마나 빠르게
    # 재생할지다. 파일 경로 자체는 patch 우려 없이 패키지에 번들돼 있어 기본값이면
    # 충분하고, 필요하면 REPLAY_FRAMES_PATH로 다른 파일을 가리킬 수 있다.
    replay_frames_path: Path | None = None
    replay_trial_index: int = 0
    replay_speed: float = 1.0
    # 기록 조회 탭 전용 — 실시간 뷰(StateManager)와 분리된 영구 저장소다.
    # 상대경로는 Path.cwd() 기준(run_*.sh가 cd하는 Sysmon/backend/)이라
    # 기본값으로 두면 backend/data/ 아래에 생긴다. *.db는 .gitignore 처리됨.
    history_db_path: Path = Path("data/history.db")
    history_image_dir: Path = Path("data/captures")


def load_settings() -> Settings:
    """환경 변수에서 실행 설정을 읽는다.

    입력: ``MONITOR_MODE``, ``ROBOT_NAMESPACES`` 등 환경 변수다.
    출력: 두 로봇 설정과 서버/ROS 정보를 담은 ``Settings`` 객체다.
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
                "../../../minipjt/mini_turtle4/resource/room_map.yaml",
            )
        ),
        replay_frames_path=(
            Path(os.environ["REPLAY_FRAMES_PATH"])
            if os.getenv("REPLAY_FRAMES_PATH")
            else None
        ),
        replay_trial_index=int(os.getenv("REPLAY_TRIAL", "0")),
        replay_speed=float(os.getenv("REPLAY_SPEED", "1.0")),
        history_db_path=Path(os.getenv("HISTORY_DB_PATH", "data/history.db")),
        history_image_dir=Path(os.getenv("HISTORY_IMAGE_DIR", "data/captures")),
        ros_interface=RosInterfaceConfig(
            fleet_status_topic=os.getenv("ROS_FLEET_STATUS_TOPIC", "/fleet/status"),
            fleet_event_topic=os.getenv("ROS_FLEET_EVENT_TOPIC", "/fleet/event"),
            fleet_detection_topic=os.getenv(
                "ROS_FLEET_DETECTION_TOPIC", "/fleet/detection"
            ),
            db_query_service=os.getenv("ROS_DB_QUERY_SERVICE", "/db/query"),
            webcam_detections_topic=os.getenv(
                "ROS_WEBCAM_DETECTIONS_TOPIC", "webcam/detections"
            ),
            oakd_detections_topic=os.getenv(
                "ROS_OAKD_DETECTIONS_TOPIC", "oakd/detections"
            ),
            odometry_topic=os.getenv("ROS_ODOMETRY_TOPIC", "odom"),
            battery_topic=os.getenv("ROS_BATTERY_TOPIC", "battery_state"),
            camera_frame_topic=os.getenv(
                "ROS_CAMERA_FRAME_TOPIC", "oakd/rgb/image_raw/compressed"
            ),
            map_frame=os.getenv("ROS_MAP_FRAME", "map").strip("/") or "map",
        ),
    )

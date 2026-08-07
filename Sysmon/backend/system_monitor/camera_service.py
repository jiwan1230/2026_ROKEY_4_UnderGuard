"""로봇별 최신 카메라 프레임을 메모리에 잠시 보관한다.

StateManager와 의도적으로 분리했다: snapshot()은 asdict()/deepcopy()로
로봇 상태를 매 폴링(1Hz)마다 JSON 직렬화하는데, 여기에 원본 이미지 바이트가
섞이면 그때마다 무겁게 복사되고 애초에 bytes는 JSON으로 나갈 수도 없다.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass


@dataclass
class CameraFrame:
    content: bytes
    format: str
    received_at: float


class CameraFrameStore:
    """실행 중에만 유지되는 로봇별 최신 프레임 캐시.

    영구 저장하지 않으며 서버를 재시작하면 사라진다 — System Monitor의
    read-only/무상태 원칙과 동일하다.
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._frames: dict[str, CameraFrame] = {}

    def update(self, robot_id: str, content: bytes, format_: str) -> None:
        """최신 프레임으로 교체한다(재인코딩 없이 원본 바이트 그대로).

        입력: 로봇 ID, 압축된 이미지 바이트, 포맷("jpeg"/"png")이다.
        출력: 없음. 사용: ROS 카메라 토픽 콜백에서 매 프레임 호출한다.
        """

        with self._lock:
            self._frames[robot_id] = CameraFrame(
                content=content, format=format_, received_at=time.time()
            )

    def get(self, robot_id: str) -> CameraFrame | None:
        """해당 로봇의 최신 프레임을 반환한다. 없으면 None이다."""

        with self._lock:
            return self._frames.get(robot_id)

    def image_url_for(self, robot_id: str) -> str | None:
        """탐지 사건에 붙일 증거 이미지 링크다.

        입력: 로봇 ID다. 출력: 캐시된 프레임이 있으면 그 프레임을 내려주는
        API 경로, 없으면 None이다. 탐지 시점과 정확히 동기화된 프레임이
        아니라 "그 로봇의 가장 최근 프레임"을 가리키는 최소 구현이다.
        """

        with self._lock:
            return f"/api/camera/{robot_id}/frame" if robot_id in self._frames else None

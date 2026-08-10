'''
로봇 상태 데이터와 카메라 이미지 데이터의 성격이 다르기 때문에 StateManager와 따로 관리...

StateManager는 로봇의 배터리, 위치, 상태 같은 비교적 가벼운 정보를 주기적으로 JSON으로 만들어 웹에 전달.

첫째, 카메라 이미지는 용량이 커서 snapshot()을 만들 때마다 deepcopy()로 이미지까지 계속 복사하면 불필요하게 무거워짐.
둘째, 이미지의 bytes 데이터는 그대로는 JSON으로 변환할 수도 없습니다.
'''

from __future__ import annotations

import threading
import time
from dataclasses import dataclass


# 중요 #
# 이미지 데이터, 이미지 형식, 수신 시간에 대한 정보가 함께 필요하기 때문에 하나의 클래스 객체로 묶음
# @dataclass는 이런 데이터 저장용 클래스를 간단하게 정의할 수 있게 해줌 (생성자 따로작성x)
@dataclass
class CameraFrame:
    content: bytes
    format: str
    received_at: float


class CameraFrameStore:
    '''각 로봇의 최신 카메라 프레임을 메모리에 저장하고, Flask와 웹 화면에서 안전하게 접근할 수 있도록 관리'''
    ## 최신 프레임만 웹에 빠르게 보여주면 되기 때문에
    ## 이렇게 하면 디스크 입출력이 없어서 빠르고 구현도 단순

    def __init__(self) -> None:
        # 중요 #
        # 여러 스레드에서 카메라 프레임에 동시 접근을 보호
        self._lock = threading.RLock()
        # # robot_id를 key로 해서 각 로봇의 최신 카메라 프레임 하나씩 저장
        self._frames: dict[str, CameraFrame] = {}

    # 중요 #
    def update(self, robot_id: str, content: bytes, format_: str) -> None:
        '''# 같은 로봇 ID로 새 이미지가 들어오면, 이전 이미지를 새 이미지로 덮어씀'''
        with self._lock:
            self._frames[robot_id] = CameraFrame(
                content=content, format=format_, received_at=time.time()
            )

    # 중요 #
    def get(self, robot_id: str) -> CameraFrame | None:
        """특정 로봇의 최신 카메라 프레임을 가져옴"""

        with self._lock:
            return self._frames.get(robot_id)

    # 중요 #
    def image_url_for(self, robot_id: str) -> str | None:
        '''해당 로봇의 카메라 프레임이 저장되어 있으면, 웹에서 그 이미지를 요청할 수 있는 API 주소를 만들어 반환'''

        with self._lock:
            # 해당 robot_id의 카메라 프레임이 현재 저장되어 있는지 확인
            # 프레임이 존재하면 해당 이미지를 가져올 수 있는 Flask API 주소를 만들어서 반환
            return f"/api/camera/{robot_id}/frame" if robot_id in self._frames else None

'''
StateManager
→ 위치, 배터리, 상태 등
→ 가벼운 JSON 데이터 관리

CameraFrameStore
→ 실제 카메라 이미지 bytes
→ 최신 프레임만 메모리에 별도 관리
'''
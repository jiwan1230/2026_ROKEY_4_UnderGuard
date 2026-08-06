"""Mock과 ROS 실행기가 따라야 하는 최소 애플리케이션 계약.

주의: 이 파일 자체는 로직을 담고 있지 않은 ``Protocol``(인터페이스) 정의일
뿐이다. "현재 모드가 Mock인지 ROS인지 고르는 로직"은 여기가 아니라
``app.py``의 ``create_app()``에 있다
(``runtime = mock if settings.mode == "mock" else ros``).
이 파일은 그렇게 골라진 ``mock`` 또는 ``ros`` 객체가 반드시 갖춰야 할
메서드 목록만 강제해, 라우트 코드가 "지금 Mock이면 이렇게, ROS면
저렇게" 식으로 분기하지 않고 ``runtime.status()``처럼 동일하게
호출할 수 있게 해준다. 실제 구현은 ``MockManager``와 ``RosBridge``에
각각 들어 있다.
"""

from __future__ import annotations

from typing import Any, Protocol


class RuntimeService(Protocol):
    """Flask 라우트가 실행 모드를 분기하지 않도록 하는 공통 인터페이스.

    ``MockManager``, ``RosBridge`` 모두 아래 5개 멤버를 구현해야 하며,
    구현 클래스가 이 클래스를 상속할 필요는 없다(구조적 타이핑).
    """

    @property
    def available(self) -> bool: ...

    @property
    def running(self) -> bool: ...

    def status(self) -> dict[str, Any]: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def send_command(self, robot_id: str, command: str) -> dict[str, Any]: ...

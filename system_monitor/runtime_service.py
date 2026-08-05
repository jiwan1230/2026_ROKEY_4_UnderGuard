"""Mock과 ROS 실행기가 따라야 하는 최소 애플리케이션 계약."""

from __future__ import annotations

from typing import Any, Protocol


class RuntimeService(Protocol):
    """Flask 라우트가 실행 모드를 분기하지 않도록 하는 공통 인터페이스."""

    @property
    def available(self) -> bool: ...

    @property
    def running(self) -> bool: ...

    def status(self) -> dict[str, Any]: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def send_command(self, robot_id: str, command: str) -> dict[str, Any]: ...

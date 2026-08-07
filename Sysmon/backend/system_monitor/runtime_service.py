"""Mock과 ROS 실행기가 따라야 하는 최소 애플리케이션 계약."""

from __future__ import annotations

from typing import Any, Protocol

# Mock과 ROS 구현은 서로 다르지만 공통 인터페이스를 사용하기 때문에 
# Flask는 현재 모드를 구체적으로 알지 않아도 동일한 방식으로 서비스를 실행하고 상태를 확인
class RuntimeService(Protocol):
    """Flask 라우트가 실행 모드를 분기하지 않도록 하는 공통 인터페이스."""

    @property
    def available(self) -> bool: ...

    @property
    def running(self) -> bool: ...

    def status(self) -> dict[str, Any]: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...

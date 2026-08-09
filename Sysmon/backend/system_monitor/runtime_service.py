from __future__ import annotations

from typing import Any, Protocol

# MockManager와 RosBridge를 Flask에서 동일한 방식으로 시작, 중지, 상태 확인할 수 있게 하는 공통 인터페이스
# Protocol은 “이 함수들을 가지고 있어야 한다는 설계 규칙”
class RuntimeService(Protocol):

    # @property가 붙으면 함수인데도 변수처럼 접근할 수 있게 해주는 기능
    # service.available → 이 서비스를 사용할 수 있는지
    # service.running → 현재 실행 중인지
    @property
    def available(self) -> bool: ...
    
    @property
    def running(self) -> bool: ...

    def status(self) -> dict[str, Any]: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...

    # 이 서비스는 Flask와 실행 서비스 사이의 공통 규칙을 정의하는 역할
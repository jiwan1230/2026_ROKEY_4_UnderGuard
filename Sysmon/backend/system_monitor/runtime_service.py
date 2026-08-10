
from __future__ import annotations

from typing import Any, Protocol

# 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 ## 핵심 #
# Flask에서 mock과 ros 실행을 동일한 방식으로 시작, 중지, 상태 확인할 수 있게 하는 공통 인터페이스를 정의
# Protocol은 “이 함수들을 가지고 있어야 한다는 설계 규칙”
class RuntimeService(Protocol):

    # @property가 붙으면 함수인데도 변수처럼 접근할 수 있게 해주는 기능
    # service.available → 이 서비스를 사용할 수 있는지
    # service.running → 현재 실행 중인지
    @property
    def available(self) -> bool: ...   
    @property
    def running(self) -> bool: ...

    # 실제로 Flask가 런타임을 제어할 때 사용하는 공통 동작    
    # Flask가 현재 상태를 받아서 웹 화면/API에 전달
    def status(self) -> dict[str, Any]: ...
    # # Flask가 실행 시작 요청
    def start(self) -> None: ...
    # Flask가 실행 중지 요청
    def stop(self) -> None: ...

    # 즉 Flask는 실제 구현이 ROS인지 Mock인지 신경 쓰지 않고, 
    # available, running, status(), start(), stop()이라는 동일한 방식으로 실행 서비스를 제어 가능

    ## available과 running은 단순한 현재 상태값이기 때문에 @property로 변수처럼 조회하도록 만들었고,
    ## status, start, stop은 상태 정보를 만들거나 실제 동작을 수행하는 기능이기 때문에 일반 메서드로 정의 

'''
Flask
  ↓
RuntimeService 인터페이스
  ↓
start() / stop() / status()
  ↓
Mock 구현 또는 ROS 구현
'''
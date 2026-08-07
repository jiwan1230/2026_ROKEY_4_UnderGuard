"""ROS 장비 없이 실시간 화면과 사건 흐름을 재현하는 데모를 생성한다."""

from __future__ import annotations

import math
import random
import threading
from typing import Callable

from .detection_service import (
    process_detection,
    record_battery_recovered,
    record_low_battery,
    record_target_lost,
    record_trap_installed,
)
from .risk_signals import DROPPINGS, ENTRY_POINT, LIVE_RODENT, normalize_risk_signal
from .state_manager import StateManager


class MockManager:
    """실제 로봇이 없어도 전체 UI 흐름을 검증하는 데모 시나리오."""

    def __init__(
        self,
        state: StateManager,
        robot_ids: list[str],
        *,
        low_battery_threshold: float = 15.0,
        map_frame: str = "map",
    ) -> None:
        self.state = state
        self.robot_ids = robot_ids
        self.low_battery_threshold = low_battery_threshold
        self.map_frame = map_frame.strip("/") or "map"
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.RLock()
        self._tick = 0
        # 대상 유실·설치 완료 뒤 자동 이동 tick이 상태를 덮지 않게 한다.
        self._scenario_holds: dict[str, str | None] = {
            robot_id: None for robot_id in robot_ids
        }
        self._scenario_active_ticks = 0
        self._scenario_completed = False
        self._initial_rat_detector = random.choice(robot_ids)

    @property
    def available(self) -> bool:
        """Mock 모드는 외부 런타임 없이 항상 사용할 수 있다."""

        return True

    @property
    def running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def status(self) -> dict:
        """모드와 무관하게 읽을 수 있는 런타임 상태를 반환한다."""

        return {
            "mode": "mock",
            "available": self.available,
            "running": self.running,
            "read_only": True,
            "low_battery_threshold": self.low_battery_threshold,
        }

    def start(self) -> None:
        """1초 주기의 Mock 갱신 스레드를 중복 없이 시작한다."""

        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="mock-manager")
        self._thread.start()

    def stop(self) -> None:
        """Mock 갱신 루프에 종료를 알리고 스레드 정리를 기다린다."""

        self._stop.set()
        thread = self._thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        if thread and thread.is_alive():
            raise RuntimeError("Mock 갱신 스레드가 제한 시간 안에 종료되지 않았습니다.")
        self._thread = None

    def trigger(self, event_type: str, robot_id: str | None = None) -> dict:
        """평가 시연용 사건을 즉시 발생시킨다.

        입력: UI가 보내는 사건 이름과 선택적 로봇 ID다.
        출력: 생성된 탐지 또는 타임라인 사건 딕셔너리다.
        사용: Mock 모드의 ``POST /api/mock/events``에서 호출한다.
        """

        robot_id = robot_id or self.robot_ids[0]
        handlers: dict[str, Callable[[], dict]] = {
            "RAT_DETECTED": lambda: self._detection(robot_id, LIVE_RODENT, 0.93, "OAK-D"),
            "RAT_HOLE_DETECTED": lambda: self._detection(
                robot_id, ENTRY_POINT, 0.87, "OAK-D"
            ),
            "DROPPINGS_DETECTED": lambda: self._detection(
                robot_id, DROPPINGS, 0.84, "OAK-D"
            ),
            "TARGET_LOST": lambda: self._target_lost(robot_id),
            "LOW_BATTERY": lambda: self._low_battery(robot_id),
            "BATTERY_RECOVERED": lambda: self._battery_recovered(robot_id),
            "TRAP_INSTALLED": lambda: self._trap_installed(robot_id),
        }
        handler = handlers.get(event_type)
        if handler is None:
            raise ValueError(f"지원하지 않는 mock event: {event_type}")
        with self._lock:
            return handler()

    def _run(self) -> None:
        """초기화 후 이동, 예약 탐지, 임무 상태를 매초 갱신한다."""

        self._initialize_robots()
        while not self._stop.wait(1.0):
            with self._lock:
                self._tick += 1
                for index, robot_id in enumerate(self.robot_ids):
                    self._update_robot_tick(index, robot_id)
                self._trigger_scheduled_detections()
                self._update_mission()

    def _initialize_robots(self) -> None:
        for index, robot_id in enumerate(self.robot_ids):
            self.state.update_robot(
                robot_id,
                role="SCOUT",
                state="SEARCHING",
                battery=88 - index * 9,
                camera_status="NORMAL",
                slam_status="NORMAL",
                nav_status="MOVING",
                current_task="쥐 공동 탐색 중",
                position_frame=self.map_frame,
                position={"x": 1.2 + index * 1.7, "y": 1.0 + index, "yaw": 0.0},
            )
        self.state.add_event(
            "두 로봇이 쥐 공동 탐색을 시작했습니다. "
            "최초 탐지 후 역할을 자동 배정합니다."
        )

    def _update_robot_tick(self, index: int, robot_id: str) -> None:
        if self._scenario_holds[robot_id] is not None:
            self.state.update_robot(robot_id, speed=0.0)
            return

        current = self.state.get_robot(robot_id)
        role_state, task, target_type = self._activity_for_role(current["role"])
        self._move_on_patrol(index, robot_id, current, role_state, task, target_type)

    @staticmethod
    def _activity_for_role(role: str) -> tuple[str, str, str | None]:
        if role == "RAT_TRACKER":
            return "TRACKING", "쥐 추적 중", LIVE_RODENT
        if role == "SURVEY_TRAP":
            return "SEARCHING", "침입구·트랩 상태 확인", ENTRY_POINT
        return "SEARCHING", "쥐 공동 탐색 중", None

    def _move_on_patrol(
        self,
        index: int,
        robot_id: str,
        current: dict,
        role_state: str,
        task: str,
        target_type: str | None,
    ) -> None:
        phase = self._tick * 0.12 + index * 1.2
        calculated_battery = max(12.0, 88 - index * 9 - self._tick * 0.03)
        battery = current["battery"]
        if battery is None or battery > 15:
            battery = calculated_battery
        self.state.update_robot(
            robot_id,
            state=role_state,
            battery=battery,
            speed=0.18 + 0.04 * abs(math.sin(phase)),
            current_task=task,
            nav_status="MOVING",
            camera_status="NORMAL",
            slam_status="NORMAL",
            position={
                "x": 2.5 + math.cos(phase) * (1.4 + index * 0.2),
                "y": 2.0 + math.sin(phase) * (1.0 + index * 0.25),
                "yaw": (phase + math.pi / 2) % (2 * math.pi),
            },
            target={
                "object_type": target_type,
                "confidence": round(0.82 + random.random() * 0.14, 3) if target_type else None,
                "distance": round(0.9 + random.random() * 0.8, 2) if target_type else None,
                "map_x": round(3.5 + math.cos(phase + 0.4), 2) if target_type else None,
                "map_y": round(2.4 + math.sin(phase + 0.4), 2) if target_type else None,
                "source": "OAK-D" if target_type else None,
            },
        )

    def _trigger_scheduled_detections(self) -> None:
        """정해진 tick에 세 종류 탐지를 생성해 시연 순서를 재현한다."""

        # 임무 완료 후 예약 탐지가 로봇을 다시 탐색/추적 상태로 되돌리지 않는다.
        if self._scenario_completed:
            return
        mission = self.state.snapshot()["mission"]
        if self._tick in {5, 18, 31}:
            tracker_id = mission.get("tracker_robot_id") or self._initial_rat_detector
            self._detection(tracker_id, LIVE_RODENT, 0.91, "OAK-D")
        if len(self.robot_ids) <= 1:
            return

        support_ids = mission.get("support_robot_ids") or [self.robot_ids[1]]
        if self._tick in {12, 26}:
            self._detection(support_ids[0], ENTRY_POINT, 0.86, "OAK-D")
        if self._tick == 22:
            self._detection(support_ids[0], DROPPINGS, 0.83, "OAK-D")

    def _update_mission(self) -> None:
        """장시간 시연 시 자동 종료 시점만 관리한다.

        화면에 보이는 전체 임무 상태 문구는 이제 ``StateManager.snapshot()``이
        각 로봇의 실제 state로부터 매번 다시 계산하므로(2.2 개선), 여기서는
        시연용 활동 tick 카운트와 ``_scenario_completed`` 플래그만 관리한다.
        """

        states = [robot["state"] for robot in self.state.snapshot()["robots"]]
        active = any(state in {"TRACKING", "SEARCHING", "RETURNING"} for state in states)
        if active:
            self._scenario_active_ticks += 1
        elif states and all(state == "COMPLETED" for state in states):
            self._scenario_completed = True

        if self._scenario_active_ticks >= 50 and active:
            self._complete_active_robots()
            self._scenario_completed = True

    def _complete_active_robots(self) -> None:
        for robot_id in self.robot_ids:
            if self._scenario_holds[robot_id] is not None:
                continue
            self.state.update_robot(
                robot_id,
                state="COMPLETED",
                speed=0.0,
                nav_status="SUCCEEDED",
                current_task="임무 완료 · 다음 지시 대기",
            )
            self._scenario_holds[robot_id] = "COMPLETED"

    def _detection(
        self,
        robot_id: str,
        object_type: str,
        confidence: float,
        source: str,
    ) -> dict:
        """Mock 탐지 좌표를 계산해 공통 탐지 처리 서비스로 전달한다."""

        if self._scenario_holds[robot_id] == "TARGET_LOST":
            self._scenario_holds[robot_id] = None
        object_type = normalize_risk_signal(object_type)
        robot = self.state.get_robot(robot_id)
        target = robot["target"]
        target_matches = (
            normalize_risk_signal(target.get("object_type")) == object_type
            and target.get("map_x") is not None
            and target.get("map_y") is not None
        )
        if target_matches:
            map_x = target["map_x"]
            map_y = target["map_y"]
            distance = target.get("distance")
        else:
            offsets = {
                LIVE_RODENT: (0.45, 0.15),
                ENTRY_POINT: (-0.35, 0.30),
                DROPPINGS: (0.30, -0.35),
            }
            offset_x, offset_y = offsets.get(object_type, (0.25, 0.25))
            map_x = round(max(0.2, min(5.8, robot["position"]["x"] + offset_x)), 2)
            map_y = round(max(0.2, min(4.8, robot["position"]["y"] + offset_y)), 2)
            distance = round(math.hypot(offset_x, offset_y), 2)
        detection = {
            "robot_id": robot_id,
            "object_type": object_type,
            "confidence": confidence,
            "distance": distance,
            "map_x": map_x,
            "map_y": map_y,
            "source": source,
            "review_status": "UNREVIEWED",
        }
        return process_detection(
            self.state,
            detection,
            event_message=f"{object_type} 탐지 결과를 수신했습니다.",
        )

    def _target_lost(self, robot_id: str) -> dict:
        event = record_target_lost(self.state, robot_id)
        self._scenario_holds[robot_id] = "TARGET_LOST"
        return event

    def _low_battery(self, robot_id: str) -> dict:
        value = max(0.0, self.low_battery_threshold - 1.0)
        return record_low_battery(self.state, robot_id, value)

    def _battery_recovered(self, robot_id: str) -> dict:
        """배터리 부족 → 경고 → 복구 시연 흐름의 마지막 단계를 재현한다.

        배터리 값을 정상 범위로 되돌리면 이후 tick에서 정상 순찰 배터리
        곡선이 재개되고(``_move_on_patrol``), 화면의 저전압 경고와 카드의
        운영 제한 문구는 실시간 배터리 값 기준으로 자동 사라진다.
        """

        return record_battery_recovered(self.state, robot_id)

    def _trap_installed(self, robot_id: str) -> dict:
        event = record_trap_installed(
            self.state,
            robot_id,
            map_frame=self.map_frame,
        )
        self._scenario_holds[robot_id] = "TRAP_INSTALLED"
        return event

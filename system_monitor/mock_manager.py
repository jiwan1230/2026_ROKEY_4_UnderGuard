"""ROS 장비 없이 화면·저장·사건 흐름을 재현하는 데모를 생성한다."""

from __future__ import annotations

import math
import random
import threading
import time
from typing import Callable

from .database import Database
from .detection_service import (
    process_detection,
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
        db: Database,
        robot_ids: list[str],
        *,
        low_battery_threshold: float = 15.0,
        map_frame: str = "map",
    ) -> None:
        self.state = state
        self.db = db
        self.robot_ids = robot_ids
        self.low_battery_threshold = low_battery_threshold
        self.map_frame = map_frame.strip("/") or "map"
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.RLock()
        self._tick = 0
        self._command_modes: dict[str, str | None] = {
            robot_id: None for robot_id in robot_ids
        }
        self._scenario_active = True
        self._mission_progress = 0
        self._mission_started_at = time.time()
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
            "commands_enabled": True,
            "mock_events_enabled": True,
            "mission_progress_available": True,
            "data_source": "SIMULATED",
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

    def reset_demo(self) -> dict[str, int]:
        """운영 데이터와 Mock 상태를 지우고 자동 시나리오를 대기시킨다.

        입력: 없음. 출력: DB 테이블별 삭제 건수다.
        사용: 관리자 초기화 API에서 호출한다. Mock 갱신과 같은 잠금을 사용하므로
        삭제 중 예약 탐지가 다시 저장되지 않으며, 서비스 스레드는 계속 살아 있다.
        """

        with self._lock:
            deleted = self.db.clear_operational_data()
            self.state.reset()
            self._tick = 0
            self._command_modes = {robot_id: None for robot_id in self.robot_ids}
            self._scenario_active = False
            self._mission_progress = 0
            self._mission_started_at = time.time()
            self._initial_rat_detector = random.choice(self.robot_ids)
            self._initialize_idle_robots()
            return deleted

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
            "TRAP_INSTALLED": lambda: self._trap_installed(robot_id),
        }
        handler = handlers.get(event_type)
        if handler is None:
            raise ValueError(f"지원하지 않는 mock event: {event_type}")
        with self._lock:
            return handler()

    def apply_command(self, robot_id: str, command: str) -> dict:
        """웹 명령을 적용하고 이후 Mock tick에서도 운전 모드를 보존한다.

        입력: 등록 로봇 ID와 StateManager가 지원하는 명령 이름이다.
        출력: DB에도 저장할 명령 사건 딕셔너리다.
        사용: 앱의 명령 API가 Mock 모드일 때 호출한다.
        """

        with self._lock:
            was_inactive = not self._scenario_active
            event = self.state.apply_command(robot_id, command)
            if command in {"START_SCOUTING", "START_TRACKING", "START_SEARCH", "RESUME"}:
                self._scenario_active = True
                self._command_modes[robot_id] = None
                if was_inactive or self._mission_progress >= 100:
                    self._tick = 0
                    self._mission_progress = 0
                    self._mission_started_at = time.time()
            else:
                self._command_modes[robot_id] = command
            return event

    def send_command(self, robot_id: str, command: str) -> dict:
        """명령을 적용하고 ROS와 같은 형태의 API 결과를 반환한다."""

        event = self.apply_command(robot_id, command)
        self.db.insert_event(event)
        return {
            "accepted": True,
            "robot_id": robot_id,
            "command": command,
            "event": event,
            "reason": None,
        }

    def _run(self) -> None:
        """초기화 후 이동, 예약 탐지, 임무 진행률을 매초 갱신한다."""

        self._initialize_robots()
        while not self._stop.wait(1.0):
            with self._lock:
                if not self._scenario_active:
                    # Mock 연결은 유지하되 위치·진행률·예약 탐지는 갱신하지 않는다.
                    for robot_id in self.robot_ids:
                        self.state.update_robot(robot_id, speed=0.0)
                    continue
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

    def _initialize_idle_robots(self) -> None:
        """초기화 직후 두 Mock 로봇을 온라인 임무 대기 상태로 배치한다."""

        for index, robot_id in enumerate(self.robot_ids):
            self.state.update_robot(
                robot_id,
                role="SCOUT",
                state="IDLE",
                battery=88 - index * 9,
                speed=0.0,
                camera_status="NORMAL",
                slam_status="NORMAL",
                nav_status="READY",
                current_task="데이터 초기화 · 임무 대기",
                position_frame=self.map_frame,
                position={"x": 1.2 + index * 1.7, "y": 1.0 + index, "yaw": 0.0},
            )

    def _update_robot_tick(self, index: int, robot_id: str) -> None:
        mode = self._command_modes[robot_id]
        if mode in {"PAUSE", "STOP", "TRAP_INSTALLED", "TARGET_LOST"}:
            self.state.update_robot(robot_id, speed=0.0)
            return

        current = self.state.get_robot(robot_id)
        if mode == "RETURN_HOME":
            self._move_toward_home(index, robot_id, current)
            return

        role_state, task, target_type = self._activity_for_role(current["role"])
        self._move_on_patrol(index, robot_id, current, role_state, task, target_type)

    @staticmethod
    def _activity_for_role(role: str) -> tuple[str, str, str | None]:
        if role == "RAT_TRACKER":
            return "TRACKING", "쥐 추적 중", LIVE_RODENT
        if role == "SURVEY_TRAP":
            return "SEARCHING", "쥐구멍 탐색 및 트랩 설치", ENTRY_POINT
        return "SEARCHING", "쥐 공동 탐색 중", None

    def _move_toward_home(self, index: int, robot_id: str, current: dict) -> None:
        home_x, home_y = 1.2 + index * 1.7, 1.0 + index
        dx = home_x - current["position"]["x"]
        dy = home_y - current["position"]["y"]
        distance = math.hypot(dx, dy)
        if distance < 0.08:
            self.state.update_robot(
                robot_id,
                state="IDLE",
                speed=0.0,
                nav_status="READY",
                current_task="복귀 완료",
                position={"x": home_x, "y": home_y},
            )
            self._command_modes[robot_id] = "STOP"
            return

        step = min(0.18, distance)
        self.state.update_robot(
            robot_id,
            state="RETURNING",
            speed=step,
            nav_status="MOVING",
            position={
                "x": current["position"]["x"] + dx / distance * step,
                "y": current["position"]["y"] + dy / distance * step,
                "yaw": math.atan2(dy, dx),
            },
        )

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
        if self._mission_progress >= 100:
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
        states = [robot["state"] for robot in self.state.snapshot()["robots"]]
        active = any(state in {"TRACKING", "SEARCHING", "RETURNING"} for state in states)
        if active:
            self._mission_progress = min(100, self._mission_progress + 2)
            mission_status = "RUNNING"
        elif states and all(state == "PAUSED" for state in states):
            mission_status = "PAUSED"
        elif states and all(state == "COMPLETED" for state in states):
            mission_status = "COMPLETED"
            self._mission_progress = 100
        else:
            mission_status = "READY"

        if self._mission_progress >= 100 and active:
            self._complete_active_robots()
            mission_status = "COMPLETED"

        self.state.set_mission(
            status=mission_status,
            progress=self._mission_progress,
            elapsed_sec=int(time.time() - self._mission_started_at),
            started_at=self._mission_started_at,
        )

    def _complete_active_robots(self) -> None:
        for robot_id in self.robot_ids:
            if self._command_modes[robot_id] is not None:
                continue
            self.state.update_robot(
                robot_id,
                state="COMPLETED",
                speed=0.0,
                nav_status="SUCCEEDED",
                current_task="임무 완료 · 다음 지시 대기",
            )
            self._command_modes[robot_id] = "TRAP_INSTALLED"

    def _detection(
        self,
        robot_id: str,
        object_type: str,
        confidence: float,
        source: str,
    ) -> dict:
        """Mock 탐지 좌표를 계산해 공통 탐지 처리 서비스로 전달한다."""

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
            self.db,
            detection,
            event_message=f"{object_type} 탐지 결과가 기록되었습니다.",
        )

    def _target_lost(self, robot_id: str) -> dict:
        event = record_target_lost(self.state, self.db, robot_id)
        self._command_modes[robot_id] = "TARGET_LOST"
        return event

    def _low_battery(self, robot_id: str) -> dict:
        value = max(0.0, self.low_battery_threshold - 1.0)
        return record_low_battery(self.state, self.db, robot_id, value)

    def _trap_installed(self, robot_id: str) -> dict:
        event = record_trap_installed(
            self.state,
            self.db,
            robot_id,
            map_frame=self.map_frame,
        )
        self._command_modes[robot_id] = "TRAP_INSTALLED"
        return event

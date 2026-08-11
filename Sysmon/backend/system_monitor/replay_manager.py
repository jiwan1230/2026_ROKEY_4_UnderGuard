"""알고리즘 검증 시뮬레이션이 남긴 궤적(JSON)을 실시간처럼 재생한다.

Mock은 지도 중심을 도는 원(``math.cos``/``math.sin``)을 그릴 뿐 벽을 모른다.
이 모듈은 그 대신 ``herding_controller_dual``의 검증 시뮬레이터
(``run_validation.py``)가 실제로 계산해 기록해 둔 Driver/Blocker 경로를
그대로 재생한다 — 화면에 뜨는 좌표가 알고리즘이 실제로 만든 값이다.
"""

from __future__ import annotations

import copy
import json
import threading
import time
from pathlib import Path
from typing import Any

from .risk_signals import LIVE_RODENT
from .state_manager import StateManager

DEFAULT_FRAMES_PATH = Path(__file__).resolve().parent / "replay_data" / "real_map_frames.json"

# herding_controller_dual의 FSM 상태(state_machine.py)를 대시보드 상태값으로 옮긴다.
_FSM_STATE_MAP = {
    "IDLE": "IDLE",
    "SEARCH": "SEARCHING",
    "TRACK": "TRACKING",
    "HERD": "TRACKING",
    "CORNER": "TRACKING",
    "LOST": "TARGET_LOST",
    "CAPTURED": "COMPLETED",
}

# Driver(추적) 카드 문구. HERD/CORNER는 Driver 입장에서는 계속 표적을 미는
# 중이라 "몰이"로 통일한다.
_DRIVER_TASK_MAP = {
    "IDLE": "대기",
    "SEARCH": "쥐 공동 탐색 중",
    "TRACK": "쥐 추적 중",
    "HERD": "쥐 몰이 중",
    "CORNER": "포획 지점으로 유도 중",
    "LOST": "대상 재탐색 대기",
    "CAPTURED": "포획 완료 · 다음 지시 대기",
}

# Blocker(지원) 카드 문구. Driver와 같은 FSM 상태를 공유하지만 실제로 하는
# 일은 다르다 — herding_planner.compute_blocking_point()가 계산한 대로
# 도주 경로를 선점해서 막는 것뿐이다. 문구를 Driver와 똑같이 두면(예전 버전)
# "로봇6도 지금 몰이 중"으로 보여서 Blocker라는 걸 알아채기 어려웠다.
_BLOCKER_TASK_MAP = {
    "IDLE": "대기",
    "SEARCH": "쥐 공동 탐색 중",
    "TRACK": "도주 경로 선점 이동 중",
    "HERD": "도주 경로 차단 중",
    "CORNER": "구석 도주 경로 차단 중",
    "LOST": "대상 재탐색 대기",
    "CAPTURED": "포획 완료 · 다음 지시 대기",
}


class ReplayManager:
    """검증 궤적 파일 하나를 무한 반복 재생하는 읽기 전용 실행기.

    Mock/ROS와 같은 ``RuntimeService`` 계약(available/running/status/
    start/stop)을 따르므로 app.py는 모드를 구분하지 않고 동일하게 다룬다.
    """

    def __init__(
        self,
        state: StateManager,
        robot_ids: list[str],
        *,
        frames_path: Path = DEFAULT_FRAMES_PATH,
        trial_index: int = 0,
        speed: float = 1.0,
        map_frame: str = "map",
    ) -> None:
        # app.py는 mock/ros처럼 이 객체도 모드와 무관하게 항상 만든다 — 그래서
        # 로봇이 1대뿐인 설정(mock/ros 테스트 등)에서도 예외 없이 만들어지되,
        # 재생에 필요한 Driver+Blocker 2대가 없으면 available만 False로 남는다.
        self.state = state
        self.driver_id, self.blocker_id = (robot_ids[0], robot_ids[1]) if len(robot_ids) >= 2 else (None, None)
        self.speed = speed if speed > 0 else 1.0
        self.map_frame = map_frame.strip("/") or "map"
        self._frames_path = Path(frames_path)
        (
            self._trial,
            self._known_traps,
            self._record_meta,
            self._trials,
            self._trial_index,
        ) = (
            self._load(self._frames_path, trial_index)
            if self.driver_id
            else (None, {}, {}, [], 0)
        )
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._discovered = False
        self._captured = False

    @property
    def available(self) -> bool:
        """재생할 궤적을 정상적으로 읽었을 때만 사용 가능하다."""

        return self._trial is not None

    @property
    def running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def status(self) -> dict[str, Any]:
        return {
            "mode": "replay",
            "available": self.available,
            "running": self.running,
            "read_only": True,
            "frames_path": str(self._frames_path),
            "model": self._trial["model"] if self._trial else None,
            "trial_success": self._trial["success"] if self._trial else None,
            "frame_count": len(self._trial["frames"]) if self._trial else 0,
        }

    def history_record(self, trial_index: int | None = None) -> dict[str, Any]:
        """요청한 검증 시험을 기록 화면이 읽을 수 있는 형태로 반환한다.

        실시간 재생은 ``apply_frame()``이 StateManager를 계속 바꾸지만, 기록
        화면은 원본 시험 전체를 한 번에 읽어 경로를 그린다. 내부 dict를 그대로
        돌려주면 API 사용자가 실수로 ReplayManager의 원본까지 바꿀 수 있으므로
        복사본을 제공한다. ``trial_index``는 기록 조회에만 사용하며 현재 실행 중인
        Replay 시험인 ``self._trial``은 바꾸지 않는다.
        """

        selected_index = self._trial_index
        selected_trial = self._trial
        if trial_index is not None:
            if trial_index < 0 or trial_index >= len(self._trials):
                raise IndexError("trial_index_out_of_range")
            selected_index = trial_index
            selected_trial = self._trials[trial_index]

        trial_options = []
        for index, trial in enumerate(self._trials):
            frames = trial.get("frames") or []
            duration = trial.get("duration")
            if duration is None and frames:
                duration = frames[-1].get("t")
            trial_options.append(
                {
                    "index": index,
                    "model": trial.get("model"),
                    "success": bool(trial.get("success")),
                    "duration": duration,
                    "frame_count": len(frames),
                    "goal_name": trial.get("goal_name"),
                    "seed": trial.get("seed"),
                }
            )

        return {
            "available": selected_trial is not None,
            "source_name": self._frames_path.name,
            "map_frame": self.map_frame,
            "driver_id": self.driver_id,
            "blocker_id": self.blocker_id,
            "selected_trial_index": selected_index,
            "trial_count": len(self._trials),
            "trial_options": trial_options,
            "traps": copy.deepcopy(self._known_traps),
            "trial": copy.deepcopy(selected_trial),
            # Replay JSON 안의 지도와 좌표 범위를 함께 보내야 프런트엔드가
            # 별도의 ROS map 파일 없이도 기록 당시의 배경 위에 경로를 그릴 수 있다.
            "map_image": self._record_meta.get("map_image"),
            "photo_frame": copy.deepcopy(self._record_meta.get("photo_frame")),
            "parameters": {
                name: self._record_meta.get(name)
                for name in ("capture_radius", "panic_distance", "sensor_range")
            },
        }

    def start(self) -> None:
        """궤적 재생 스레드를 중복 없이 시작한다."""

        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="replay-manager")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._thread = None

    @staticmethod
    def _load(
        path: Path, trial_index: int
    ) -> tuple[
        dict[str, Any] | None,
        dict[str, Any],
        dict[str, Any],
        list[dict[str, Any]],
        int,
    ]:
        """파일을 못 읽거나 trial이 없으면 조용히 재생 불가 상태로 남긴다.

        ``available``이 이걸 반영하므로 /api/health가 degraded로 표시한다 —
        Mock처럼 항상 켜져 있어야 하는 기능이 아니라 데모용 부가 모드다.
        ``traps``(top/left/bottom 등 방의 고정 포획 지점)는 trial이 아니라
        파일 최상위에 한 번만 들어있어 trial과 별도로 꺼낸다.
        """

        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None, {}, {}, [], 0
        trials = data.get("trials") or []
        if not trials:
            return None, {}, {}, [], 0
        selected_index = trial_index % len(trials)
        metadata = {
            name: data.get(name)
            for name in (
                "map_image",
                "photo_frame",
                "capture_radius",
                "panic_distance",
                "sensor_range",
            )
        }
        return (
            trials[selected_index],
            data.get("traps") or {},
            metadata,
            trials,
            selected_index,
        )

    def _run(self) -> None:
        if self._trial is None:
            return
        self._announce_known_layout()
        while not self._stop.is_set():
            self._play_once()
            if self._stop.wait(3.0):  # 한 바퀴 끝나면 3초 쉬고 처음부터 반복
                return

    def _announce_known_layout(self) -> None:
        """재생 시작 직후, 방의 고정 정보(덫 위치·쥐 시작 위치)를 한 번에 보여준다.

        실시간 로봇 시점(안 보이면 표시 안 함)과 달리 Replay는 이미 다 끝난
        기록을 되짚는 것이라, 사용자가 처음부터 방 구조·표적 위치를 보고
        따라갈 수 있는 게 더 유용하다. ``_run()``이 재생 루프 진입 전 한 번만
        호출하므로, 반복 재생(``_play_once`` 재호출)마다 덫이 중복으로
        쌓이지 않는다.
        """

        for robot_id in (self.driver_id, self.blocker_id):
            self.state.update_robot(
                robot_id,
                role="SCOUT",
                state="SEARCHING",
                battery=90.0,
                camera_status="NORMAL",
                slam_status="NORMAL",
                nav_status="MOVING",
                current_task="쥐 공동 탐색 중",
                position_frame=self.map_frame,
            )
        for name, point in self._known_traps.items():
            x, y = point
            self.state.add_trap(
                {
                    "robot_id": None,
                    "map_x": round(x, 2),
                    "map_y": round(y, 2),
                    "status": "INSTALLED",
                    "name": name,
                }
            )
        first_frame = self._trial["frames"][0]
        start_x, start_y = first_frame["target"]
        self.state.add_detection(
            {
                "robot_id": self.driver_id,
                "object_type": LIVE_RODENT,
                "confidence": None,
                "distance": None,
                "map_x": round(start_x, 2),
                "map_y": round(start_y, 2),
                "source": "REPLAY",
                "review_status": "UNREVIEWED",
                "image_url": None,
            }
        )
        self.state.add_event(
            f"저장된 검증 시뮬레이션(model={self._trial['model']})을 재생합니다. "
            f"덫 {len(self._known_traps)}곳과 쥐 시작 위치를 표시합니다.",
        )

    def _play_once(self) -> None:
        self._discovered = False
        self._captured = False
        start_wall = time.time()
        for frame in self._trial["frames"]:
            if self._stop.is_set():
                return
            due = start_wall + frame["t"] / self.speed
            wait = due - time.time()
            if wait > 0 and self._stop.wait(wait):
                return
            self.apply_frame(frame)

    def apply_frame(self, frame: dict[str, Any]) -> None:
        """프레임 한 개를 StateManager에 반영한다.

        시간 대기 로직과 분리해 둬서 테스트가 실시간 재생 없이 프레임을
        순서대로 직접 호출해 검증할 수 있다.
        """

        fsm_state = frame.get("state", "SEARCH")
        dashboard_state = _FSM_STATE_MAP.get(fsm_state, "SEARCHING")
        driver_task = _DRIVER_TASK_MAP.get(fsm_state, "쥐 공동 탐색 중")
        blocker_task = _BLOCKER_TASK_MAP.get(fsm_state, "쥐 공동 탐색 중")
        target_x, target_y = frame["target"]
        driver_x, driver_y = frame["driver"]
        blocker_x, blocker_y = frame["blocker"]
        discovered = bool(frame.get("discovered"))

        if discovered and not self._discovered:
            self._discovered = True
            self.state.add_detection(
                {
                    "robot_id": self.driver_id,
                    "object_type": LIVE_RODENT,
                    "confidence": 0.92,
                    "distance": round(frame.get("dist", 0.0), 2),
                    "map_x": round(target_x, 2),
                    "map_y": round(target_y, 2),
                    "source": "OAK-D",
                    "review_status": "UNREVIEWED",
                    "image_url": None,
                }
            )
            if self.state.assign_roles_from_rat_detection(self.driver_id):
                self.state.add_event(
                    f"{self.driver_id}가 쥐를 최초 발견했습니다. 추적 역할로 전환합니다.",
                    robot_id=self.driver_id,
                    event_type="ROLE_ASSIGNED",
                )

        target = (
            {
                "object_type": LIVE_RODENT,
                "confidence": 0.9,
                "distance": round(frame.get("dist", 0.0), 2),
                "map_x": round(target_x, 2),
                "map_y": round(target_y, 2),
                "source": "OAK-D",
            }
            if discovered
            else {
                "object_type": None,
                "confidence": None,
                "distance": None,
                "map_x": None,
                "map_y": None,
                "source": None,
            }
        )
        self.state.update_robot(
            self.driver_id,
            state=dashboard_state,
            current_task=driver_task,
            speed=0.2,
            nav_status="MOVING",
            camera_status="NORMAL",
            slam_status="NORMAL",
            position={"x": driver_x, "y": driver_y, "yaw": 0.0},
            target=target,
        )
        self.state.update_robot(
            self.blocker_id,
            state=dashboard_state,
            current_task=blocker_task if dashboard_state != "SEARCHING" else "침입구·트랩 상태 확인",
            speed=0.2,
            nav_status="MOVING",
            camera_status="NORMAL",
            slam_status="NORMAL",
            position={"x": blocker_x, "y": blocker_y, "yaw": 0.0},
        )

        # 덫 위치는 _announce_known_layout()이 재생 시작 시 이미 다 보여줬다 —
        # 여기서는 "그중 한 곳에서 실제로 포획됐다"는 사건만 한 번 남긴다
        # (또 add_trap을 부르면 같은 자리에 마커가 겹쳐 4번째 덫처럼 보인다).
        if fsm_state == "CAPTURED" and self._trial.get("success") and not self._captured:
            self._captured = True
            self.state.add_event(
                "표적을 포획 지점으로 유도하는 데 성공했습니다.",
                robot_id=self.driver_id,
                event_type="TRAP_INSTALLED",
            )

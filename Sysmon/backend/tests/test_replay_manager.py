import json
import tempfile
import unittest
from pathlib import Path

from system_monitor.replay_manager import ReplayManager
from system_monitor.state_manager import StateManager


def _write_frames_file(tmp_dir: Path, trials: list[dict], traps: dict | None = None) -> Path:
    path = tmp_dir / "frames.json"
    path.write_text(
        json.dumps({"trials": trials, "traps": traps or {}}), encoding="utf-8"
    )
    return path


def _frame(**overrides):
    base = {
        "t": 0.0,
        "target": [1.0, 1.0],
        "driver": [0.0, 0.0],
        "blocker": [2.0, 2.0],
        "state": "SEARCH",
        "discovered": False,
        "dist": 3.0,
    }
    base.update(overrides)
    return base


class ReplayManagerTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.state = StateManager(
            [("robot4", "SCOUT"), ("robot6", "SCOUT")], offline_timeout_sec=100
        )

    def _manager(self, trials, traps=None, **kwargs):
        path = _write_frames_file(Path(self.tmp.name), trials, traps)
        return ReplayManager(
            self.state, ["robot4", "robot6"], frames_path=path, **kwargs
        )

    def test_missing_file_is_reported_as_unavailable(self):
        manager = ReplayManager(
            self.state,
            ["robot4", "robot6"],
            frames_path=Path(self.tmp.name) / "nope.json",
        )
        self.assertFalse(manager.available)
        self.assertFalse(manager.status()["available"])

    def test_positions_are_applied_to_driver_and_blocker(self):
        manager = self._manager(
            [{"model": "reactive_flee", "success": True, "frames": [_frame()]}]
        )
        manager.apply_frame(manager._trial["frames"][0])
        robots = {r["robot_id"]: r for r in self.state.snapshot()["robots"]}

        self.assertEqual((robots["robot4"]["position"]["x"], robots["robot4"]["position"]["y"]), (0.0, 0.0))
        self.assertEqual((robots["robot6"]["position"]["x"], robots["robot6"]["position"]["y"]), (2.0, 2.0))

    def test_fsm_state_is_translated_to_dashboard_state(self):
        manager = self._manager(
            [{"model": "reactive_flee", "success": True, "frames": [_frame(state="HERD")]}]
        )
        manager.apply_frame(manager._trial["frames"][0])
        robot = self.state.get_robot("robot4")
        self.assertEqual(robot["state"], "TRACKING")

    def test_discovery_creates_exactly_one_detection_and_assigns_roles(self):
        manager = self._manager(
            [
                {
                    "model": "reactive_flee",
                    "success": True,
                    "frames": [
                        _frame(t=0.0, discovered=False),
                        _frame(t=0.1, discovered=True, state="TRACK"),
                        _frame(t=0.2, discovered=True, state="TRACK"),
                    ],
                }
            ]
        )
        for frame in manager._trial["frames"]:
            manager.apply_frame(frame)

        snapshot = self.state.snapshot()
        self.assertEqual(len(snapshot["detections"]), 1)
        self.assertEqual(snapshot["mission"]["tracker_robot_id"], "robot4")
        robots = {r["robot_id"]: r for r in snapshot["robots"]}
        self.assertEqual(robots["robot4"]["role"], "RAT_TRACKER")
        self.assertEqual(robots["robot6"]["role"], "SURVEY_TRAP")

    def test_capture_on_successful_trial_fires_exactly_one_capture_event(self):
        """덫 마커 자체는 _announce_known_layout()이 시작할 때 이미 다 띄워
        둔다 — 여기서는 "그중 한 곳에서 실제로 포획됐다"는 사건 1건만 확인한다.
        같은 자리에 또 add_trap을 부르면 4번째 덫처럼 겹쳐 보이므로 하면 안 된다."""

        manager = self._manager(
            [
                {
                    "model": "reactive_flee",
                    "success": True,
                    "frames": [
                        _frame(state="CAPTURED"),
                        _frame(state="CAPTURED"),
                        _frame(state="CAPTURED"),
                    ],
                }
            ]
        )
        for frame in manager._trial["frames"]:
            manager.apply_frame(frame)

        self.assertEqual(self.state.snapshot()["traps"], [])
        capture_events = [
            e for e in self.state.snapshot()["events"] if e["event_type"] == "TRAP_INSTALLED"
        ]
        self.assertEqual(len(capture_events), 1)

    def test_capture_on_failed_trial_does_not_fire_a_capture_event(self):
        manager = self._manager(
            [
                {
                    "model": "reactive_flee",
                    "success": False,
                    "frames": [_frame(state="CAPTURED")],
                }
            ]
        )
        manager.apply_frame(manager._trial["frames"][0])
        capture_events = [
            e for e in self.state.snapshot()["events"] if e["event_type"] == "TRAP_INSTALLED"
        ]
        self.assertEqual(capture_events, [])

    def test_known_layout_shows_all_traps_and_initial_target_upfront(self):
        """Replay는 로봇 시점(fog of war)이 아니라 이미 끝난 기록이라, 재생
        시작 즉시 방의 고정 덫 3곳과 쥐 시작 위치가 화면에 보여야 한다."""

        manager = self._manager(
            [
                {
                    "model": "reactive_flee",
                    "success": True,
                    "frames": [_frame(target=[0.5, -8.25], discovered=False)],
                }
            ],
            traps={"top": [-2.81, -5.36], "left": [-2.17, -2.21], "bottom": [1.74, -6.36]},
        )
        manager._announce_known_layout()

        snapshot = self.state.snapshot()
        self.assertEqual(len(snapshot["traps"]), 3)
        self.assertEqual({t["name"] for t in snapshot["traps"]}, {"top", "left", "bottom"})
        self.assertEqual(len(snapshot["detections"]), 1)
        self.assertEqual(
            (snapshot["detections"][0]["map_x"], snapshot["detections"][0]["map_y"]),
            (0.5, -8.25),
        )

    def test_blocker_task_text_differs_from_driver_once_tracking(self):
        """로봇6(Blocker)이 로봇4(Driver)와 똑같은 "몰이 중" 문구를 쓰면 화면만
        보고 누가 Blocker인지 구분이 안 된다 — 서로 다른 문구를 써야 한다."""

        manager = self._manager(
            [{"model": "reactive_flee", "success": True, "frames": [_frame(state="HERD")]}]
        )
        manager.apply_frame(manager._trial["frames"][0])
        robots = {r["robot_id"]: r for r in self.state.snapshot()["robots"]}

        self.assertNotEqual(robots["robot4"]["current_task"], robots["robot6"]["current_task"])
        self.assertIn("차단", robots["robot6"]["current_task"])

    def test_trial_index_wraps_around_available_trials(self):
        manager = self._manager(
            [
                {"model": "a", "success": True, "frames": [_frame()]},
                {"model": "b", "success": True, "frames": [_frame()]},
            ],
            trial_index=2,  # len(trials) == 2 -> wraps back to index 0
        )
        self.assertEqual(manager._trial["model"], "a")

    def test_single_robot_is_unavailable_instead_of_raising(self):
        """app.py는 mock/ros 모드에서도 이 객체를 항상 만든다 — 로봇이 1대뿐인
        설정(예: 기존 mock/ros 테스트)에서 예외로 앱 생성 자체가 깨지면 안 된다."""

        manager = ReplayManager(self.state, ["robot4"])
        self.assertFalse(manager.available)
        manager.start()  # 로봇 부족으로 즉시 종료돼야 하며 예외를 내면 안 된다
        manager.stop()
        self.assertFalse(manager.running)


if __name__ == "__main__":
    unittest.main()

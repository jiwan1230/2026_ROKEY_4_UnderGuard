import unittest

from system_monitor.mock_manager import MockManager
from system_monitor.state_manager import StateManager


class MockManagerTest(unittest.TestCase):
    def setUp(self):
        self.state = StateManager([("robot4", "SURVEY_TRAP")], offline_timeout_sec=100)
        self.state.update_robot(
            "robot4",
            state="SEARCHING",
            position_frame="map",
            position={"x": 1.0, "y": 1.0},
            target={
                "object_type": "rat_hole",
                "map_x": 4.0,
                "map_y": 4.0,
                "distance": 1.2,
            },
        )
        self.mock = MockManager(self.state, ["robot4"])

    def test_matching_object_uses_current_target_coordinates(self):
        detection = self.mock.trigger("RAT_HOLE_DETECTED", "robot4")
        self.assertEqual((detection["map_x"], detection["map_y"]), (4.0, 4.0))

    def test_different_object_gets_independent_coordinates(self):
        detection = self.mock.trigger("DROPPINGS_DETECTED", "robot4")
        self.assertEqual((detection["map_x"], detection["map_y"]), (1.3, 0.65))

    def test_rat_detection_assigns_roles_from_detector(self):
        state = StateManager(
            [("robot4", "SCOUT"), ("robot5", "SCOUT")], offline_timeout_sec=100
        )
        state.update_robot("robot4", state="SEARCHING", position={"x": 1, "y": 1})
        state.update_robot("robot5", state="SEARCHING", position={"x": 2, "y": 2})
        mock = MockManager(state, ["robot4", "robot5"])

        mock.trigger("RAT_DETECTED", "robot5")
        snapshot = state.snapshot()
        robots = {robot["robot_id"]: robot for robot in snapshot["robots"]}

        self.assertEqual(snapshot["mission"]["tracker_robot_id"], "robot5")
        self.assertEqual(robots["robot5"]["role"], "RAT_TRACKER")
        self.assertEqual(robots["robot4"]["role"], "SURVEY_TRAP")
        self.assertEqual(snapshot["events"][0]["event_type"], "DETECTION")

    def test_mock_events_use_canonical_risk_signal_names(self):
        expected = {
            "RAT_DETECTED": "LIVE_RODENT",
            "RAT_HOLE_DETECTED": "ENTRY_POINT",
            "DROPPINGS_DETECTED": "DROPPINGS",
        }
        for event_type, object_type in expected.items():
            self.assertEqual(
                self.mock.trigger(event_type, "robot4")["object_type"], object_type
            )

    def test_trap_event_uses_current_map_position_in_memory(self):
        event = self.mock.trigger("TRAP_INSTALLED", "robot4")
        trap = self.state.snapshot()["traps"][0]

        self.assertEqual(event["event_type"], "TRAP_INSTALLED")
        self.assertEqual((trap["map_x"], trap["map_y"]), (1.0, 1.0))
        self.assertEqual(trap["id"], 1)

    def test_trap_event_rejects_non_map_coordinates(self):
        self.state.update_robot("robot4", position_frame="odom")
        with self.assertRaisesRegex(ValueError, "map frame"):
            self.mock.trigger("TRAP_INSTALLED", "robot4")
        self.assertEqual(self.state.snapshot()["traps"], [])

    def test_scheduled_detection_does_not_reopen_completed_mission(self):
        self.mock._scenario_completed = True
        self.state.update_robot("robot4", state="COMPLETED")
        self.mock._tick = 5

        self.mock._trigger_scheduled_detections()

        snapshot = self.state.snapshot()
        self.assertEqual(snapshot["robots"][0]["state"], "COMPLETED")
        self.assertEqual(snapshot["detections"], [])

    def test_mission_reports_running_while_a_robot_is_active(self):
        # setUp이 robot4를 이미 SEARCHING(활동 상태)으로 두었다.
        self.mock._update_mission()

        self.assertEqual(self.state.snapshot()["mission"]["status"], "RUNNING")
        self.assertEqual(self.mock._scenario_active_ticks, 1)

    def test_mission_is_ready_when_no_robot_is_active(self):
        self.state.update_robot("robot4", state="IDLE")

        self.mock._update_mission()

        self.assertEqual(self.state.snapshot()["mission"]["status"], "READY")
        self.assertEqual(self.mock._scenario_active_ticks, 0)

    def test_mission_completes_once_all_robots_report_completed(self):
        self.state.update_robot("robot4", state="COMPLETED")

        self.mock._update_mission()

        self.assertEqual(self.state.snapshot()["mission"]["status"], "COMPLETED")
        self.assertTrue(self.mock._scenario_completed)

    def test_mission_auto_completes_after_fifty_active_ticks(self):
        # 49에서 시작해, 이번 호출의 +1로 임계값(50)에 닿는지 확인한다.
        self.mock._scenario_active_ticks = 49

        self.mock._update_mission()

        snapshot = self.state.snapshot()
        self.assertEqual(snapshot["mission"]["status"], "COMPLETED")
        self.assertTrue(self.mock._scenario_completed)
        self.assertEqual(snapshot["robots"][0]["state"], "COMPLETED")
        self.assertEqual(self.mock._scenario_holds["robot4"], "COMPLETED")

    def test_mission_falls_back_to_ready_for_unproducible_states(self):
        """명령 API가 삭제되며 로봇을 PAUSED로 만들 방법 자체가 없어져,
        ``_update_mission``의 PAUSED 전용 분기를 제거했다(정리 완료). 그
        분기가 없어도 활동·완료가 아닌 나머지는 전부 READY로 떨어지는
        기본 동작만 남았다는 걸 확인한다.
        """

        self.state.update_robot("robot4", state="PAUSED")

        self.mock._update_mission()

        self.assertEqual(self.state.snapshot()["mission"]["status"], "READY")


if __name__ == "__main__":
    unittest.main()

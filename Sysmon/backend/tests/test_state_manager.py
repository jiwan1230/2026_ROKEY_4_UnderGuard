import time
import unittest

from system_monitor.state_manager import StateManager


class StateManagerTest(unittest.TestCase):
    def setUp(self):
        self.state = StateManager([("robot4", "RAT_TRACKER")], offline_timeout_sec=100)

    def test_update_and_snapshot(self):
        self.state.update_robot(
            "robot4",
            state="TRACKING",
            battery=72,
            position={"x": 1.2, "y": 3.4, "yaw": 0.5},
        )
        robot = self.state.snapshot()["robots"][0]
        self.assertEqual(robot["connection"], "ONLINE")
        self.assertEqual(robot["state"], "TRACKING")
        self.assertAlmostEqual(robot["position"]["x"], 1.2)

    def test_invalid_state(self):
        with self.assertRaises(ValueError):
            self.state.update_robot("robot4", state="UNKNOWN_STATE")

    def test_summary_only_reports_connection_counts(self):
        self.assertEqual(
            self.state.snapshot()["summary"],
            {"robots_online": 0, "robots_total": 1},
        )

        self.state.update_robot("robot4", state="IDLE")
        self.assertEqual(
            self.state.snapshot()["summary"],
            {"robots_online": 1, "robots_total": 1},
        )

    def test_first_rat_detector_gets_tracker_role_once(self):
        state = StateManager(
            [("robot4", "SCOUT"), ("robot5", "SCOUT")],
            offline_timeout_sec=100,
        )
        state.update_robot("robot4", state="SEARCHING")
        state.update_robot("robot5", state="SEARCHING")

        assignment = state.assign_roles_from_rat_detection("robot5")
        # 반환 타입은 dict | None(이미 배정된 뒤 재호출 시 None)이라, 여기서는
        # 최초 호출이라 dict가 온다는 걸 타입 체커에게도 명시적으로 알려준다.
        assert assignment is not None
        snapshot = state.snapshot()
        robots = {robot["robot_id"]: robot for robot in snapshot["robots"]}
        mission = snapshot["mission"]

        self.assertEqual(assignment["tracker_robot_id"], "robot5")
        self.assertEqual(robots["robot5"]["role"], "RAT_TRACKER")
        self.assertEqual(robots["robot5"]["state"], "TRACKING")
        self.assertEqual(robots["robot4"]["role"], "SURVEY_TRAP")
        self.assertEqual(robots["robot4"]["state"], "SEARCHING")
        self.assertEqual(mission["role_assignment_status"], "ASSIGNED")
        self.assertEqual(mission["tracker_robot_id"], "robot5")

        self.assertIsNone(state.assign_roles_from_rat_detection("robot4"))
        robots = {robot["robot_id"]: robot for robot in state.snapshot()["robots"]}
        self.assertEqual(robots["robot5"]["role"], "RAT_TRACKER")
        self.assertEqual(robots["robot4"]["role"], "SURVEY_TRAP")

    def test_robot_recovers_after_reconnecting_past_offline_timeout(self):
        """오프라인 타임아웃을 넘긴 로봇이 새 갱신을 받으면 자동 복구되는지 확인한다."""
        state = StateManager([("robot4", "SCOUT")], offline_timeout_sec=0.05)
        state.update_robot("robot4", state="SEARCHING")
        self.assertEqual(state.snapshot()["robots"][0]["connection"], "ONLINE")

        # 실제로 시간이 흐르길 기다리는 대신, 마지막 갱신 시각을 과거로 돌려
        # "메시지가 끊긴 지 오래됐다"를 재현한다.
        state._robots["robot4"].last_update = time.time() - 1.0

        offline = state.snapshot()["robots"][0]
        self.assertEqual(offline["connection"], "OFFLINE")
        self.assertEqual(offline["state"], "OFFLINE")
        self.assertEqual(offline["speed"], 0.0)

        # 재접속: 아무 갱신(하트비트 포함)이나 들어오면 자동 복구되어야 한다.
        state.mark_heartbeat("robot4")

        recovered = state.snapshot()["robots"][0]
        self.assertEqual(recovered["connection"], "ONLINE")
        self.assertEqual(recovered["state"], "IDLE")

    def test_live_items_receive_session_local_ids(self):
        first = self.state.add_detection(
            {"robot_id": "robot4", "object_type": "LIVE_RODENT"}
        )
        second = self.state.add_detection(
            {"robot_id": "robot4", "object_type": "ENTRY_POINT"}
        )
        trap = self.state.add_trap(
            {"robot_id": "robot4", "map_x": 1.2, "map_y": 3.4}
        )

        self.assertEqual((first["id"], second["id"]), (1, 2))
        self.assertEqual(trap["id"], 1)


if __name__ == "__main__":
    unittest.main()

import tempfile
import unittest
from pathlib import Path

from system_monitor.database import Database
from system_monitor.mock_manager import MockManager
from system_monitor.state_manager import StateManager


class MockManagerTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.db = Database(Path(self.temp.name) / "test.db")
        self.db.initialize()
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
        self.mock = MockManager(self.state, self.db, ["robot4"])

    def tearDown(self):
        self.temp.cleanup()

    def test_matching_object_uses_current_target_coordinates(self):
        detection = self.mock.trigger("RAT_HOLE_DETECTED", "robot4")
        self.assertEqual((detection["map_x"], detection["map_y"]), (4.0, 4.0))

    def test_different_object_gets_independent_coordinates(self):
        detection = self.mock.trigger("DROPPINGS_DETECTED", "robot4")
        self.assertEqual((detection["map_x"], detection["map_y"]), (1.3, 0.65))
        self.assertNotEqual((detection["map_x"], detection["map_y"]), (4.0, 4.0))

    def test_rat_detection_assigns_roles_from_detector(self):
        state = StateManager(
            [("robot4", "SCOUT"), ("robot5", "SCOUT")],
            offline_timeout_sec=100,
        )
        state.update_robot("robot4", state="SEARCHING", position={"x": 1.0, "y": 1.0})
        state.update_robot("robot5", state="SEARCHING", position={"x": 2.0, "y": 2.0})
        mock = MockManager(state, self.db, ["robot4", "robot5"])

        mock.trigger("RAT_DETECTED", "robot5")
        snapshot = state.snapshot()
        robots = {robot["robot_id"]: robot for robot in snapshot["robots"]}

        self.assertEqual(snapshot["mission"]["tracker_robot_id"], "robot5")
        self.assertEqual(robots["robot5"]["role"], "RAT_TRACKER")
        self.assertEqual(robots["robot4"]["role"], "SURVEY_TRAP")
        self.assertEqual(snapshot["events"][0]["event_type"], "DETECTION")
        self.assertEqual(snapshot["events"][1]["event_type"], "ROLE_ASSIGNED")

        mock.trigger("RAT_DETECTED", "robot4")
        robots = {robot["robot_id"]: robot for robot in state.snapshot()["robots"]}
        self.assertEqual(robots["robot5"]["role"], "RAT_TRACKER")
        self.assertEqual(robots["robot4"]["role"], "SURVEY_TRAP")

    def test_mock_events_use_canonical_risk_signal_names(self):
        expected = {
            "RAT_DETECTED": "LIVE_RODENT",
            "RAT_HOLE_DETECTED": "ENTRY_POINT",
            "DROPPINGS_DETECTED": "DROPPINGS",
        }
        for event_type, object_type in expected.items():
            item = self.mock.trigger(event_type, "robot4")
            self.assertEqual(item["object_type"], object_type)

    def test_trap_event_records_current_map_position(self):
        event = self.mock.trigger("TRAP_INSTALLED", "robot4")

        trap = self.state.snapshot()["traps"][0]

        self.assertEqual(event["event_type"], "TRAP_INSTALLED")
        self.assertEqual((trap["map_x"], trap["map_y"]), (1.0, 1.0))
        self.assertEqual(self.db.search_traps()[0]["id"], trap["id"])

    def test_trap_event_rejects_non_map_coordinates(self):
        self.state.update_robot("robot4", position_frame="odom")

        with self.assertRaisesRegex(ValueError, "map frame"):
            self.mock.trigger("TRAP_INSTALLED", "robot4")

        self.assertEqual(self.state.snapshot()["traps"], [])
        self.assertEqual(self.db.search_traps(), [])

    def test_scheduled_detection_does_not_reopen_completed_mission(self):
        self.mock._mission_progress = 100
        self.state.update_robot(
            "robot4",
            state="COMPLETED",
            current_task="임무 완료 · 다음 지시 대기",
        )
        self.mock._tick = 5

        self.mock._trigger_scheduled_detections()

        snapshot = self.state.snapshot()
        self.assertEqual(snapshot["robots"][0]["state"], "COMPLETED")
        self.assertEqual(snapshot["detections"], [])


if __name__ == "__main__":
    unittest.main()

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

    def test_command(self):
        event = self.state.apply_command("robot4", "RETURN_HOME")
        self.assertEqual(event["event_type"], "COMMAND")
        self.assertEqual(self.state.snapshot()["robots"][0]["state"], "RETURNING")

    def test_active_alerts_follow_current_robot_state(self):
        self.state.update_robot("robot4", state="TARGET_LOST", battery=70)
        self.assertEqual(self.state.snapshot()["summary"]["active_alerts"], 1)

        self.state.update_robot("robot4", state="TRACKING", battery=70)
        self.assertEqual(self.state.snapshot()["summary"]["active_alerts"], 0)

        self.state.update_robot("robot4", battery=14)
        self.assertEqual(self.state.snapshot()["summary"]["active_alerts"], 1)

    def test_first_rat_detector_gets_tracker_role_once(self):
        state = StateManager(
            [("robot4", "SCOUT"), ("robot5", "SCOUT")],
            offline_timeout_sec=100,
        )
        state.update_robot("robot4", state="SEARCHING")
        state.update_robot("robot5", state="SEARCHING")

        assignment = state.assign_roles_from_rat_detection("robot5")
        robots = {robot["robot_id"]: robot for robot in state.snapshot()["robots"]}
        mission = state.snapshot()["mission"]

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

    def test_role_specific_start_command_is_validated(self):
        state = StateManager(
            [("robot4", "SCOUT"), ("robot5", "SCOUT")],
            offline_timeout_sec=100,
        )
        state.update_robot("robot4", state="SEARCHING")
        state.update_robot("robot5", state="SEARCHING")
        state.assign_roles_from_rat_detection("robot5")

        with self.assertRaises(ValueError):
            state.apply_command("robot4", "START_TRACKING")
        state.apply_command("robot4", "START_SEARCH")
        state.apply_command("robot5", "START_TRACKING")

    def test_reset_restores_initial_roles_and_clears_runtime_state(self):
        self.state.update_robot("robot4", state="TRACKING", battery=75)
        self.state.add_detection({"robot_id": "robot4", "object_type": "LIVE_RODENT"})
        self.state.add_event("탐지", robot_id="robot4", event_type="DETECTION")
        self.state.add_trap({"robot_id": "robot4", "map_x": 1.0, "map_y": 2.0})
        self.state.set_mission(status="RUNNING", progress=60)

        self.state.reset()
        snapshot = self.state.snapshot()

        self.assertEqual(snapshot["robots"][0]["role"], "RAT_TRACKER")
        self.assertEqual(snapshot["robots"][0]["state"], "OFFLINE")
        self.assertEqual(snapshot["mission"]["status"], "READY")
        self.assertEqual(snapshot["mission"]["progress"], 0)
        self.assertEqual(snapshot["events"], [])
        self.assertEqual(snapshot["detections"], [])
        self.assertEqual(snapshot["traps"], [])

    def test_clear_operational_history_preserves_live_robot_and_mission(self):
        self.state.update_robot(
            "robot4",
            state="TRACKING",
            battery=75,
            position={"x": 1.2, "y": 3.4},
        )
        self.state.add_detection(
            {"robot_id": "robot4", "object_type": "LIVE_RODENT"}
        )
        self.state.add_event("탐지", robot_id="robot4", event_type="DETECTION")
        self.state.add_trap(
            {"robot_id": "robot4", "map_x": 1.2, "map_y": 3.4}
        )
        self.state.set_mission(status="RUNNING", progress=40)

        self.state.clear_operational_history()
        snapshot = self.state.snapshot()

        self.assertEqual(snapshot["events"], [])
        self.assertEqual(snapshot["detections"], [])
        self.assertEqual(snapshot["traps"], [])
        self.assertEqual(snapshot["robots"][0]["state"], "TRACKING")
        self.assertEqual(snapshot["robots"][0]["position"]["x"], 1.2)
        self.assertEqual(snapshot["mission"]["status"], "RUNNING")
        self.assertEqual(snapshot["mission"]["progress"], 40)


if __name__ == "__main__":
    unittest.main()

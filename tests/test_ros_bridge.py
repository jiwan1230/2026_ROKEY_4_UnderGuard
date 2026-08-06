import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from system_monitor.config import RobotConfig, RosInterfaceConfig
from system_monitor.database import Database
from system_monitor.ros_bridge import RosBridge
from system_monitor.state_manager import StateManager


def detection_array(frame_id: str, class_id: str = "rc_car"):
    hypothesis = SimpleNamespace(class_id=class_id, score=0.92)
    result = SimpleNamespace(hypothesis=hypothesis)
    position = SimpleNamespace(x=1.5, y=2.0, z=2.5)
    bbox = SimpleNamespace(center=SimpleNamespace(position=position))
    detection = SimpleNamespace(results=[result], bbox=bbox)
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        detections=[detection],
    )


def odometry(frame_id: str):
    """ROS 설치 없이 odom 콜백의 좌표계 보존을 검증하는 최소 메시지다."""

    pose = SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=1.25, y=2.5),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )
    twist = SimpleNamespace(
        twist=SimpleNamespace(linear=SimpleNamespace(x=0.1, y=0.0))
    )
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        pose=pose,
        twist=twist,
    )


class RosBridgeTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.db = Database(Path(self.temp.name) / "test.db")
        self.db.initialize()
        self.state = StateManager([("robot4", "SCOUT")], offline_timeout_sec=100)
        robots = (RobotConfig("robot4", "SCOUT"),)
        self.bridge = RosBridge(self.state, self.db, robots)

    def tearDown(self):
        self.temp.cleanup()

    def test_map_frame_detection_keeps_map_coordinates(self):
        self.bridge._on_detection("robot4", "OAK-D", detection_array("map"))

        row = self.db.search_detections()[0]
        robot = self.state.get_robot("robot4")
        self.assertEqual((row["map_x"], row["map_y"]), (1.5, 2.0))
        self.assertEqual(row["object_type"], "LIVE_RODENT")
        self.assertEqual(robot["state"], "TRACKING")
        self.assertEqual(robot["role"], "RAT_TRACKER")
        self.assertEqual(self.state.snapshot()["mission"]["status"], "RUNNING")

    def test_configured_map_frame_is_used_for_detection_coordinates(self):
        self.bridge.interface = RosInterfaceConfig(map_frame="robot4/map")

        self.bridge._on_detection(
            "robot4",
            "OAK-D",
            detection_array("/robot4/map", class_id="rat_hole"),
        )

        row = self.db.search_detections()[0]
        self.assertEqual((row["map_x"], row["map_y"]), (1.5, 2.0))
        self.assertEqual(row["object_type"], "ENTRY_POINT")

    def test_robot_topic_supports_suffix_absolute_and_template_names(self):
        robot = RobotConfig("/robot4", "SCOUT")

        self.assertEqual(robot.topic("odom"), "/robot4/odom")
        self.assertEqual(robot.topic("/fleet/status"), "/fleet/status")
        self.assertEqual(
            robot.topic("/{namespace}/odometry/filtered"),
            "/robot4/odometry/filtered",
        )

    def test_sensor_frame_is_not_mislabeled_as_map_coordinates(self):
        self.bridge._on_detection("robot4", "WEBCAM", detection_array("camera_link"))

        row = self.db.search_detections()[0]
        self.assertIsNone(row["map_x"])
        self.assertIsNone(row["map_y"])
        self.assertEqual(row["distance"], 2.5)

    def test_odometry_preserves_source_coordinate_frame(self):
        self.bridge._on_odom("robot4", odometry("odom"))

        robot = self.state.get_robot("robot4")
        self.assertEqual(robot["position_frame"], "odom")
        self.assertEqual((robot["position"]["x"], robot["position"]["y"]), (1.25, 2.5))
        self.assertEqual(robot["nav_status"], "MOVING")

    def test_oakd_timeout_records_target_lost_once_and_keeps_position(self):
        self.bridge._on_detection("robot4", "OAK-D", detection_array("map"))
        before = self.state.get_robot("robot4")["target"]
        self.bridge._last_live_rodent_at["robot4"] = time.monotonic() - 2.0

        self.bridge._check_target_timeouts()
        self.bridge._check_target_timeouts()

        robot = self.state.get_robot("robot4")
        self.assertEqual(robot["state"], "TARGET_LOST")
        self.assertEqual(robot["target"], before)
        with self.db.connect() as conn:
            count = conn.execute(
                "SELECT COUNT(*) FROM system_events WHERE event_type = 'TARGET_LOST'"
            ).fetchone()[0]
        self.assertEqual(count, 1)

    def test_low_battery_warning_is_recorded_once_until_recovery(self):
        low = SimpleNamespace(percentage=0.14)
        normal = SimpleNamespace(percentage=0.20)

        self.bridge._on_battery("robot4", low)
        self.bridge._on_battery("robot4", low)
        self.bridge._on_battery("robot4", normal)
        self.bridge._on_battery("robot4", low)

        with self.db.connect() as conn:
            count = conn.execute(
                "SELECT COUNT(*) FROM system_events WHERE event_type = 'LOW_BATTERY'"
            ).fetchone()[0]
        self.assertEqual(count, 2)
        self.assertEqual(self.state.get_robot("robot4")["battery"], 14.0)

    def test_main_fleet_status_is_mapped_to_dashboard_state(self):
        self.bridge._on_fleet_status(SimpleNamespace(data="robot4:PATROLLING:85"))

        robot = self.state.get_robot("robot4")
        self.assertEqual(robot["state"], "SEARCHING")
        self.assertEqual(robot["current_task"], "창고 순찰 중")
        self.assertEqual(robot["battery"], 85.0)
        self.assertEqual(robot["nav_status"], "MOVING")
        self.assertEqual(self.state.snapshot()["mission"]["status"], "RUNNING")

        self.bridge._on_fleet_status(SimpleNamespace(data="robot4:DOCKED:84"))
        robot = self.state.get_robot("robot4")
        self.assertEqual(robot["state"], "COMPLETED")
        self.assertEqual(robot["nav_status"], "STOPPED")

    def test_malformed_or_unregistered_fleet_status_is_ignored(self):
        self.bridge._on_fleet_status(SimpleNamespace(data="invalid"))
        self.bridge._on_fleet_status(SimpleNamespace(data="robot6:IDLE:100"))

        self.assertEqual(self.state.get_robot("robot4")["state"], "OFFLINE")

    def test_main_fleet_events_create_detection_and_trap_records(self):
        self.bridge._on_fleet_status(SimpleNamespace(data="robot4:PATROLLING:90"))
        self.bridge._on_fleet_event(
            SimpleNamespace(data="rat_detected:1.20:3.40")
        )
        self.bridge._on_fleet_event(SimpleNamespace(data="trap_ok:2.00:4.00"))

        detection = self.db.search_detections()[0]
        trap = self.db.search_traps()[0]
        self.assertEqual(detection["object_type"], "LIVE_RODENT")
        self.assertEqual(detection["robot_id"], "robot4")
        self.assertEqual((detection["map_x"], detection["map_y"]), (1.2, 3.4))
        self.assertEqual((trap["map_x"], trap["map_y"]), (2.0, 4.0))

    def test_main_fleet_command_uses_shared_colon_format(self):
        class FleetString:
            def __init__(self, *, data):
                self.data = data

        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, msg):
                self.messages.append(msg.data)

        publisher = Publisher()
        self.bridge._command_publisher = publisher
        with patch("system_monitor.ros_bridge.String", FleetString):
            result = self.bridge.send_command("robot4", "START_SCOUTING")

        self.assertTrue(result["accepted"])
        self.assertEqual(publisher.messages, ["robot4:PATROL"])
        unsupported = self.bridge.send_command("robot4", "PAUSE")
        self.assertFalse(unsupported["accepted"])
        self.assertIn("지원하지 않는", unsupported["reason"])


if __name__ == "__main__":
    unittest.main()

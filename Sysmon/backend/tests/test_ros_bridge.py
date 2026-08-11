import time
import unittest
from types import SimpleNamespace

from system_monitor.config import RobotConfig, RosInterfaceConfig
from system_monitor.ros_bridge import RosBridge
from system_monitor.state_manager import StateManager


def detection_array(frame_id: str, class_id: str = "rc_car"):
    hypothesis = SimpleNamespace(class_id=class_id, score=0.92)
    result = SimpleNamespace(hypothesis=hypothesis)
    position = SimpleNamespace(x=1.5, y=2.0, z=2.5)
    bbox = SimpleNamespace(center=SimpleNamespace(position=position))
    detection = SimpleNamespace(results=[result], bbox=bbox)
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id), detections=[detection]
    )


def odometry(frame_id: str):
    """ROS 설치 없이 odom 콜백을 검증하는 최소 메시지를 만든다."""

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
        header=SimpleNamespace(frame_id=frame_id), pose=pose, twist=twist
    )


class RosBridgeTest(unittest.TestCase):
    def setUp(self):
        self.state = StateManager([("robot4", "SCOUT")], offline_timeout_sec=100)
        self.bridge = RosBridge(
            self.state, (RobotConfig("robot4", "SCOUT"),)
        )

    def _event_count(self, event_type):
        return sum(
            event["event_type"] == event_type
            for event in self.state.snapshot()["events"]
        )

    def test_map_frame_detection_keeps_map_coordinates(self):
        self.bridge._on_detection("robot4", "OAK-D", detection_array("map"))

        row = self.state.snapshot()["detections"][0]
        robot = self.state.get_robot("robot4")
        self.assertEqual((row["map_x"], row["map_y"]), (1.5, 2.0))
        self.assertEqual(row["object_type"], "LIVE_RODENT")
        self.assertEqual(robot["state"], "TRACKING")
        self.assertEqual(robot["role"], "RAT_TRACKER")

    def test_configured_map_frame_is_used_for_detection_coordinates(self):
        self.bridge.interface = RosInterfaceConfig(map_frame="robot4/map")
        self.bridge._on_detection(
            "robot4", "OAK-D", detection_array("/robot4/map", "rat_hole")
        )

        row = self.state.snapshot()["detections"][0]
        self.assertEqual((row["map_x"], row["map_y"]), (1.5, 2.0))
        self.assertEqual(row["object_type"], "ENTRY_POINT")

    def test_sensor_frame_is_not_mislabeled_as_map_coordinates(self):
        self.bridge._on_detection("robot4", "WEBCAM", detection_array("camera_link"))
        row = self.state.snapshot()["detections"][0]

        self.assertIsNone(row["map_x"])
        self.assertIsNone(row["map_y"])

    def test_camera_frame_is_cached_without_reencoding(self):
        self.bridge._on_camera_frame(
            "robot4", SimpleNamespace(data=b"raw-jpeg-bytes", format="jpeg")
        )

        frame = self.bridge.camera_frame_store.get("robot4")
        self.assertEqual(frame.content, b"raw-jpeg-bytes")
        self.assertEqual(frame.format, "jpeg")

    def test_detection_carries_image_url_once_a_frame_is_cached(self):
        self.bridge._on_detection("robot4", "OAK-D", detection_array("map"))
        row = self.state.snapshot()["detections"][0]
        self.assertIsNone(row["image_url"])

        self.bridge._on_camera_frame(
            "robot4", SimpleNamespace(data=b"raw-jpeg-bytes", format="jpeg")
        )
        self.bridge._on_detection("robot4", "OAK-D", detection_array("map"))
        row = self.state.snapshot()["detections"][0]
        self.assertEqual(row["image_url"], "/api/camera/robot4/frame")
        self.assertEqual(row["distance"], 2.5)

    def test_robot_topic_supports_suffix_absolute_and_template_names(self):
        robot = RobotConfig("/robot4", "SCOUT")
        self.assertEqual(robot.topic("odom"), "/robot4/odom")
        self.assertEqual(robot.topic("/fleet/status"), "/fleet/status")
        self.assertEqual(
            robot.topic("/{namespace}/odometry/filtered"),
            "/robot4/odometry/filtered",
        )

    def test_odometry_preserves_source_coordinate_frame(self):
        self.bridge._on_odom("robot4", odometry("odom"))
        robot = self.state.get_robot("robot4")

        self.assertEqual(robot["position_frame"], "odom")
        self.assertEqual((robot["position"]["x"], robot["position"]["y"]), (1.25, 2.5))
        self.assertEqual(robot["nav_status"], "MOVING")

    def test_oakd_timeout_reports_target_lost_once(self):
        self.bridge._on_detection("robot4", "OAK-D", detection_array("map"))
        before = self.state.get_robot("robot4")["target"]
        self.bridge._last_live_rodent_at["robot4"] = time.monotonic() - 2.0

        self.bridge._check_target_timeouts()
        self.bridge._check_target_timeouts()

        self.assertEqual(self.state.get_robot("robot4")["state"], "TARGET_LOST")
        self.assertEqual(self.state.get_robot("robot4")["target"], before)
        self.assertEqual(self._event_count("TARGET_LOST"), 1)

    def test_low_battery_warning_is_reported_once_until_recovery(self):
        low = SimpleNamespace(percentage=0.14)
        normal = SimpleNamespace(percentage=0.20)
        self.bridge._on_battery("robot4", low)
        self.bridge._on_battery("robot4", low)
        self.bridge._on_battery("robot4", normal)
        self.bridge._on_battery("robot4", low)

        self.assertEqual(self._event_count("LOW_BATTERY"), 2)
        self.assertEqual(self.state.get_robot("robot4")["battery"], 14.0)

    def test_main_fleet_status_is_mapped_to_dashboard_state(self):
        self.bridge._on_fleet_status(SimpleNamespace(data="robot4:PATROLLING:85"))
        robot = self.state.get_robot("robot4")

        self.assertEqual(robot["state"], "SEARCHING")
        self.assertEqual(robot["current_task"], "창고 순찰 중")
        self.assertEqual(robot["battery"], 85.0)
        # 전체 임무 상태는 로봇 state로부터 매번 계산된다. SEARCHING은
        # "주변 위험요소 확인 중"(VERIFYING) 우선순위 버킷에 속한다.
        self.assertEqual(self.state.snapshot()["mission"]["status"], "VERIFYING")

    def test_robot_recovers_when_fleet_status_resumes_after_gap(self):
        """일정 시간 /fleet/status가 끊겼다가 다시 오면 자동 복구되는지 확인한다."""
        self.bridge._on_fleet_status(SimpleNamespace(data="robot4:PATROLLING:80"))
        self.assertEqual(self.state.get_robot("robot4")["connection"], "ONLINE")

        # setUp의 offline_timeout_sec=100을 확실히 넘기도록 충분히 되돌린다.
        self.state._robots["robot4"].last_update = time.time() - 200.0
        self.assertEqual(
            self.state.snapshot()["robots"][0]["connection"], "OFFLINE"
        )

        self.bridge._on_fleet_status(SimpleNamespace(data="robot4:PATROLLING:79"))

        recovered = self.state.get_robot("robot4")
        self.assertEqual(recovered["connection"], "ONLINE")
        self.assertEqual(recovered["state"], "SEARCHING")
        self.assertEqual(recovered["battery"], 79.0)

    def test_malformed_or_unregistered_fleet_status_is_ignored(self):
        self.bridge._on_fleet_status(SimpleNamespace(data="invalid"))
        self.bridge._on_fleet_status(SimpleNamespace(data="robot6:IDLE:100"))
        self.assertEqual(self.state.get_robot("robot4")["state"], "OFFLINE")

    def test_main_fleet_events_create_live_detection_and_trap_markers(self):
        self.bridge._on_fleet_status(SimpleNamespace(data="robot4:PATROLLING:90"))
        self.bridge._on_fleet_event(SimpleNamespace(data="rat_detected:1.20:3.40"))
        self.bridge._on_fleet_event(SimpleNamespace(data="trap_ok:2.00:4.00"))

        snapshot = self.state.snapshot()
        detection = snapshot["detections"][0]
        trap = snapshot["traps"][0]
        self.assertEqual(detection["object_type"], "LIVE_RODENT")
        self.assertEqual((detection["map_x"], detection["map_y"]), (1.2, 3.4))
        self.assertEqual((trap["map_x"], trap["map_y"]), (2.0, 4.0))

if __name__ == "__main__":
    unittest.main()

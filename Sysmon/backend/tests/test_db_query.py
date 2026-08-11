"""로봇 DB(db_node) 기록 조회 연동 — ROS 없이도 도는 부분만 검증한다.

실제 조회 결과는 db_node·MySQL이 떠 있어야 나오므로, 여기서는 계약과
장애 격리(“DB가 없어도 실시간 화면은 그대로”)를 확인한다.
"""

import tempfile
import unittest
from pathlib import Path

from system_monitor.app import create_app
from system_monitor.config import RobotConfig, RosInterfaceConfig, Settings
from system_monitor.ros_bridge import RosBridge
from system_monitor.state_manager import StateManager


class DbQueryRouteTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        (root / "my_map.pgm").write_bytes(b"P5\n2 2\n255\n" + bytes([0, 100, 205, 255]))
        map_yaml = root / "my_map.yaml"
        map_yaml.write_text(
            "image: my_map.pgm\nresolution: 0.5\norigin: [0, 0, 0]\n", encoding="utf-8"
        )
        self.app = create_app(
            Settings(
                mode="mock",
                robots=(RobotConfig("robot4", "SCOUT"),),
                offline_timeout_sec=100,
                poll_interval_ms=1000,
                map_yaml_path=map_yaml,
                history_db_path=root / "history.db",
                history_image_dir=root / "captures",
            )
        )

    def tearDown(self):
        self.app.extensions["mock_manager"].stop()
        self.temp.cleanup()

    def test_only_the_four_agreed_queries_are_exposed(self):
        client = self.app.test_client()

        response = client.get("/api/db/nope")
        self.assertEqual(response.status_code, 404)
        self.assertEqual(
            response.get_json()["allowed"],
            ["detections", "missions", "report", "traps"],
        )

    def test_write_routes_do_not_exist(self):
        """조회 전용 계약 — DB를 고치는 경로는 UI에 열려 있으면 안 된다."""

        client = self.app.test_client()

        self.assertEqual(client.post("/api/db/detections", json={}).status_code, 405)
        self.assertEqual(client.delete("/api/db/detections").status_code, 405)

    def test_db_outage_does_not_break_live_screen(self):
        """db_node가 없어도 조회만 503이고 실시간·기록 화면은 그대로 떠야 한다."""

        client = self.app.test_client()

        self.assertEqual(client.get("/api/db/report").status_code, 503)
        self.assertEqual(client.get("/api/snapshot").status_code, 200)
        self.assertEqual(client.get("/api/history/summary").status_code, 200)

    def test_bad_limit_is_rejected_before_reaching_ros(self):
        client = self.app.test_client()

        response = client.get("/api/db/detections?limit=abc")
        self.assertEqual(response.status_code, 400)


class DetectionMetaTest(unittest.TestCase):
    """/fleet/detection이 붙으면 robot_id를 추측하지 않는지 확인한다."""

    def _bridge(self):
        robots = (RobotConfig("robot4", "SCOUT"), RobotConfig("robot6", "SCOUT"))
        state = StateManager([(r.robot_id, r.role) for r in robots])
        return RosBridge(state, robots, interface=RosInterfaceConfig())

    def test_falls_back_to_guess_without_detection_topic(self):
        bridge = self._bridge()

        robot_id, confidence = bridge._detection_meta("rat_detected")

        self.assertEqual(robot_id, "robot4")   # 설정된 첫 로봇 = 기존 추측 동작
        self.assertIsNone(confidence)

    def test_uses_real_robot_id_and_confidence_when_available(self):
        bridge = self._bridge()

        bridge._on_fleet_detection(
            _DetectionMsg(object_type="RAT", robot_id="robot6", confidence=0.91)
        )
        robot_id, confidence = bridge._detection_meta("rat_detected")

        self.assertEqual(robot_id, "robot6")
        self.assertAlmostEqual(confidence, 0.91)

    def test_opening_and_rat_are_tracked_separately(self):
        bridge = self._bridge()

        bridge._on_fleet_detection(
            _DetectionMsg(object_type="RAT", robot_id="robot6", confidence=0.91)
        )
        bridge._on_fleet_detection(
            _DetectionMsg(object_type="OPENING", robot_id="robot4", confidence=0.72)
        )

        self.assertEqual(bridge._detection_meta("rat_detected")[0], "robot6")
        self.assertEqual(bridge._detection_meta("opening_confirmed")[0], "robot4")

    def test_malformed_message_is_ignored(self):
        bridge = self._bridge()

        bridge._on_fleet_detection(object())        # 필드가 없는 값
        bridge._on_fleet_detection(
            _DetectionMsg(object_type="RAT", robot_id="", confidence=0.5)
        )

        self.assertEqual(bridge._detection_meta("rat_detected"), ("robot4", None))


class _DetectionMsg:
    """turtle_interfaces/DetectionEvent 대역 — ROS 없이 콜백만 돌리기 위한 것."""

    def __init__(self, object_type, robot_id, confidence):
        self.object_type = object_type
        self.robot_id = robot_id
        self.confidence = confidence
        self.x = 0.0
        self.y = 0.0


if __name__ == "__main__":
    unittest.main()

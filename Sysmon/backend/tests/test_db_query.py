"""로봇 DB(db_node) 기록 조회 연동 — ROS 없이도 도는 부분만 검증한다.

실제 조회 결과는 db_node·MySQL이 떠 있어야 나오므로, 여기서는 계약과
장애 격리(“DB가 없어도 실시간 화면은 그대로”)를 확인한다.
"""

import tempfile
import unittest
from datetime import datetime
from pathlib import Path

from system_monitor.app import create_app, history_detections_from_db
from system_monitor.config import RobotConfig, RosInterfaceConfig, Settings
from system_monitor.history_store import HistoryStore
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
        """db_node가 없으면 DB 조회만 503이고 실시간 화면은 그대로 떠야 한다.

        탐지 기록도 로봇 DB가 원본이라 같이 503이지만(화면이 이유를 표시한다),
        관제 서버가 직접 쌓는 이동 경로는 DB와 무관하게 계속 나와야 한다.
        """

        client = self.app.test_client()

        self.assertEqual(client.get("/api/db/report").status_code, 503)
        self.assertEqual(client.get("/api/snapshot").status_code, 200)
        self.assertEqual(client.get("/api/history/summary").status_code, 503)
        self.assertEqual(client.get("/api/history/trail").status_code, 200)

    def test_bad_limit_is_rejected_before_reaching_ros(self):
        client = self.app.test_client()

        response = client.get("/api/db/detections?limit=abc")
        self.assertEqual(response.status_code, 400)


class HistoryFromDbTest(unittest.TestCase):
    """로봇 DB 행 → 기록 조회 화면 모양 변환. db_node·MySQL 없이 검증한다."""

    ROWS = [
        {
            "detection_id": 1,
            "detected_at": "2026-08-12 09:00:00",
            "object_type": "RAT",
            "x": 1.0,
            "y": 2.0,
            "confidence": 0.9,
            "robot_id": "robot4",
        },
        {
            "detection_id": 2,
            "detected_at": "2026-08-12 10:00:00",
            "object_type": "OPENING",
            "x": 3.0,
            "y": 4.0,
            "confidence": 0.7,
            "robot_id": "robot6",
        },
    ]

    def test_db_labels_become_screen_labels(self):
        items = history_detections_from_db(self.ROWS)

        self.assertEqual([item["object_type"] for item in items],
                         ["LIVE_RODENT", "ENTRY_POINT"])
        self.assertEqual([item["id"] for item in items], [1, 2])
        self.assertEqual(items[0]["map_x"], 1.0)
        self.assertEqual(items[0]["map_y"], 2.0)

    def test_datetime_becomes_epoch_seconds(self):
        """화면 타임라인이 epoch 숫자로 계산하므로 문자열이면 안 된다."""

        items = history_detections_from_db(self.ROWS)

        expected = datetime(2026, 8, 12, 9, 0, 0).timestamp()
        self.assertAlmostEqual(items[0]["timestamp"], expected)
        self.assertAlmostEqual(items[1]["timestamp"] - items[0]["timestamp"], 3600)

    def test_filters_are_applied_here_because_db_only_takes_limit(self):
        by_kind = history_detections_from_db(self.ROWS, object_type="ENTRY_POINT")
        by_robot = history_detections_from_db(self.ROWS, robot_id="robot4")
        by_time = history_detections_from_db(
            self.ROWS, since=datetime(2026, 8, 12, 9, 30, 0).timestamp()
        )

        self.assertEqual([item["id"] for item in by_kind], [2])
        self.assertEqual([item["id"] for item in by_robot], [1])
        self.assertEqual([item["id"] for item in by_time], [2])

    def test_unreadable_timestamp_survives_but_drops_out_of_time_filter(self):
        rows = [dict(self.ROWS[0], detected_at=None)]

        self.assertIsNone(history_detections_from_db(rows)[0]["timestamp"])
        self.assertEqual(history_detections_from_db(rows, since=0), [])


class TrailRecordingTest(unittest.TestCase):
    """이동 경로는 로봇 DB에 없어서 관제 서버가 amcl_pose를 받아 직접 쌓는다."""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.store = HistoryStore(root / "history.db", root / "captures")
        robots = (RobotConfig("robot4", "SCOUT"),)
        state = StateManager([(r.robot_id, r.role) for r in robots])
        self.bridge = RosBridge(state, robots, history_store=self.store)
        self.state = state

    def tearDown(self):
        self.temp.cleanup()

    def _pose(self, x, y, frame="map"):
        return _PoseMsg(x, y, frame)

    def test_amcl_pose_is_recorded_and_shown_in_map_frame(self):
        self.bridge._on_amcl_pose("robot4", self._pose(1.0, 2.0))

        trail = self.store.get_trail(robot_id="robot4")
        self.assertEqual(len(trail), 1)
        self.assertEqual((trail[0]["map_x"], trail[0]["map_y"]), (1.0, 2.0))
        robot = self.state.snapshot()["robots"][0]
        self.assertEqual(robot["position_frame"], "map")
        self.assertEqual(robot["position"]["x"], 1.0)

    def test_standing_still_does_not_fill_the_trail(self):
        self.bridge._on_amcl_pose("robot4", self._pose(1.0, 2.0))
        self.bridge._on_amcl_pose("robot4", self._pose(1.01, 2.01))   # 문턱 이내
        self.bridge._on_amcl_pose("robot4", self._pose(1.5, 2.0))     # 문턱 밖

        self.assertEqual(len(self.store.get_trail()), 2)

    def test_odom_no_longer_moves_a_robot_that_has_a_map_position(self):
        """odom은 원점이 달라 맵 위에서 어긋난다 — 지도가 두 값 사이로 튀면 안 된다."""

        self.bridge._on_amcl_pose("robot4", self._pose(1.0, 2.0))
        self.bridge._on_odom("robot4", _OdomMsg(9.0, 9.0))

        robot = self.state.snapshot()["robots"][0]
        self.assertEqual(robot["position"]["x"], 1.0)
        self.assertEqual(robot["position_frame"], "map")
        self.assertEqual(robot["nav_status"], "MOVING")     # 속도는 odom에서 온다

    def test_odom_still_positions_a_robot_without_amcl(self):
        self.bridge._on_odom("robot4", _OdomMsg(9.0, 9.0))

        robot = self.state.snapshot()["robots"][0]
        self.assertEqual(robot["position"]["x"], 9.0)
        self.assertEqual(robot["position_frame"], "odom")


class _PoseMsg:
    """geometry_msgs/PoseWithCovarianceStamped 대역."""

    def __init__(self, x, y, frame="map"):
        self.header = _Header(frame)
        self.pose = _Nested(_Pose(x, y))


class _OdomMsg:
    """nav_msgs/Odometry 대역 — 위치는 odom 프레임, 속도 포함."""

    def __init__(self, x, y):
        self.header = _Header("odom")
        self.pose = _Nested(_Pose(x, y))
        self.twist = _Nested(_Twist())


class _Header:
    def __init__(self, frame_id):
        self.frame_id = frame_id


class _Nested:
    def __init__(self, value):
        self.pose = value
        self.twist = value


class _Pose:
    def __init__(self, x, y):
        self.position = _Vec(x, y)
        self.orientation = _Quat()


class _Twist:
    def __init__(self):
        self.linear = _Vec(0.5, 0.0)


class _Vec:
    def __init__(self, x, y):
        self.x, self.y, self.z = x, y, 0.0


class _Quat:
    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.w = 1.0


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

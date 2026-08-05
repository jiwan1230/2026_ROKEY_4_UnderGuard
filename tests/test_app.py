import tempfile
import unittest
from pathlib import Path

from system_monitor.app import create_app, start_background_service
from system_monitor.config import RobotConfig, Settings


def payload_shape(value):
    """값은 제외하고 Mock/ROS JSON의 필드와 중첩 타입만 비교한다."""

    if isinstance(value, dict):
        return {key: payload_shape(item) for key, item in sorted(value.items())}
    if isinstance(value, list):
        return [payload_shape(value[0])] if value else []
    return type(value).__name__


class AppTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        map_image = root / "my_map.pgm"
        map_yaml = root / "my_map.yaml"
        map_image.write_bytes(b"P5\n2 2\n255\n" + bytes([0, 100, 205, 255]))
        map_yaml.write_text(
            "image: my_map.pgm\nresolution: 0.5\norigin: [0, 0, 0]\n",
            encoding="utf-8",
        )
        settings = Settings(
            mode="mock",
            secret_key="test-secret",
            database_path=Path(self.temp.name) / "test.db",
            robots=(RobotConfig("robot4", "SCOUT"),),
            offline_timeout_sec=100,
            poll_interval_ms=1000,
            map_yaml_path=map_yaml,
        )
        self.app = create_app(settings)

    def tearDown(self):
        self.app.extensions["mock_manager"].stop()
        self.temp.cleanup()

    def test_background_service_starts_explicitly_once(self):
        manager = self.app.extensions["mock_manager"]
        self.assertFalse(manager.running)

        start_background_service(self.app)
        start_background_service(self.app)

        self.assertTrue(manager.running)

    def test_login_and_authenticated_api(self):
        client = self.app.test_client()
        self.assertEqual(client.get("/api/health").status_code, 200)
        self.assertEqual(client.get("/api/snapshot").status_code, 401)

        response = client.post(
            "/login",
            data={"username": "admin", "password": "admin123"},
        )
        self.assertEqual(response.status_code, 302)
        self.assertEqual(response.headers["Location"], "/dashboard")
        self.assertEqual(client.get("/api/snapshot").status_code, 200)

        dashboard = client.get("/dashboard")
        self.assertIn("전체 임무 진행률".encode(), dashboard.data)
        self.assertIn("탐지 누적".encode(), dashboard.data)
        self.assertIn("미해결 경고".encode(), dashboard.data)
        self.assertIn("최근 이벤트".encode(), dashboard.data)
        self.assertIn(b'id="map-legend-card" class="map-legend-card hidden"', dashboard.data)
        self.assertIn(b'id="map-marker-detail"', dashboard.data)
        self.assertIn(b'id="stop-all-robots"', dashboard.data)
        self.assertIn(b'id="command-confirm-dialog"', dashboard.data)

        map_metadata = client.get("/api/map").get_json()
        self.assertTrue(map_metadata["available"])
        self.assertEqual((map_metadata["width"], map_metadata["height"]), (2, 2))
        map_image = client.get("/api/map/image")
        self.assertEqual(map_image.status_code, 200)
        self.assertEqual(map_image.mimetype, "image/png")

        detections_page = client.get("/detections")
        self.assertIn(b"js/detections.js", detections_page.data)
        static_response = client.get("/static/js/detections.js")
        self.assertEqual(static_response.status_code, 200)
        static_response.close()

    def test_mock_and_ros_expose_the_same_runtime_contract(self):
        mock_runtime = self.app.extensions["runtime_service"].status()
        ros_settings = Settings(
            mode="ros",
            secret_key="test-secret",
            database_path=Path(self.temp.name) / "ros-test.db",
            robots=(RobotConfig("robot4", "SCOUT"),),
            offline_timeout_sec=100,
            poll_interval_ms=1000,
            map_yaml_path=Path(self.temp.name) / "my_map.yaml",
        )
        ros_app = create_app(ros_settings)
        ros_runtime = ros_app.extensions["runtime_service"].status()

        self.assertEqual(set(mock_runtime), set(ros_runtime))
        self.assertTrue(mock_runtime["commands_enabled"])
        self.assertFalse(ros_runtime["commands_enabled"])

        mock_client = self.app.test_client()
        ros_client = ros_app.test_client()
        for client in (mock_client, ros_client):
            client.post("/login", data={"username": "admin", "password": "admin123"})

        mock_snapshot = mock_client.get("/api/snapshot").get_json()
        ros_snapshot = ros_client.get("/api/snapshot").get_json()
        self.assertEqual(set(ros_snapshot["runtime"]), set(mock_runtime))
        self.assertEqual(payload_shape(mock_snapshot), payload_shape(ros_snapshot))

        # 최근 UI 개선 요소는 두 모드가 같은 템플릿 계약을 사용해야 한다.
        mock_dashboard = mock_client.get("/dashboard").data
        ros_dashboard = ros_client.get("/dashboard").data
        common_ids = (
            b'id="open-event-dialog"',
            b'id="stop-all-robots"',
            b'id="command-confirm-dialog"',
            b'id="map-marker-detail"',
            b'id="camera-stack"',
            b'id="event-dialog"',
        )
        for element_id in common_ids:
            self.assertIn(element_id, mock_dashboard)
            self.assertIn(element_id, ros_dashboard)
        self.assertIn(b'id="mock-panel-toggle"', mock_dashboard)
        self.assertNotIn(b'id="mock-panel-toggle"', ros_dashboard)

        # 모드별 값은 달라도 공통 조회 API의 JSON 형태는 같아야 한다.
        for endpoint in ("/api/health", "/api/map", "/api/detections"):
            self.assertEqual(
                payload_shape(mock_client.get(endpoint).get_json()),
                payload_shape(ros_client.get(endpoint).get_json()),
            )

    def test_command_response_shape_is_shared_by_both_modes(self):
        mock = self.app.extensions["mock_manager"]
        mock_result = mock.send_command("robot4", "START_SCOUTING")

        ros_settings = Settings(
            mode="ros",
            secret_key="test-secret",
            database_path=Path(self.temp.name) / "ros-command.db",
            robots=(RobotConfig("robot4", "SCOUT"),),
            offline_timeout_sec=100,
            poll_interval_ms=1000,
            map_yaml_path=Path(self.temp.name) / "my_map.yaml",
        )
        ros = create_app(ros_settings).extensions["ros_bridge"]
        ros_result = ros.send_command("robot4", "START_SCOUTING")

        self.assertEqual(set(mock_result), set(ros_result))
        self.assertTrue(mock_result["accepted"])
        self.assertFalse(ros_result["accepted"])


if __name__ == "__main__":
    unittest.main()

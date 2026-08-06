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
        self.map_yaml = root / "my_map.yaml"
        map_image.write_bytes(b"P5\n2 2\n255\n" + bytes([0, 100, 205, 255]))
        self.map_yaml.write_text(
            "image: my_map.pgm\nresolution: 0.5\norigin: [0, 0, 0]\n",
            encoding="utf-8",
        )
        self.app = create_app(self._settings("mock"))

    def tearDown(self):
        self.app.extensions["mock_manager"].stop()
        self.temp.cleanup()

    def _settings(self, mode):
        return Settings(
            mode=mode,
            robots=(RobotConfig("robot4", "SCOUT"),),
            offline_timeout_sec=100,
            poll_interval_ms=1000,
            map_yaml_path=self.map_yaml,
        )

    def test_dashboard_and_live_apis_are_public(self):
        client = self.app.test_client()

        self.assertEqual(client.get("/").status_code, 200)
        self.assertEqual(client.get("/dashboard").status_code, 200)
        self.assertEqual(client.get("/api/health").status_code, 200)
        self.assertEqual(client.get("/api/snapshot").status_code, 200)
        self.assertEqual(client.get("/api/map").status_code, 200)
        self.assertEqual(client.get("/api/map/image").mimetype, "image/png")

        dashboard = client.get("/dashboard").data
        self.assertNotIn("탐지 DB".encode(), dashboard)
        self.assertNotIn("로그아웃".encode(), dashboard)
        self.assertIn("탐지 마커".encode(), dashboard)
        self.assertIn("현재 세션 이벤트".encode(), dashboard)
        self.assertIn(b'id="fleet-connection-state"', dashboard)
        self.assertNotIn(b'data-command=', dashboard)
        self.assertNotIn(b'id="stop-all-robots"', dashboard)
        self.assertNotIn(b'id="command-confirm-dialog"', dashboard)

    def test_removed_login_database_and_reset_routes_do_not_exist(self):
        client = self.app.test_client()

        self.assertEqual(client.get("/login").status_code, 404)
        self.assertEqual(client.post("/logout").status_code, 404)
        self.assertEqual(client.get("/detections").status_code, 404)
        self.assertEqual(client.get("/api/detections").status_code, 404)
        self.assertEqual(client.post("/api/commands", json={}).status_code, 404)
        self.assertEqual(
            client.post("/api/admin/reset-operational-data", json={}).status_code,
            404,
        )

    def test_background_service_starts_explicitly_once(self):
        manager = self.app.extensions["mock_manager"]
        self.assertFalse(manager.running)

        start_background_service(self.app)
        start_background_service(self.app)

        self.assertTrue(manager.running)

    def test_mock_and_ros_expose_the_same_live_contract(self):
        ros_app = create_app(self._settings("ros"))
        mock_runtime = self.app.extensions["runtime_service"].status()
        ros_runtime = ros_app.extensions["runtime_service"].status()

        self.assertEqual(set(mock_runtime), set(ros_runtime))
        self.assertTrue(mock_runtime["read_only"])
        self.assertTrue(ros_runtime["read_only"])
        for removed_field in (
            "mock_events_enabled",
            "mission_progress_available",
            "data_source",
        ):
            self.assertNotIn(removed_field, mock_runtime)
        mock_client = self.app.test_client()
        ros_client = ros_app.test_client()
        self.assertEqual(
            payload_shape(mock_client.get("/api/snapshot").get_json()),
            payload_shape(ros_client.get("/api/snapshot").get_json()),
        )
        for endpoint in ("/api/health", "/api/map"):
            self.assertEqual(
                payload_shape(mock_client.get(endpoint).get_json()),
                payload_shape(ros_client.get(endpoint).get_json()),
            )

        self.assertIn(b'id="mock-panel-toggle"', mock_client.get("/").data)
        self.assertNotIn(b'id="mock-panel-toggle"', ros_client.get("/").data)

        snapshot = mock_client.get("/api/snapshot").get_json()
        self.assertNotIn("active_alerts", snapshot["summary"])
        self.assertNotIn("progress", snapshot["mission"])
        self.assertNotIn("elapsed_sec", snapshot["mission"])
        self.assertNotIn("started_at", snapshot["mission"])

    def test_removed_feature_css_is_not_shipped(self):
        response = self.app.test_client().get("/static/css/dashboard.css")
        style = response.data
        for selector in (
            b".database-panel",
            b".login-card",
            b".data-reset-button",
            b".review-select",
            b".kpi-grid",
            b".connection-indicator",
            b".target-detail",
            b".progress{",
            b".command-row",
            b".command-confirm-dialog",
            b".fleet-stop-button",
            b".map-legend{",
            b"#camera-robot",
            b".map-tools .map-legend",
        ):
            self.assertNotIn(selector, style)
        response.close()


if __name__ == "__main__":
    unittest.main()

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
        root = Path(self.temp.name)
        return Settings(
            mode=mode,
            robots=(RobotConfig("robot4", "SCOUT"),),
            offline_timeout_sec=100,
            poll_interval_ms=1000,
            map_yaml_path=self.map_yaml,
            # 기본값(data/history.db)은 cwd 기준이라 테스트 중 실제 저장소에
            # 파일을 남긴다 — 임시 디렉터리로 격리한다.
            history_db_path=root / "history.db",
            history_image_dir=root / "captures",
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
        self.assertIn("기록 조회".encode(), dashboard)
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

    def test_camera_frame_route_serves_cached_frame_and_404s_when_missing(self):
        client = self.app.test_client()

        missing = client.get("/api/camera/robot4/frame")
        self.assertEqual(missing.status_code, 404)

        self.app.extensions["camera_frame_store"].update(
            "robot4", b"raw-jpeg-bytes", "jpeg"
        )
        found = client.get("/api/camera/robot4/frame")
        self.assertEqual(found.status_code, 200)
        self.assertEqual(found.mimetype, "image/jpeg")
        self.assertEqual(found.data, b"raw-jpeg-bytes")

    def test_history_routes_are_read_only_and_query_stored_records(self):
        client = self.app.test_client()
        store = self.app.extensions["history_store"]

        # 기록이 없을 때
        self.assertEqual(client.get("/api/history/summary").get_json(),
                          {"detections": 0, "trail_points": 0})
        self.assertEqual(client.get("/api/history/detections").get_json(), [])
        self.assertEqual(client.get("/api/history/trail").get_json(), [])

        detection_id = store.record_detection(
            robot_id="robot4", object_type="LIVE_RODENT", map_x=1.0, map_y=2.0,
            confidence=0.9, image_bytes=b"jpeg-bytes", image_ext="jpg",
        )
        store.record_trail_point(robot_id="robot4", map_x=1.0, map_y=2.0)

        detections = client.get("/api/history/detections").get_json()
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0]["id"], detection_id)
        self.assertEqual(detections[0]["image_url"],
                          f"/api/history/detections/{detection_id}/image")

        image = client.get(detections[0]["image_url"])
        self.assertEqual(image.status_code, 200)
        self.assertEqual(image.data, b"jpeg-bytes")

        self.assertEqual(client.get("/api/history/detections/9999/image").status_code, 404)

        trail = client.get("/api/history/trail?robot_id=robot4").get_json()
        self.assertEqual(len(trail), 1)

        # 쓰기/삭제 라우트는 없다 — read-only 원칙 유지. GET 라우트 자체는
        # 있으므로 다른 메서드는 404가 아니라 405(Method Not Allowed)다.
        self.assertEqual(client.post("/api/history/detections", json={}).status_code, 405)
        self.assertEqual(
            client.delete(f"/api/history/detections/{detection_id}/image").status_code, 405
        )

    def test_history_query_params_reject_non_numeric_values(self):
        client = self.app.test_client()
        self.assertEqual(
            client.get("/api/history/detections?since=not-a-number").status_code, 400
        )
        self.assertEqual(
            client.get("/api/history/trail?limit=not-a-number").status_code, 400
        )

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

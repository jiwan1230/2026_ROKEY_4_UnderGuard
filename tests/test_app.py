import tempfile
import time
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
        self.assertNotIn("전체 임무 진행률".encode(), dashboard.data)
        self.assertNotIn("탐지 누적".encode(), dashboard.data)
        self.assertNotIn("미해결 경고".encode(), dashboard.data)
        self.assertIn(b'id="fleet-connection-state"', dashboard.data)
        self.assertIn(b'id="operations-alert-banner"', dashboard.data)
        self.assertIn("최근 이벤트".encode(), dashboard.data)
        self.assertIn(b'id="map-legend-card" class="map-legend-card hidden"', dashboard.data)
        self.assertIn(b'id="map-marker-detail"', dashboard.data)
        self.assertIn(b'id="stop-all-robots"', dashboard.data)
        self.assertIn(b'class="panel-title fleet-panel-header"', dashboard.data)
        self.assertIn("전체 정지".encode(), dashboard.data)
        self.assertNotIn("전체 이동 정지".encode(), dashboard.data)
        self.assertIn(b'id="command-confirm-dialog"', dashboard.data)

        dashboard_script = client.get("/static/js/dashboard.js")
        dashboard_style = client.get("/static/css/dashboard.css")
        script_data = dashboard_script.data
        style_data = dashboard_style.data
        self.assertNotIn(b"camera-detection-facts", script_data)
        self.assertNotIn(b"camera-detection-facts", style_data)
        self.assertIn(b"camera-metadata-compact", script_data)
        self.assertNotIn("<small>임무 상태</small>".encode(), script_data)
        self.assertNotIn("<small>카메라 상태</small>".encode(), script_data)
        self.assertIn(b"shortRoleLabels", script_data)
        for unused_selector in (
            b".kpi-grid",
            b".connection-indicator",
            b".target-detail",
            b".progress{",
        ):
            self.assertNotIn(unused_selector, style_data)
        dashboard_script.close()
        dashboard_style.close()

        map_metadata = client.get("/api/map").get_json()
        self.assertTrue(map_metadata["available"])
        self.assertEqual((map_metadata["width"], map_metadata["height"]), (2, 2))
        map_image = client.get("/api/map/image")
        self.assertEqual(map_image.status_code, 200)
        self.assertEqual(map_image.mimetype, "image/png")

        detections_page = client.get("/detections")
        self.assertIn(b"js/detections.js", detections_page.data)
        self.assertIn(b'id="open-data-reset"', detections_page.data)
        self.assertIn(b'id="data-reset-dialog"', detections_page.data)
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
            b'id="fleet-connection-state"',
            b'id="operations-alert-banner"',
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

    def test_admin_can_reset_mock_data_but_users_and_schema_remain(self):
        client = self.app.test_client()
        client.post("/login", data={"username": "admin", "password": "admin123"})
        db = self.app.extensions["database"]
        state = self.app.extensions["state_manager"]
        mock = self.app.extensions["mock_manager"]
        state.update_robot(
            "robot4",
            state="SEARCHING",
            position_frame="map",
            position={"x": 1.0, "y": 2.0},
        )
        mock.trigger("RAT_DETECTED", "robot4")
        mock.trigger("TRAP_INSTALLED", "robot4")

        self.assertEqual(
            client.post("/api/admin/reset-operational-data", json={}).status_code,
            400,
        )
        response = client.post(
            "/api/admin/reset-operational-data",
            json={"confirmation": "RESET_OPERATIONAL_DATA"},
        )

        self.assertEqual(response.status_code, 200)
        result = response.get_json()
        self.assertTrue(result["reset"])
        self.assertFalse(result["scenario_active"])
        self.assertGreaterEqual(result["deleted"]["detections"], 1)
        self.assertGreaterEqual(result["deleted"]["events"], 1)
        self.assertGreaterEqual(result["deleted"]["traps"], 1)
        self.assertEqual(db.search_detections(), [])
        self.assertEqual(db.search_traps(), [])
        self.assertIsNotNone(db.find_user("admin"))
        snapshot = state.snapshot()
        self.assertEqual(snapshot["mission"]["status"], "READY")
        self.assertEqual(snapshot["events"], [])
        self.assertEqual(snapshot["detections"], [])
        self.assertEqual(snapshot["traps"], [])
        self.assertEqual(snapshot["robots"][0]["state"], "IDLE")
        self.assertEqual(snapshot["robots"][0]["connection"], "ONLINE")

    def test_operational_reset_requires_admin_and_supports_ros_mode(self):
        client = self.app.test_client()
        with client.session_transaction() as session:
            session["user"] = {"username": "operator", "role": "OPERATOR"}
        self.assertNotIn(b'id="open-data-reset"', client.get("/detections").data)
        self.assertEqual(
            client.post(
                "/api/admin/reset-operational-data",
                json={"confirmation": "RESET_OPERATIONAL_DATA"},
            ).status_code,
            403,
        )

        ros_settings = Settings(
            mode="ros",
            secret_key="test-secret",
            database_path=Path(self.temp.name) / "ros-reset.db",
            robots=(RobotConfig("robot4", "SCOUT"),),
            offline_timeout_sec=100,
            poll_interval_ms=1000,
            map_yaml_path=Path(self.temp.name) / "my_map.yaml",
        )
        ros_app = create_app(ros_settings)
        ros_client = ros_app.test_client()
        ros_client.post("/login", data={"username": "admin", "password": "admin123"})
        ros_db = ros_app.extensions["database"]
        ros_state = ros_app.extensions["state_manager"]
        ros_state.update_robot(
            "robot4",
            state="TRACKING",
            position={"x": 2.5, "y": 1.5},
            position_frame="map",
        )
        detection = {"robot_id": "robot4", "object_type": "LIVE_RODENT"}
        trap = {"robot_id": "robot4", "map_x": 2.5, "map_y": 1.5}
        ros_state.add_detection(detection)
        ros_state.add_event("탐지", robot_id="robot4", event_type="DETECTION")
        ros_state.add_trap(trap)
        ros_db.insert_detection(detection)
        ros_db.insert_event(
            {"robot_id": "robot4", "event_type": "DETECTION", "message": "탐지"}
        )
        ros_db.insert_trap(trap)

        ros_page = ros_client.get("/detections")
        self.assertIn(b'id="open-data-reset"', ros_page.data)
        self.assertIn("ROS 수집 노드".encode(), ros_page.data)
        response = ros_client.post(
            "/api/admin/reset-operational-data",
            json={"confirmation": "RESET_OPERATIONAL_DATA"},
        )

        self.assertEqual(response.status_code, 200)
        result = response.get_json()
        self.assertEqual(result["mode"], "ros")
        self.assertIsNone(result["scenario_active"])
        self.assertEqual(
            result["deleted"], {"detections": 1, "events": 1, "traps": 1}
        )
        self.assertEqual(ros_db.search_detections(), [])
        self.assertEqual(ros_db.search_traps(), [])
        snapshot = ros_state.snapshot()
        self.assertEqual(snapshot["events"], [])
        self.assertEqual(snapshot["detections"], [])
        self.assertEqual(snapshot["traps"], [])
        self.assertEqual(snapshot["robots"][0]["state"], "TRACKING")
        self.assertEqual(snapshot["robots"][0]["position"]["x"], 2.5)

    def test_demo_reset_keeps_service_running_without_restarting_scenario(self):
        client = self.app.test_client()
        client.post("/login", data={"username": "admin", "password": "admin123"})
        start_background_service(self.app)
        mock = self.app.extensions["mock_manager"]
        self.assertTrue(mock.running)

        response = client.post(
            "/api/admin/reset-operational-data",
            json={"confirmation": "RESET_OPERATIONAL_DATA"},
        )

        self.assertEqual(response.status_code, 200)
        self.assertFalse(response.get_json()["scenario_active"])
        self.assertTrue(mock.running)
        snapshot = self.app.extensions["state_manager"].snapshot()
        position = snapshot["robots"][0]["position"]
        time.sleep(1.2)
        after_wait = self.app.extensions["state_manager"].snapshot()
        self.assertEqual(after_wait["robots"][0]["position"], position)
        self.assertEqual(after_wait["mission"]["progress"], 0)
        self.assertEqual(after_wait["detections"], [])
        self.assertEqual(after_wait["events"], [])
        self.assertEqual(
            self.app.extensions["database"].search_detections(),
            [],
        )


if __name__ == "__main__":
    unittest.main()

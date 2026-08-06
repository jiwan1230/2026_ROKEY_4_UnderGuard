"""인증·로컬 DB 없이 실시간 관제 UI와 ROS/Mock 생명주기를 조립한다."""

from __future__ import annotations

import atexit
from io import BytesIO
from pathlib import Path

from flask import Flask, jsonify, render_template, request, send_file

from .config import Settings, load_settings
from .map_service import MapService
from .mock_manager import MockManager
from .ros_bridge import RosBridge
from .runtime_service import RuntimeService
from .state_manager import StateManager


def create_app(settings: Settings | None = None) -> Flask:
    """실시간 상태만 보관하는 Flask 애플리케이션을 생성한다.

    입력: 선택적인 실행 설정이다. 출력: 관제 화면과 API가 준비된 Flask 앱이다.
    사용: 테스트는 설정을 직접 전달하고 실제 실행은 ``main()``을 사용한다.
    로컬 DB와 로그인은 만들지 않으며 사건은 프로세스 메모리에만 유지한다.
    """

    settings = settings or load_settings()
    if settings.mode not in {"mock", "ros"}:
        raise ValueError("MONITOR_MODE는 mock 또는 ros여야 합니다.")

    app = Flask(__name__)
    app.config["JSON_AS_ASCII"] = False

    map_yaml_path = settings.map_yaml_path
    if not map_yaml_path.is_absolute():
        map_yaml_path = Path.cwd() / map_yaml_path
    map_service = MapService(
        map_yaml_path,
        frame_id=settings.ros_interface.map_frame,
    )

    state = StateManager(
        [(robot.robot_id, robot.role) for robot in settings.robots],
        offline_timeout_sec=settings.offline_timeout_sec,
        low_battery_threshold=settings.low_battery_threshold,
    )
    mock = MockManager(
        state,
        [robot.robot_id for robot in settings.robots],
        low_battery_threshold=settings.low_battery_threshold,
        map_frame=settings.ros_interface.map_frame,
    )
    ros = RosBridge(
        state,
        settings.robots,
        target_loss_timeout_sec=settings.target_loss_timeout_sec,
        low_battery_threshold=settings.low_battery_threshold,
        interface=settings.ros_interface,
    )
    runtime: RuntimeService = mock if settings.mode == "mock" else ros

    app.extensions["settings"] = settings
    app.extensions["state_manager"] = state
    app.extensions["mock_manager"] = mock
    app.extensions["ros_bridge"] = ros
    app.extensions["runtime_service"] = runtime
    app.extensions["map_service"] = map_service

    @app.get("/")
    @app.get("/dashboard")
    def dashboard():
        """로그인 단계 없이 실시간 통합 관제 화면을 반환한다."""

        return render_template(
            "dashboard.html",
            mode=settings.mode,
            poll_interval_ms=settings.poll_interval_ms,
        )

    @app.get("/api/health")
    def health():
        return jsonify(
            {
                "status": "ok" if runtime.available else "degraded",
                "mode": settings.mode,
                "ros_available": ros.available,
                "runtime": runtime.status(),
            }
        )

    @app.get("/api/snapshot")
    def snapshot():
        """현재 프로세스가 수신한 실시간 상태와 최근 사건만 반환한다."""

        result = state.snapshot()
        result["runtime"] = runtime.status()
        return jsonify(result)

    @app.get("/api/map")
    def map_metadata():
        """정적 ROS 맵의 좌표 변환 메타데이터와 이미지 URL을 반환한다."""

        return jsonify(map_service.describe("/api/map/image"))

    @app.get("/api/map/image")
    def map_image():
        """브라우저가 표시할 수 있도록 PGM을 PNG로 변환해 반환한다."""

        try:
            content = map_service.png_bytes()
        except (OSError, ValueError) as exc:
            return jsonify({"error": str(exc)}), 404
        return send_file(
            BytesIO(content),
            mimetype="image/png",
            download_name="map.png",
            max_age=60,
        )

    @app.post("/api/mock/events")
    def mock_event():
        """Mock 모드에서 실시간 UI 검증용 사건을 발생시킨다."""

        if settings.mode != "mock":
            return jsonify({"error": "mock_mode_only"}), 409
        payload = request.get_json(silent=True) or {}
        try:
            result = mock.trigger(
                str(payload.get("event_type", "")), payload.get("robot_id")
            )
            return jsonify(result)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400

    return app


def start_background_service(app: Flask) -> None:
    """현재 모드의 수집기를 프로세스에서 한 번만 시작한다."""

    if app.extensions.get("background_service_started"):
        return
    service = app.extensions["runtime_service"]
    service.start()
    app.extensions["background_service_started"] = True
    atexit.register(service.stop)


def main() -> None:
    """환경 설정으로 앱과 수집기를 시작한다."""

    app = create_app()
    start_background_service(app)
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)


if __name__ == "__main__":
    main()

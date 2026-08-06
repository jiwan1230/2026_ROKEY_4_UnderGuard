"""Flask 앱 조립, 인증 API, Mock/ROS 서비스 생명주기를 정의한다."""

from __future__ import annotations

import atexit
from datetime import datetime, time, timedelta, timezone
from functools import wraps
from io import BytesIO
from pathlib import Path
from typing import Any, Callable, TypeVar

from flask import (
    Flask,
    jsonify,
    redirect,
    render_template,
    request,
    send_file,
    session,
    url_for,
)

from .config import Settings, load_settings
from .database import Database
from .map_service import MapService
from .mock_manager import MockManager
from .ros_bridge import RosBridge
from .runtime_service import RuntimeService
from .security import verify_password
from .state_manager import StateManager

F = TypeVar("F", bound=Callable[..., Any])


def login_required(func: F) -> F:
    """로그인하지 않은 요청을 로그인 화면 또는 401 응답으로 전환한다."""

    @wraps(func)
    def wrapped(*args: Any, **kwargs: Any):
        if "user" not in session:
            if request.path.startswith("/api/"):
                return jsonify({"error": "authentication_required"}), 401
            return redirect(url_for("login"))
        return func(*args, **kwargs)

    return wrapped  # type: ignore[return-value]


def create_app(settings: Settings | None = None) -> Flask:
    """설정에 맞는 Flask 애플리케이션을 생성한다.

    입력: ``settings``는 모드, DB 경로, 로봇/로그인 설정이다.
    생략하면 환경 변수에서 읽는다.
    출력: 라우트와 상태/DB/서비스 객체가 준비된 ``Flask`` 인스턴스다.
    사용: 테스트는 ``create_app(test_settings)``로 생성하고,
    실행은 ``main()``을 사용한다.
    """

    settings = settings or load_settings()
    if settings.mode not in {"mock", "ros"}:
        raise ValueError("MONITOR_MODE는 mock 또는 ros여야 합니다.")

    app = Flask(__name__)
    app.secret_key = settings.secret_key
    app.config["JSON_AS_ASCII"] = False

    database_path = settings.database_path
    if not database_path.is_absolute():
        database_path = Path.cwd() / database_path
    db = Database(database_path)
    db.initialize()

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
    for saved_trap in reversed(db.search_traps(limit=100)):
        state.add_trap(saved_trap)
    mock = MockManager(
        state,
        db,
        [robot.robot_id for robot in settings.robots],
        low_battery_threshold=settings.low_battery_threshold,
        map_frame=settings.ros_interface.map_frame,
    )
    ros = RosBridge(
        state,
        db,
        settings.robots,
        target_loss_timeout_sec=settings.target_loss_timeout_sec,
        low_battery_threshold=settings.low_battery_threshold,
        interface=settings.ros_interface,
    )
    runtime: RuntimeService = mock if settings.mode == "mock" else ros

    # 라우트와 테스트가 함께 쓸 객체를 extensions에 보관한다.
    app.extensions["settings"] = settings
    app.extensions["database"] = db
    app.extensions["state_manager"] = state
    app.extensions["mock_manager"] = mock
    app.extensions["ros_bridge"] = ros
    app.extensions["runtime_service"] = runtime
    app.extensions["map_service"] = map_service

    @app.get("/")
    def index():
        return redirect(url_for("dashboard" if "user" in session else "login"))

    @app.route("/login", methods=["GET", "POST"])
    def login():
        error = None
        if request.method == "POST":
            username = request.form.get("username", "").strip()
            password = request.form.get("password", "")
            user = db.find_user(username)
            if user and verify_password(password, user["password_hash"]):
                session.clear()
                session["user"] = {"username": user["username"], "role": user["role"]}
                return redirect(url_for("dashboard"))
            error = "아이디 또는 비밀번호가 올바르지 않습니다."
        return render_template("login.html", error=error)

    @app.post("/logout")
    @login_required
    def logout():
        session.clear()
        return redirect(url_for("login"))

    @app.get("/dashboard")
    @login_required
    def dashboard():
        return render_template(
            "dashboard.html",
            mode=settings.mode,
            poll_interval_ms=settings.poll_interval_ms,
            user=session["user"],
        )

    @app.get("/detections")
    @login_required
    def detections_page():
        return render_template(
            "detections.html",
            mode=settings.mode,
            user=session["user"],
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
    @login_required
    def snapshot():
        result = state.snapshot()
        result["runtime"] = runtime.status()
        return jsonify(result)

    @app.get("/api/map")
    @login_required
    def map_metadata():
        """정적 ROS 맵의 좌표 변환 메타데이터와 이미지 URL을 반환한다."""

        return jsonify(map_service.describe(url_for("map_image")))

    @app.get("/api/map/image")
    @login_required
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

    @app.get("/api/detections")
    @login_required
    def detections_api():
        """검색 조건을 DB 필터로 바꾸고 ``{"items": [...]}``로 반환한다."""

        def utc_boundary(value: str | None, *, next_day: bool = False) -> str | None:
            """브라우저의 로컬 날짜 경계를 UTC DB 시간 문자열로 변환한다."""

            if not value:
                return None
            day = datetime.strptime(value, "%Y-%m-%d").date()
            if next_day:
                day += timedelta(days=1)
            local_time = datetime.combine(day, time.min).astimezone()
            return local_time.astimezone(timezone.utc).strftime("%Y-%m-%d %H:%M:%S")

        try:
            detected_after = utc_boundary(request.args.get("date_from"))
            detected_before = utc_boundary(request.args.get("date_to"), next_day=True)
            limit = int(request.args.get("limit", "200"))
        except (TypeError, ValueError):
            return (
                jsonify({"error": "날짜 또는 조회 개수가 올바르지 않습니다."}),
                400,
            )
        rows = db.search_detections(
            robot_id=request.args.get("robot_id") or None,
            object_type=request.args.get("object_type") or None,
            review_status=request.args.get("review_status") or None,
            detected_after=detected_after,
            detected_before=detected_before,
            limit=limit,
        )
        return jsonify({"items": rows})

    @app.patch("/api/detections/<int:detection_id>")
    @login_required
    def update_detection(detection_id: int):
        """운영자가 선택한 탐지의 검토 상태와 메모를 저장한다."""

        payload = request.get_json(silent=True) or {}
        status = str(payload.get("review_status", "REVIEWED"))
        memo = str(payload.get("memo", ""))
        try:
            if not db.update_detection_status(detection_id, status, memo):
                return jsonify({"error": "not_found"}), 404
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        return jsonify({"updated": True})

    @app.post("/api/admin/reset-operational-data")
    @login_required
    def reset_operational_data():
        """현재 모드의 운영 데이터와 화면 이력을 관리자 확인 후 초기화한다.

        입력: JSON ``confirmation=RESET_OPERATIONAL_DATA``.
        출력: 모드, 테이블별 삭제 건수와 수집기 실행 상태다.
        사용: 탐지 DB 화면의 관리자 전용 초기화 대화상자에서 호출한다.
        Mock은 임무까지 대기시키고 ROS는 현재 연결·위치와 수집기를 유지한다.
        """

        if session["user"].get("role") != "ADMIN":
            return jsonify({"error": "admin_required"}), 403
        payload = request.get_json(silent=True) or {}
        if payload.get("confirmation") != "RESET_OPERATIONAL_DATA":
            return jsonify({"error": "reset_confirmation_required"}), 400

        if settings.mode == "mock":
            deleted = mock.reset_demo()
            scenario_active: bool | None = False
        else:
            deleted = ros.reset_operational_data()
            scenario_active = None

        return jsonify(
            {
                "reset": True,
                "mode": settings.mode,
                "deleted": deleted,
                "scenario_active": scenario_active,
                "collector_running": runtime.running,
            }
        )

    @app.post("/api/commands")
    @login_required
    def command():
        """이동 명령을 검증한 뒤 현재 모드의 Mock/ROS 어댑터에 전달한다."""

        payload = request.get_json(silent=True) or {}
        robot_id = str(payload.get("robot_id", ""))
        command_name = str(payload.get("command", ""))
        try:
            result = runtime.send_command(robot_id, command_name)
            return jsonify(result), 200 if result.get("accepted") else 409
        except (KeyError, ValueError) as exc:
            return jsonify({"error": str(exc)}), 400

    @app.post("/api/mock/events")
    @login_required
    def mock_event():
        """수동 데모 이벤트를 발생시키고 그 처리 결과를 반환한다."""

        if settings.mode != "mock":
            return jsonify({"error": "mock_mode_only"}), 409
        payload = request.get_json(silent=True) or {}
        try:
            result = mock.trigger(str(payload.get("event_type", "")), payload.get("robot_id"))
            return jsonify(result)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400

    return app


def start_background_service(app: Flask) -> None:
    """실행 모드에 맞는 데이터 수집기를 명시적으로 한 번만 시작한다.

    입력: ``create_app``으로 생성되어 서비스 객체가 등록된 Flask 앱이다.
    출력: 없음. 선택한 서비스의 백그라운드 스레드를 시작한다.
    사용: 개발 서버 재로더의 중복 실행을 피하려 실제 프로세스에서
    한 번 호출한다.
    """
    if app.extensions.get("background_service_started"):
        return

    service = app.extensions["runtime_service"]

    service.start()
    app.extensions["background_service_started"] = True
    atexit.register(service.stop)


def main() -> None:
    """환경 설정으로 앱과 수집기를 시작하고 Flask 개발 서버를 실행한다."""

    app = create_app()
    start_background_service(app)
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)


if __name__ == "__main__":
    main()

"""인증·로컬 DB 없이 실시간 관제 UI와 ROS/Mock 생명주기를 조립한다."""

from __future__ import annotations

import atexit
from io import BytesIO
from pathlib import Path

from flask import Flask, jsonify, render_template, request, send_file

from .camera_service import CameraFrameStore
from .config import Settings, load_settings
from .history_store import HistoryStore
from .map_service import MapService
from .mock_manager import MockManager
from .replay_manager import DEFAULT_FRAMES_PATH, ReplayManager
from .ros_bridge import RosBridge
from .runtime_service import RuntimeService
from .state_manager import StateManager

# 화면(templates/static)은 Sysmon/frontend/ 밑에 backend와 물리적으로 분리돼
# 있어 Flask의 기본 자동탐색(모듈과 같은 폴더의 templates/static)을 못 쓴다.
# app.py 위치(system_monitor/) 기준 3단계 위가 Sysmon/이다.
_FRONTEND_DIR = Path(__file__).resolve().parent.parent.parent / "frontend"


# settings의 값을 참고하여 새로운 Flask 객체를 만들고, 그 Flask 객체에 설정을 적용한다.
# settings는 system_monitor/config.py에서 정의되고 만들어지는 설정 객체
def create_app(settings: Settings | None = None) -> Flask:

    settings = settings or load_settings()
    if settings.mode not in {"mock", "ros", "replay"}:
        raise ValueError("MONITOR_MODE는 mock, ros 또는 replay여야 합니다.")

    app = Flask(
        __name__,
        template_folder=str(_FRONTEND_DIR / "templates"),
        static_folder=str(_FRONTEND_DIR / "static"),
    )
    app.config["JSON_AS_ASCII"] = False

    map_yaml_path = settings.map_yaml_path
    if not map_yaml_path.is_absolute():
        map_yaml_path = Path.cwd() / map_yaml_path
    # 핵심 포인트 3 — 지도 처리를 MapService로 분리
    # 앞에서 준비한 지도 YAML 파일 경로를 전달
    # 설정 객체에 저장된 지도 좌표계 이름을 가져와 MapService의 frame_id 매개변수로 전달 
    # 이를 통해 지도 데이터가 ROS의 map 좌표계를 기준으로 사용되도록 설정
    map_service = MapService(
        map_yaml_path,
        frame_id=settings.ros_interface.map_frame,
    )
    # 핵심 포인트 4 — StateManager가 관제 상태의 중심
    state = StateManager(
        # 설정에 저장된 각 로봇에서 ID와 역할을 꺼내 리스트로 만듭니다.
        [(robot.robot_id, robot.role) for robot in settings.robots],
        offline_timeout_sec=settings.offline_timeout_sec,
    )
    # Mock이 실제 맵 범위 밖으로 로봇을 움직이지 않도록, 맵을 못 읽는
    # 경우에만 예전 my_map.yaml 언저리 크기로 대체한다(맵 자체가 없어도
    # Mock 시연은 항상 동작해야 하므로).
    try:
        map_origin_x, map_origin_y, map_width_m, map_height_m = map_service.bounds()
    except (OSError, ValueError):
        map_origin_x, map_origin_y, map_width_m, map_height_m = 0.0, 0.0, 6.0, 5.0
    mock = MockManager(
        state,
        [robot.robot_id for robot in settings.robots],
        low_battery_threshold=settings.low_battery_threshold,
        map_frame=settings.ros_interface.map_frame,
        map_origin=(map_origin_x, map_origin_y),
        map_size=(map_width_m, map_height_m),
    )
    # 로봇별 최신 카메라 프레임 캐시. StateManager와 분리해서 원본 바이트가
    # /api/snapshot의 JSON 직렬화 경로에 안 섞이게 한다(camera_service.py).
    camera_frame_store = CameraFrameStore()

    # 기록 조회 탭 전용 영구 저장소 — 실시간 뷰(StateManager)와 분리된다.
    # 상대경로는 map_yaml_path와 같은 규칙(Path.cwd() 기준)으로 푼다.
    history_db_path = settings.history_db_path
    if not history_db_path.is_absolute():
        history_db_path = Path.cwd() / history_db_path
    history_image_dir = settings.history_image_dir
    if not history_image_dir.is_absolute():
        history_image_dir = Path.cwd() / history_image_dir
    history_store = HistoryStore(history_db_path, history_image_dir)
    # 핵심 포인트 5 — ROS 데이터를 공통 상태로 변환
    # 실제 ROS 2 메시지를 받을 RosBridge 객체를 생성
    ros = RosBridge(
        state,
        settings.robots,
        target_loss_timeout_sec=settings.target_loss_timeout_sec,
        low_battery_threshold=settings.low_battery_threshold,
        interface=settings.ros_interface,
        camera_frame_store=camera_frame_store,
    )
    # replay 모드 — herding_controller_dual 검증 시뮬레이션이 남긴 궤적을
    # 재생한다. Mock/ROS와 마찬가지로 항상 만들어 두되(app.extensions로
    # 공유), 실제로 도는 건 mode == "replay"일 때뿐이다.
    replay = ReplayManager(
        state,
        [robot.robot_id for robot in settings.robots],
        frames_path=settings.replay_frames_path or DEFAULT_FRAMES_PATH,
        trial_index=settings.replay_trial_index,
        speed=settings.replay_speed,
        map_frame=settings.ros_interface.map_frame,
    )

    # 핵심 포인트 6 — Mock/ROS/Replay를 같은 방식으로 사용
    # 실행 모드에 따라 실제 사용할 서비스를 선택
    # runtime이 MockManager든 RosBridge든 ReplayManager든 RuntimeService의
    # 공통 규칙을 따름 — 현재 설정된 모드에 따라 선택된 실행 서비스 객체
    runtime: RuntimeService = {
        "mock": mock,
        "replay": replay,
    }.get(settings.mode, ros)

    # 핵심 포인트 7 — 생성한 객체를 app.extensions에서 공유
    # app.extensions["settings"] = settings는
    # 생성된 설정 객체를 Flask 앱의 공용 저장 공간에 "settings"라는 이름으로 보관해,
    # 다른 코드에서도 다시 사용할 수 있게 하는 코드
    app.extensions["settings"] = settings
    app.extensions["state_manager"] = state
    app.extensions["mock_manager"] = mock
    app.extensions["ros_bridge"] = ros
    app.extensions["replay_manager"] = replay
    app.extensions["runtime_service"] = runtime     # 현재 실행 모드에 따라 선택된 서비스를 저장
    app.extensions["map_service"] = map_service
    app.extensions["camera_frame_store"] = camera_frame_store
    app.extensions["history_store"] = history_store

    # 핵심 포인트 8 — 하나의 함수가 두 URL을 처리
    # 두 주소로 들어오는 GET 요청을 같은 함수가 처리하도록 연결하는 장식자(데코레이터)
    # 사용자가 다음 두 주소 중 어느 곳으로 접속해도 같은 dashboard() 함수가 실행되고, 같은 화면을 보여줌
    @app.get("/")
    @app.get("/dashboard")
    def dashboard():
        """현재 개발 단계에서는, 로그인 단계 없이 실시간 통합 관제 화면을 반환한다."""

        # HTML 템플릿을 화면으로 반환
        # templates/dashboard.html 파일을 사용
        # 현재 실행 모드를 HTML에 전달
        # 웹 화면의 상태 갱신 간격을 전달
        return render_template(
            "dashboard.html",
            mode=settings.mode,
            poll_interval_ms=settings.poll_interval_ms,
        )

    # 핵심 포인트 9 — 서비스가 정상인지 외부에서 확인
    # jsonify()는 전달한 파이썬 딕셔너리를 웹에서 주고받을 수 있는 JSON 응답으로 만들어 반환하는 Flask 함수
    # "status": 현재 실행 서비스가 정상 사용 가능한지 표시
    # "ros_available": ROS 기능을 사용할 수 있는지 여부
    # "runtime": 현재 선택된 실행 서비스의 상세 상태
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

    # 웹 관제 화면은 이 API를 주기적으로 호출해 현재 상태를 갱신
    # 현재 서버 메모리에 저장된 데이터만 반환
    # StateManager에서 웹 화면에 필요한 전체 상태를 가져옴
    @app.get("/api/snapshot")
    def snapshot():
        """현재 프로세스가 수신한 실시간 상태와 최근 사건만 반환한다."""

        result = state.snapshot()
        # 로봇 상태와 실행 서비스 상태를 한 번에 함께 보내기 위해서
        result["runtime"] = runtime.status()
        return jsonify(result)

    @app.get("/api/map")
    def map_metadata():
        """정적 ROS 맵의 좌표 변환 메타데이터와 이미지 URL을 반환한다."""

        return jsonify(map_service.describe("/api/map/image"))

    # 핵심 포인트 11 — ROS PGM 지도를 웹 PNG로 변환
    # ROS PGM 지도는 로봇이 이동할 공간을 흑백 이미지로 표현한 지도 파일
    # 지도 변환 중 오류가 발생할 수 있으므로 예외 처리를 시작
    # content에는 이미지 파일의 내용이 들어 있지만, 아직 디스크 파일로 저장된 것은 아님
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

    # 핵심 - 로봇별 카메라 이미지를 가져오는 API
    # URL에서 받은 robot_id를 매개변수로 받음
    @app.get("/api/camera/<robot_id>/frame")
    def camera_frame(robot_id: str):

        # 중요 - 해당 로봇의 가장 최신 카메라 프레임을 가져옴
        frame = camera_frame_store.get(robot_id)
        if frame is None:
            # 카메라 프레임이 아직 준비되지 않았다면, 
            # no_frame_available이라는 오류 정보를 JSON 형식으로 만들어 HTTP 404 상태 코드와 함께 클라이언트에 반환
            return jsonify({"error": "no_frame_available"}), 404
        return send_file(
            BytesIO(frame.content),
            mimetype=f"image/{frame.format}",
            # 중요 - 지도 → 거의 안 바뀜, 카메라 → 계속 최신 프레임으로 바뀜
            # 카메라 이미지는 계속 바뀌는 실시간 데이터이기 때문에, 
            # 캐시하지 않고 항상 최신 프레임을 받아오도록 max_age=0으로 설정
            max_age=0,
        )

    # 기록 조회 탭 전용 — 전부 조회(GET)만 있고 쓰기/삭제 라우트는 없다.
    # "기록을 남기고 볼 수는 있지만 지우거나 고칠 수는 없다"는 read-only
    # 원칙을 이 저장소에도 그대로 유지한다.
    @app.get("/api/history/summary")
    def history_summary():
        return jsonify(history_store.summary())

    @app.get("/api/history/detections")
    def history_detections():
        args = request.args
        try:
            limit = min(int(args.get("limit", 200)), 1000)
            since = float(args["since"]) if "since" in args else None
            until = float(args["until"]) if "until" in args else None
        except (TypeError, ValueError):
            return jsonify({"error": "limit/since/until은 숫자여야 합니다."}), 400
        return jsonify(
            history_store.list_detections(
                limit=limit,
                since=since,
                until=until,
                object_type=args.get("object_type") or None,
                robot_id=args.get("robot_id") or None,
            )
        )

    @app.get("/api/history/detections/<int:detection_id>/image")
    def history_detection_image(detection_id: int):
        path = history_store.image_path_for(detection_id)
        if path is None:
            return jsonify({"error": "no_image_available"}), 404
        return send_file(path, max_age=3600)

    @app.get("/api/history/trail")
    def history_trail():
        args = request.args
        try:
            limit = min(int(args.get("limit", 5000)), 20000)
            since = float(args["since"]) if "since" in args else None
            until = float(args["until"]) if "until" in args else None
        except (TypeError, ValueError):
            return jsonify({"error": "limit/since/until은 숫자여야 합니다."}), 400
        return jsonify(
            history_store.get_trail(
                robot_id=args.get("robot_id") or None,
                since=since,
                until=until,
                limit=limit,
            )
        )

    # 로봇 DB(db_node/MySQL) 기록 조회 — 여기도 조회만 있고 쓰기 라우트는 없다.
    # MySQL에 직접 붙지 않고 db_node의 /db/query 서비스를 거친다: DB 스키마가
    # 바뀌어도 UI는 안 깨지고, DB 접속 정보도 관제 서버에 두지 않는다.
    DB_QUERIES = {"detections", "missions", "traps", "report"}

    @app.get("/api/db/<query_name>")
    def db_query(query_name: str):
        if query_name not in DB_QUERIES:
            return jsonify({"error": "unknown_query", "allowed": sorted(DB_QUERIES)}), 404
        try:
            limit = min(int(request.args.get("limit", 100)), 500)
        except (TypeError, ValueError):
            return jsonify({"error": "limit은 숫자여야 합니다."}), 400

        result, error = ros.query_db(query_name, {"limit": limit})
        if error is not None:
            # DB가 없거나 응답이 없어도 실시간 화면은 그대로 돈다 — 이 요청만 실패한다.
            status = 503 if error in {"ros_unavailable", "db_node_unavailable"} else 502
            return jsonify({"error": error}), status
        return jsonify(result)

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

# Create_app() End
#==========================================================================================================

def start_background_service(app: Flask) -> None:
    """현재 모드의 수집기를 프로세스에서 한 번만 시작한다."""

    # 중요 - 서비스가 이미 시작됐는지 확인
    if app.extensions.get("background_service_started"):
        return
    service = app.extensions["runtime_service"]
    service.start()
    app.extensions["background_service_started"] = True
    # 중요 - 프로그램이 종료될 때 서비스를 안전하게 종료하도록 예약
    atexit.register(service.stop)


def main() -> None:
    """환경 설정으로 앱과 수집기를 시작한다."""

    app = create_app()
    start_background_service(app)
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)


if __name__ == "__main__":
    main()

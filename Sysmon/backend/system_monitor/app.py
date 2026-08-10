"""시스템에 필요한 각 서비스를 생성하고 연결한 뒤,
Flask API를 통해 웹 관제 화면에 제공하는 전체 시스템의 조립 및 진입점"""

from __future__ import annotations

import atexit
from io import BytesIO
from pathlib import Path

from flask import Flask, jsonify, render_template, request, send_file

from .camera_service import CameraFrameStore
from .config import Settings, load_settings
from .map_service import MapService
from .mock_manager import MockManager
from .ros_bridge import RosBridge
from .runtime_service import RuntimeService
from .state_manager import StateManager

# 화면(templates/static)은 Sysmon/frontend/ 밑에 backend와 물리적으로 분리돼
# 있어 Flask의 기본 자동탐색(모듈과 같은 폴더의 templates/static)을 못 쓴다.
# app.py 위치(system_monitor/) 기준 3단계 위가 Sysmon/이다.
_FRONTEND_DIR = Path(__file__).resolve().parent.parent.parent / "frontend"


# 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 #
'''
설정을 불러와 Flask 앱을 만들고,
StateManager, MapService, RosBridge 같은 시스템 구성 요소들을 생성해서 하나의 웹 서버로 연결
'''
def create_app(settings: Settings | None = None) -> Flask:

    settings = settings or load_settings()
    if settings.mode not in {"mock", "ros"}:
        raise ValueError("MONITOR_MODE는 mock 또는 ros여야 합니다.")

    # 핵심 포인트 #
    # Flask 애플리케이션 객체를 만드는 코드
    app = Flask(
        __name__, # 현재 실행 중인 모듈의 위치를 알려주는 값
        template_folder=str(_FRONTEND_DIR / "templates"), # HTML 파일이 있는 폴더를 지정
        static_folder=str(_FRONTEND_DIR / "static"), # 정적 파일이 있는 폴더를 지정
    ) # 문자열 경로로 변환
    # JSON에서 한글을 사용할 수 있도록 설정
    app.config["JSON_AS_ASCII"] = False

    map_yaml_path = settings.map_yaml_path
    if not map_yaml_path.is_absolute():
        map_yaml_path = Path.cwd() / map_yaml_path

    # 핵심 포인트 #
    # ROS 맵을 읽고, 웹에서 사용할 수 있는 지도 정보와 이미지로 처리하는 객체를 생성
    map_service = MapService(
        # 용할 ROS 지도 YAML 파일의 경로
        map_yaml_path,
        # 이 지도가 어떤 좌표계를 기준으로 하는지 전달(보통 map)
        frame_id=settings.ros_interface.map_frame,
    )

    # 핵심 포인트 #
    # 설정에 등록된 로봇들을 이용해서 StateManager를 만듦
    state = StateManager(
        # 설정에 저장된 각 로봇에서 ID와 역할을 꺼내 리스트로 만듭니다.
        [(robot.robot_id, robot.role) for robot in settings.robots],
        # 로봇으로부터 일정 시간 동안 데이터가 들어오지 않았을 때 OFFLINE으로 판단하기 위한 기준 시간을 전달
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

    # 핵심 포인트 #
    # 로봇별 최신 카메라 이미지를 잠깐 저장해둘 공간을 만듦
    camera_frame_store = CameraFrameStore()

    # 핵심 포인트 #
    # StateManager와 로봇 설정, 탐지 유실 시간, 배터리 임계값, ROS 인터페이스, 카메라 저장소를 전달해서
    # 실제 ROS 2 메시지를 받을 RosBridge 객체를 생성
    ros = RosBridge(
        state,
        settings.robots,
        target_loss_timeout_sec=settings.target_loss_timeout_sec,
        low_battery_threshold=settings.low_battery_threshold,
        interface=settings.ros_interface,
        camera_frame_store=camera_frame_store,
    )

    # 핵심 포인트 #
    # 실행 모드에 따라 실제 사용할 서비스를 선택'''
    # runtime이 MockManager든 RosBridge든 RuntimeService의 공통 규칙을 따름
    # runtime은 실시간 데이터 자체가 아니라,
    # 현재 설정된 모드에 따라 선택된 Mock 또는 ROS 실행 서비스 객체
    runtime: RuntimeService = mock if settings.mode == "mock" else ros

    # 핵심 포인트 #
    # 앞에서 만들어 둔 여러 서비스 객체들을 Flask 앱 안에 공용으로 저장해둠
    # 다른 코드에서도 다시 사용할 수 있게 하는 코드
    app.extensions["settings"] = settings
    app.extensions["state_manager"] = state
    app.extensions["mock_manager"] = mock
    app.extensions["ros_bridge"] = ros
    app.extensions["runtime_service"] = runtime     # 현재 실행 모드에 따라 선택된 서비스를 저장
    app.extensions["map_service"] = map_service
    app.extensions["camera_frame_store"] = camera_frame_store


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

    # 핵심 포인트 # ㅡ 서버와 현재 실행 서비스의 상태를 /api/health로 제공'''
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

    # 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 #
    # 현재 시스템의 전체 관제 상태를 모아서 JSON으로 웹에 보내주는 API
    @app.get("/api/snapshot") # 웹에서 /api/snapshot 주소로 GET 요청이 오면 이 함수를 실행
    def snapshot():
        # StateManager에서 현재 로봇 상태, 임무 상태, 탐지·이벤트 같은 현재 관제 정보를 한 번에 가져옴
        result = state.snapshot()
        # 여기에 현재 ROS/Mock 실행 서비스의 상태도 추가
        result["runtime"] = runtime.status()
        # 최종 데이터를 JSON 형식으로 변환해서 웹 프론트엔드에 반환
        return jsonify(result)


    @app.get("/api/map")
    def map_metadata():
        """정적 ROS 맵의 좌표 변환 메타데이터와 이미지 URL을 반환한다."""

        return jsonify(map_service.describe("/api/map/image"))

    # 핵심 포인트 # ㅡ ROS 지도 이미지를 웹에서 볼 수 있도록 /api/map/image로 제공'''
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

    # 핵심 포인트 # ㅡ 각 로봇의 최신 카메라 이미지를 /api/camera/<robot_id>/frame으로 제공'''
    # URL에서 받은 robot_id를 매개변수로 받음
    @app.get("/api/camera/<robot_id>/frame")
    def camera_frame(robot_id: str):

        # 해당 로봇의 가장 최신 카메라 프레임을 가져옴
        frame = camera_frame_store.get(robot_id)
        if frame is None:
            # 카메라 프레임이 아직 준비되지 않았다면,
            # no_frame_available이라는 오류 정보를 JSON 형식으로 만들어 HTTP 404 상태 코드와 함께 클라이언트에 반환
            return jsonify({"error": "no_frame_available"}), 404
        return send_file(
            BytesIO(frame.content),
            mimetype=f"image/{frame.format}",
            # 지도 → 거의 안 바뀜, 카메라 → 계속 최신 프레임으로 바뀜
            # 카메라 이미지는 계속 바뀌는 실시간 데이터이기 때문에,
            # 캐시하지 않고 항상 최신 프레임을 받아오도록 max_age=0으로 설정
            max_age=0,
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

# Create_app() End
#==========================================================================================================

def start_background_service(app: Flask) -> None:
    """현재 모드의 수집기를 프로세스에서 한 번만 시작한다."""

    # 서비스가 이미 시작됐는지 확인
    if app.extensions.get("background_service_started"):
        return
    service = app.extensions["runtime_service"]
    service.start()
    app.extensions["background_service_started"] = True
    # 프로그램이 종료될 때 서비스를 안전하게 종료하도록 예약
    atexit.register(service.stop)

# 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 ## 중요 #
# 이 프로그램을 실제로 시작하는 마지막 진입점
def main() -> None:

    # Flask 앱과 StateManager, RosBridge, MapService 같은 필요한 서비스들을 생성하고 연결
    app = create_app()
    # 현재 실행 모드에 맞는 서비스, 즉 ROS 또는 Mock mode를 시작
    start_background_service(app)
    # run() 함수로 Flask 웹 서버를 실제로 실행
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
    '''
    host="0.0.0.0" → 같은 네트워크의 다른 PC에서도 접속 가능
    port=5000 → 5000번 포트 사용
    debug=False → 디버그 모드 끔
    threaded=True → 여러 요청을 스레드로 처리할 수 있게 설정
    '''


if __name__ == "__main__":
    main()

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
WORKSPACE_SETUP="$WORKSPACE_DIR/install/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS 2 설정 파일을 찾을 수 없습니다: $ROS_SETUP" >&2
  exit 1
fi

# ROS/colcon의 setup 스크립트는 미정의 환경변수를 확인할 수 있으므로 source하는
# 동안에만 nounset을 끈다. 실행 본문은 다시 엄격 모드로 돌아간다.
set +u
source "$ROS_SETUP"
if [[ ! -f "$WORKSPACE_SETUP" ]]; then
  echo "현재 저장소의 ROS 워크스페이스가 빌드되지 않았습니다." >&2
  echo "다음을 먼저 실행하세요:" >&2
  echo "  cd $WORKSPACE_DIR" >&2
  echo "  source $ROS_SETUP" >&2
  echo "  colcon build --packages-select turtle_interfaces turtle_project" >&2
  exit 1
fi
source "$WORKSPACE_SETUP"
set -u

cd "$SCRIPT_DIR"

# 다른 시험에서 남은 Fast DDS 프로필 경로가 사라졌다면 Fast DDS가 매 실행마다
# XMLPARSER 오류를 출력한다. Fast DDS 버전에 따라 두 환경변수 이름 중 하나를
# 읽으므로 둘 다 검사하고, 존재하지 않는 값만 제거한다.
for profile_var in FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE; do
  profile_path="${!profile_var:-}"
  if [[ -n "$profile_path" && ! -f "$profile_path" ]]; then
    echo "존재하지 않는 $profile_var 설정을 무시합니다: $profile_path" >&2
    unset "$profile_var"
  fi
done

# 사용: ROS 2와 로봇 워크스페이스를 source한 터미널에서 ./run_ros.sh
# 출력: Mock과 같은 Flask/API를 사용하며 입력 데이터만 ROS 2 토픽에서 받는다.
export MONITOR_MODE="ros"
# 별도 설정이 없으면 DB 브랜치에 포함된 현재 운영 지도를 사용한다.
export MAP_YAML_PATH="${MAP_YAML_PATH:-$WORKSPACE_DIR/src/turtle_project/resource/room_map.yaml}"

if ! python3 -c "import rclpy; from turtle_interfaces.msg import DetectionEvent; from turtle_interfaces.srv import DbQuery" >/dev/null 2>&1; then
  echo "ROS 2 또는 UnderGuard 커스텀 인터페이스를 불러올 수 없습니다." >&2
  echo "워크스페이스를 다시 빌드한 뒤 install/setup.bash를 source해 주세요." >&2
  exit 1
fi

exec python3 -m system_monitor.app

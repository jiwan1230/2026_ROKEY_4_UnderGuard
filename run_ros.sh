#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

# 사용: ROS 2와 로봇 워크스페이스를 source한 터미널에서 ./run_ros.sh
# 출력: Mock과 같은 Flask/API를 사용하며 입력 데이터만 ROS 2 토픽에서 받는다.
export MONITOR_MODE="ros"
export SECRET_KEY="${SECRET_KEY:-dev-secret-change-me}"

if ! python3 -c "import rclpy" >/dev/null 2>&1; then
  echo "ROS 2 Python 환경을 찾을 수 없습니다." >&2
  echo "먼저 /opt/ros/humble/setup.bash와 로봇 워크스페이스를 source해 주세요." >&2
  exit 1
fi

exec python3 -m system_monitor.app

#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

# 이 스크립트 이름과 실제 런타임이 어긋나지 않도록 Mock 모드를 고정한다.
export MONITOR_MODE="mock"
export SECRET_KEY="${SECRET_KEY:-dev-secret-change-me}"
exec python3 -m system_monitor.app

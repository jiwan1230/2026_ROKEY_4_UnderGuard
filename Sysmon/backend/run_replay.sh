#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

# 이 스크립트 이름과 실제 런타임이 어긋나지 않도록 Replay 모드를 고정한다.
# herding_controller_dual 검증 시뮬레이션이 남긴 실제 궤적(system_monitor/
# replay_data/real_map_frames.json)을 재생한다 — Mock의 원 궤적이 아니라
# 알고리즘이 실제로 계산한 도주/추격 경로다. 어느 trial을 얼마나 빠르게
# 재생할지는 REPLAY_TRIAL / REPLAY_SPEED로 바꿀 수 있다(기본 0번, 1배속).
export MONITOR_MODE="replay"
exec python3 -m system_monitor.app

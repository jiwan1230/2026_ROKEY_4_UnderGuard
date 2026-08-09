#!/usr/bin/env bash
# WiFi + Discovery Server 환경에서는 lifecycle 전환 서비스 응답이 자주 타임아웃 나서
# map_server/amcl/nav2가 unconfigured나 inactive에 멈춘다. 멈춘 것만 골라 밀어 올린다.
#
#   ./nav2_activate.sh [namespace] [wait_sec] [nodes]   기본값 robot4 90 전체
#
# localization.launch.py / nav2.launch.py 를 먼저 띄운 뒤에 실행할 것.
set -u
NS="${1:-robot4}"
NS="${NS#/}"        # '/robot4'로 넘어와도 앞 슬래시 제거 (launch가 ns 그대로 넘김)
WAIT="${2:-90}"     # 노드가 뜰 때까지 기다릴 최대 초

# 3번째 인자로 활성화할 노드 집합을 좁힐 수 있다 (예: localization만, navigation만).
# 순서 중요: 코스트맵이 map_server를 먼저 필요로 한다. 생략하면 전체.
NODES="${3:-}"
if [ -z "$NODES" ]; then
    NODES="map_server amcl controller_server smoother_server planner_server \
           behavior_server bt_navigator waypoint_follower velocity_smoother"
fi

state() { timeout 8 ros2 lifecycle get "/$NS/$1" 2>/dev/null | tail -1; }

# launch와 동시에 실행돼도 되도록, 목록의 마지막 노드가 뜰 때까지 기다린다.
echo "노드 대기 중 (최대 ${WAIT}초)..."
deadline=$((SECONDS + WAIT))
until [ -n "$(state "${NODES##* }")" ]; do
    if [ "$SECONDS" -ge "$deadline" ]; then
        echo "❌ ${WAIT}초 안에 노드가 안 떴습니다. launch 터미널의 에러를 확인하세요."
        exit 1
    fi
    sleep 2
done
echo "노드 확인됨."
echo

for phase in configure activate; do
    echo "── $phase ──"
    for n in $NODES; do
        s=$(state "$n")
        case "$s" in
            '')             echo "  $n: 노드 없음 (launch 안 떴나?)" ;;
            *unconfigured*) [ "$phase" = configure ] && \
                            printf '  %-20s %s\n' "$n" "$(timeout 25 ros2 lifecycle set "/$NS/$n" configure 2>&1 | tail -1)" ;;
            *inactive*)     [ "$phase" = activate ] && \
                            printf '  %-20s %s\n' "$n" "$(timeout 25 ros2 lifecycle set "/$NS/$n" activate 2>&1 | tail -1)" ;;
            *active*)       [ "$phase" = configure ] && printf '  %-20s 이미 active\n' "$n" ;;
        esac
    done
done

echo
echo "── 최종 상태 ──"
bad=0
found=0
for n in $NODES; do
    s=$(state "$n")
    [ -z "$s" ] && continue
    found=$((found + 1))
    case "$s" in *active*) mark="✅" ;; *) mark="❌"; bad=1 ;; esac
    printf '  %s %-20s %s\n' "$mark" "$n" "$s"
done

echo
if [ "$found" = 0 ]; then
    echo "❌ lifecycle 노드가 하나도 없습니다. localization.launch.py / nav2.launch.py를"
    echo "   먼저 띄우세요 (또는 이미 죽었는지 그 터미널을 확인)."
    exit 1
elif [ "$bad" = 0 ]; then
    echo "전부 active. TF 확인:"
    echo "  ros2 run tf2_ros tf2_echo map base_link --ros-args \\"
    echo "      -r /tf:=/$NS/tf -r /tf_static:=/$NS/tf_static"
else
    echo "❌ 남은 게 있습니다. 한 번 더 실행해 보세요 (타임아웃이면 대개 재시도로 붙습니다)."
    exit 1
fi

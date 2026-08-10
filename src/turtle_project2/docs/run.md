# 실행 명령 — 네임스페이스 정렬 기준

모든 로봇 노드는 **`__ns:=/robotX` + 상대토픽** 한 방식으로 통일했다. TF를 직접
쓰는 노드(detector·trap_check)만 `/tf` remap을 추가한다. db_node·central은
`/fleet/*` 절대토픽만 쓰므로 네임스페이스 없이 중앙에서 한 번만 띄운다.

> 정렬 원리: robot_agent는 이제 `target_pose`·`battery_state`·`patrol_hold`를
> 상대경로로 구독한다(detector·trap_check가 발행하는 것과 같은 상대경로).
> `__ns:=/robot4`를 걸면 양쪽 다 `/robot4/...`가 되어 붙는다. robot_agent는
> `__ns`를 **반드시** 걸어야 한다(안 걸면 토픽이 `/target_pose`로 떠서 안 맞음).

## 로봇 PC (robot4 기준 — robot6은 /robot4→/robot6, robot4.yaml→robot6.yaml)

```bash
# 0) Nav2 기반 (localization + nav2 + RViz). 뜨면 RViz 2D Pose Estimate로 초기위치.
ros2 launch turtle_project robot_bringup.launch.py

# 1) 카메라 sync (TF 안 씀)
ros2 run turtle_project camera_node --ros-args -r __ns:=/robot4

# 2) 감지 (best.pt 자동, TF 씀 → /tf remap 필수)
ros2 run turtle_project detector_node --ros-args \
  -r __ns:=/robot4 -r /tf:=tf -r /tf_static:=tf_static

# 3) trap 점검 (TF 씀 → /tf remap 필수)
ros2 run turtle_project trap_check_node --ros-args \
  -r __ns:=/robot4 -r /tf:=tf -r /tf_static:=tf_static

# 4) 로봇 주행 (__ns + params-file 둘 다: __ns=토픽정렬, params=dock좌표·식별)
ros2 run turtle_project robot_agent --ros-args \
  -r __ns:=/robot4 --params-file src/turtle_project/config/robot4.yaml
```

## 중앙 PC (전체에서 한 번만 — 네임스페이스 없음)

```bash
# central_node, db_node, webcam_node, rat_herding_node(배관), herding_node(몰이 알고리즘)를 한 번에.
# herding_node는 turtle_project.herding 내장 (플랜 A 단일 blocker).
ros2 launch turtle_project central_pc.launch.py
```

central을 띄우면 전원 도킹 상태에서 **자동으로 한 대를 UNDOCK시켜 순찰을 시작**한다
(부트스트랩). 이후 배터리<임계로 A가 도킹하면 B를 깨워 교대, 쥐 감지 시 역할배정.

## 왜 `/tf` remap이 필요한가
`tf2_ros`의 TransformListener는 네임스페이스를 무시하고 절대경로 `/tf`를 구독한다.
로봇 TF는 `/robot4/tf`로 나오므로, `-r /tf:=tf`(상대)로 remap해야 `__ns` 밑에서
`/robot4/tf`를 듣는다. detector·trap_check가 `map→base_link` 조회에 쓴다.
robot_agent는 TF를 직접 안 쓰고 Nav2 액션 서버(네임스페이스됨)만 쓰므로 불필요.

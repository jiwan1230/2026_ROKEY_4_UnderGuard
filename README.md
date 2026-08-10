# ws_turtle — 2로봇 herding 시뮬레이션 워크스페이스

turtle_project 최신본(plan A/B herding 내장) + sim 전용 리소스(patrol_waypoints2).
빌드: `colcon build --symlink-install --packages-select turtle_interfaces turtle_project`

포함 안 된 upstream 패키지 (src/에 직접 클론 필요):
turtlebot4, turtlebot4_simulator, turtlebot4_desktop, turtlebot4_tutorials,
m-explore-ros2, mini_turtle4

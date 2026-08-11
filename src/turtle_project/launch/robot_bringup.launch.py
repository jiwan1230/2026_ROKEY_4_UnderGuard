"""localization -> (active 확인) -> navigation 순으로 엄격히 순차 기동.

    ros2 launch turtle_project robot_bringup.launch.py                  # robot4
    ros2 launch turtle_project robot_bringup.launch.py namespace:=/robot6

map_server/amcl(localization)이 active된 뒤에야 navigation 프로세스를 띄운다.
코스트맵이 맵(map_server)과 map->odom TF(amcl)를 필요로 하기 때문. WiFi에서
lifecycle autostart가 자주 타임아웃 나므로 nav2_activate.sh로 단계별로 밀어
올린다 — localization을 active까지 민 뒤(그 스크립트 종료가 신호) navigation.

mini_turtle4의 robot_bringup을 turtle_project용으로 복제한 것 — 맵/스크립트/
nav2.yaml을 이 패키지 안에 담아 단독으로 뜬다.

뜬 뒤 할 일:
  1. RViz에서 2D Pose Estimate로 초기 위치 지정 (매번 필요 — AMCL이 기억 못 함)
  2. 다른 터미널에서 app 노드 실행 (robot_agent 등, params-file로 namespace 지정)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

NAMESPACE = '/robot4'
# 순찰 waypoint와 같은 프레임의 실제 맵 — 설치본 share에서 참조 (재-SLAM 하면
# resource의 파일을 갱신하고 colcon build). map:= 로 오버라이드.
ROOM_MAP = os.path.join(
    get_package_share_directory('turtle_project'), 'resource', 'room_map.yaml')
LOCAL_NODES = 'map_server amcl'
NAV_NODES = ('controller_server smoother_server planner_server '
             'behavior_server bt_navigator waypoint_follower velocity_smoother')


def generate_launch_description():
    nav = get_package_share_directory('turtlebot4_navigation')
    viz = get_package_share_directory('turtlebot4_viz')
    mine = get_package_share_directory('turtle_project')
    ns = LaunchConfiguration('namespace')
    args = {'namespace': ns}.items()
    script = PathJoinSubstitution([mine, 'scripts', 'nav2_activate.sh'])

    def include(pkg, name, extra=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, 'launch', name])),
            launch_arguments=list(args) + (extra or []))

    def activate(nodes):
        # ns(=/robot4)를 그대로 넘긴다 — 스크립트가 앞 슬래시를 벗겨 쓴다.
        return ExecuteProcess(
            cmd=['bash', script, ns, '90', nodes], output='screen')

    # 1) localization 띄우고 active될 때까지 민다.
    localization = include(nav, 'localization.launch.py',
                           [('map', LaunchConfiguration('map'))])
    activate_local = activate(LOCAL_NODES)

    # 2) localization active(=activate_local 종료) 뒤에야 navigation을 띄운다.
    navigation = include(nav, 'nav2.launch.py',
                         [('params_file',
                           PathJoinSubstitution([mine, 'config', 'nav2.yaml']))])
    activate_nav = activate(NAV_NODES)

    def on_local_done(event, _ctx):
        if event.returncode == 0:
            return [navigation, activate_nav]
        return [LogInfo(msg='localization 활성화 실패 — navigation 시작 안 함. '
                            '위 에러를 확인하세요.')]

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=NAMESPACE),
        DeclareLaunchArgument('map', default_value=ROOM_MAP),
        localization,
        include(viz, 'view_robot.launch.py'),
        activate_local,
        RegisterEventHandler(OnProcessExit(target_action=activate_local,
                                           on_exit=on_local_done)),
    ])

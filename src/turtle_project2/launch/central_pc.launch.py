"""중앙 PC 노드 묶음: central_node, db_node, webcam_node, rat_herding_node(배관) +
쥐몰이 알고리즘 본체(herding_node) — plan 인자로 고른다 (둘 다 turtle_project 내장).

rat_herding_node는 fleet 프로토콜(String)/TF를 herding_node가 원하는
PoseStamped 3종으로 변환·중계만 한다 — 실제 HerdingCore.step()은 herding_node
프로세스 안에서 돈다(resource/<plan별>.yaml로 튜닝됨).

plan:=a (기본) -> turtle_project.herding      (robot B만 몰이, robot A는 detector가 추적)
plan:=b        -> turtle_project.herding_dual (robot A/B 둘 다 알고리즘이 몬다)
plan:=b면 robot A의 detector도 -p plan:=b 로 띄워야 한다 (추적 goal 발행 끔).
노드 이름은 양쪽 다 'herding_controller' 고정 — rat_herding_node가 relay하는
토픽 경로(herding_controller/...)가 plan과 무관하게 같아야 하기 때문.

  ros2 launch turtle_project central_pc.launch.py            # plan A
  ros2 launch turtle_project central_pc.launch.py plan:=b    # plan B (dual)

로봇 PC 쪽 노드(camera_node/detector_node/trap_check_node/robot_agent)는
로봇마다 별도로 띄우는 launch가 따로 필요하다(여기 범위 아님, docs/run.md 참고).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# plan -> (실행파일, 파라미터 yaml). 둘 다 turtle_project 패키지 내장.
PLAN = {'a': ('herding_node', 'herding_params.yaml'),
        'b': ('herding_dual_node', 'herding_dual_params.yaml')}


def _herding_node(context, *_args, **_kwargs):
    plan = LaunchConfiguration('plan').perform(context)
    if plan not in PLAN:
        raise RuntimeError(f"알 수 없는 plan '{plan}' — 'a' 또는 'b'만 지원")
    exe, yaml_name = PLAN[plan]
    params = os.path.join(get_package_share_directory('turtle_project'),
                          'resource', yaml_name)
    # /map: central엔 전역 맵 토픽이 없다 — robot4의 map_server 것을 쓴다
    # (두 로봇 같은 맵). 없어도 돌지만 장애물 없음 취급이라 벽 뚫는 경로가 나온다.
    return [Node(package='turtle_project', executable=exe,
                 name='herding_controller', parameters=[params],
                 remappings=[('/map', '/robot4/map')])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'plan', default_value='a',
            description="쥐몰이: 'a'=단일 blocker(B만), 'b'=dual(A/B 둘 다 알고리즘)"),
        Node(package='turtle_project', executable='central_node', name='central_node'),
        Node(package='turtle_project', executable='db_node', name='db_node'),
        Node(package='turtle_project', executable='webcam_node', name='webcam_node'),
        Node(package='turtle_project', executable='rat_herding_node', name='rat_herding_node'),
        OpaqueFunction(function=_herding_node),
    ])

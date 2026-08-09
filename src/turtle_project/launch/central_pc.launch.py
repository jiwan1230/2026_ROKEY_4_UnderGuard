"""중앙 PC 노드 묶음: central_node, db_node, webcam_node, rat_herding_node(배관) +
쥐몰이 알고리즘 본체(herding_node) — plan 인자로 패키지를 고른다.

rat_herding_node는 fleet 프로토콜(String)/TF를 herding_node가 원하는
PoseStamped 3종으로 변환·중계만 한다 — 실제 HerdingCore.step()은 herding_node
프로세스 안에서 돈다(<plan 패키지>/config/herding_params.yaml로 튜닝됨).

plan:=a (기본) -> herding_controller (robot B만 몰이, robot A는 detector가 추적)
plan:=b        -> herding_controller_dual (robot A/B 둘 다 알고리즘이 몬다)
두 패키지 모두 실행 파일명이 herding_node라 노드 이름은 양쪽 다 'herding_controller'로
고정한다 — rat_herding_node가 relay하는 토픽 경로(herding_controller/...)가 plan과
무관하게 같아야 하기 때문.

  ros2 launch turtle_project central_pc.launch.py plan:=a
  ros2 launch turtle_project central_pc.launch.py plan:=b

로봇 PC 쪽 노드(camera_node/detector_node/trap_check_node/robot_agent)는
로봇마다 별도로 띄우는 launch가 따로 필요하다(여기 범위 아님, docs/run.md 참고).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PLAN_PACKAGE = {'a': 'herding_controller', 'b': 'herding_controller_dual'}


def _herding_node(context, *_args, **_kwargs):
    plan = LaunchConfiguration('plan').perform(context)
    pkg = PLAN_PACKAGE.get(plan)
    if pkg is None:
        raise RuntimeError(f"알 수 없는 plan '{plan}' — 'a' 또는 'b'만 지원")
    params = os.path.join(get_package_share_directory(pkg), 'config', 'herding_params.yaml')
    return [Node(package=pkg, executable='herding_node', name='herding_controller',
                 parameters=[params])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'plan', default_value='a',
            description="쥐몰이 알고리즘: 'a'=herding_controller(단일 blocker), "
                        "'b'=herding_controller_dual(로봇 2대 모두 알고리즘 제어)"),
        Node(package='turtle_project', executable='central_node', name='central_node'),
        Node(package='turtle_project', executable='db_node', name='db_node'),
        Node(package='turtle_project', executable='webcam_node', name='webcam_node'),
        Node(package='turtle_project', executable='rat_herding_node', name='rat_herding_node'),
        OpaqueFunction(function=_herding_node),
    ])

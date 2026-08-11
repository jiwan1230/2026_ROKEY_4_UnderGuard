"""로봇 1대 전체 노드 — bringup(Nav2) + detector + trap_check + robot_agent.

docs/run.md의 로봇 터미널 4개를 런치 하나로 묶는다. robot:= 인자로 어느
로봇인지 정한다 (네임스페이스·params yaml이 같이 정해진다).

  ros2 launch turtle_project robot_nodes.launch.py robot:=robot4
  ros2 launch turtle_project robot_nodes.launch.py robot:=robot6

bringup이 뜨면 RViz 2D Pose Estimate로 초기위치를 잡아야 하는 건 그대로다.
/tf remap 이유는 docs/run.md 참고 (TransformListener가 절대경로 /tf만 구독).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TF_REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def _nodes(context, *_args, **_kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    share = get_package_share_directory('turtle_project')
    ns = f'/{robot}'
    nodes = []
    # nav2:=false — 이미 bringup(nav2)이 떠 있는 PC에서 우리 노드만 추가로 띄울
    # 때. nav2를 이중 실행하면 amcl·lifecycle manager가 충돌해 기존
    # localization까지 망가진다.
    if LaunchConfiguration('nav2').perform(context).lower() != 'false':
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'robot_bringup.launch.py')),
            launch_arguments=[('namespace', ns)]))
    return nodes + [
        Node(package='turtle_project', executable='detector_node',
             namespace=ns, remappings=TF_REMAPS),
        Node(package='turtle_project', executable='trap_check_node',
             namespace=ns, remappings=TF_REMAPS),
        Node(package='turtle_project', executable='robot_agent', namespace=ns,
             parameters=[os.path.join(share, 'config', f'{robot}.yaml')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='robot4',
                              description="'robot4' 또는 'robot6'"),
        DeclareLaunchArgument('nav2', default_value='true',
                              description="'false'면 bringup(nav2) 제외 — "
                                          "이미 떠 있을 때 이중 실행 방지"),
        OpaqueFunction(function=_nodes),
    ])

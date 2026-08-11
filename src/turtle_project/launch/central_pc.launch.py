"""중앙 PC 노드 묶음: central_node, db_node, webcam_node, rat_herding_node(배관) +
쥐몰이 알고리즘 본체(herding_node, turtle_project.herding — plan A 단일 blocker).

rat_herding_node는 fleet 프로토콜(String)/TF를 herding_node가 원하는
PoseStamped 3종으로 변환·중계만 한다 — 실제 HerdingCore.step()은 herding_node
프로세스 안에서 돈다(resource/herding_params.yaml로 튜닝됨).

노드 이름은 'herding_controller' 고정 — rat_herding_node가 relay하는 토픽
경로(herding_controller/...)가 이 이름을 전제한다.

  ros2 launch turtle_project central_pc.launch.py

로봇 PC 쪽 노드(camera_node/detector_node/trap_check_node/robot_agent)는
로봇마다 별도로 띄우는 launch가 따로 필요하다(여기 범위 아님, docs/run.md 참고).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('turtle_project'),
                          'resource', 'herding_params.yaml')
    return LaunchDescription([
        Node(package='turtle_project', executable='central_node', name='central_node'),
        Node(package='turtle_project', executable='db_node', name='db_node'),
        Node(package='turtle_project', executable='webcam_node', name='webcam_node'),
        Node(package='turtle_project', executable='rat_herding_node', name='rat_herding_node'),
        # /map: central엔 전역 맵 토픽이 없다 — robot4의 map_server 것을 쓴다
        # (두 로봇 같은 맵). 없어도 돌지만 장애물 없음 취급이라 벽 뚫는 경로가 나온다.
        Node(package='turtle_project', executable='herding_node',
             name='herding_controller', parameters=[params],
             remappings=[('/map', '/robot4/map')]),
    ])

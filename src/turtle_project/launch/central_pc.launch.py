"""중앙 PC 노드 묶음: central_node, db_node, webcam_node, rat_herding_node(배관) +
herding_controller 패키지의 herding_node(몰이 알고리즘 본체).

rat_herding_node는 fleet 프로토콜(String)/TF를 herding_node가 원하는
PoseStamped 3종으로 변환·중계만 한다 — 실제 HerdingCore.step()은 herding_node
프로세스 안에서 돈다(herding_controller/herding_params.yaml 파라미터로 튜닝됨).

로봇 PC 쪽 노드(camera_node/detector_node/trap_check_node/robot_agent)는
로봇마다 별도로 띄우는 launch가 따로 필요하다(여기 범위 아님).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

HERDING_PARAMS = os.path.join(
    get_package_share_directory('herding_controller'), 'config', 'herding_params.yaml')


def generate_launch_description():
    return LaunchDescription([
        Node(package='turtle_project', executable='central_node', name='central_node'),
        Node(package='turtle_project', executable='db_node', name='db_node'),
        Node(package='turtle_project', executable='webcam_node', name='webcam_node'),
        Node(package='turtle_project', executable='rat_herding_node', name='rat_herding_node'),
        Node(package='herding_controller', executable='herding_node', name='herding_controller',
             parameters=[HERDING_PARAMS]),
    ])

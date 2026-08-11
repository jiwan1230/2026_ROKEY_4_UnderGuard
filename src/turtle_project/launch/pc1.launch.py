"""PC1(중앙 + robot4) 원터치 실행 — central_pc + robot_nodes(robot4).

  export MYSQL_PASSWORD='...'   # db_node 필수 — 이 셸에서 먼저
  ros2 launch turtle_project pc1.launch.py

central은 대기 모드로 뜬다 — UI(Sysmon)의 '시스템 시작' 버튼(= system:START)
을 눌러야 순찰이 시작된다. UI는 별도 실행: Sysmon/backend/run_ros.sh.
bringup RViz가 뜨면 2D Pose Estimate로 robot4 초기위치부터 잡을 것.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(name, args=None):
    share = get_package_share_directory('turtle_project')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)),
        launch_arguments=args or [])


def generate_launch_description():
    return LaunchDescription([
        # nav2:=false — bringup(nav2)이 이미 떠 있으면 이중 실행 방지용으로 제외.
        DeclareLaunchArgument('nav2', default_value='true'),
        _include('robot_nodes.launch.py',
                 [('robot', 'robot4'), ('nav2', LaunchConfiguration('nav2'))]),
        _include('central_pc.launch.py'),
    ])

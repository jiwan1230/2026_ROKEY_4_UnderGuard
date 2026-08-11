"""PC2(robot6 전용) 원터치 실행 — robot_nodes(robot6)만.

  ros2 launch turtle_project pc2.launch.py

중앙 노드는 PC1에 있으므로 여기선 로봇 노드만 띄운다. bringup RViz가 뜨면
2D Pose Estimate로 robot6 초기위치부터 잡을 것.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('turtle_project')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'robot_nodes.launch.py')),
            launch_arguments=[('robot', 'robot6')]),
    ])

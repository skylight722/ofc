#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    # 공용 YAML은 stack_master의 share/config에 설치됨
    cfg = str(Path(get_package_share_directory('stack_master')) / 'config' / 'controller_enabled.yaml')

    return LaunchDescription([
        # TinyLidarNet
        Node(
            package='ofc',
            executable='tln_inference',     # ← setup.py의 console_scripts 이름
            name='tln_inference',           # ← YAML 키와 동일해야 함
            output='screen',
            parameters=[cfg],
        ),
        # Follow-The-Gap
        Node(
            package='ofc',
            executable='ftg',               # ← setup.py의 console_scripts 이름
            name='ftg_controller',          # ← YAML 키와 동일해야 함
            output='screen',
            parameters=[cfg],
        ),
        Node(
            package='ofc', executable='enabled_guard',
            name='enabled_guard', output='screen',
            parameters=[{
                'targets': ['tln_inference', 'ftg_controller'],
                'prefer':  'tln_inference',
            }],
        ),
    ])


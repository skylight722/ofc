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

        Node(
            package='ofc',
            executable='joy_controller',
            name='ofc_joy_controller',
            output='screen',
            parameters=[{
                'drive_topic': '/drive',
                'deadman_button': 4,     # PS5 L1
                'axis_steer': 3,         # PS5 오른스틱 X
                'scale_steer': 0.34,
                'deadzone': 0.05,
                'fixed_speed': 3.0,      # 항상 3 m/s로 주행
                'publish_hz': 50.0,
                'drive_publish_enabled': False,  # 시작 시 OFF, enabled_guard가 관리
            }],
        ),

        Node(
            package='ofc',
            executable='bag_recorder_sim',
            name='bag_recorder_sim',
            output='screen',
            parameters=[{
                'start_button': 1,
                'stop_button': 3,
            }],
        ),

    ])


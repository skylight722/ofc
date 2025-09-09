#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

def generate_launch_description():
    # ====== 0) 경로 설정 ======
    f1tenth_share = get_package_share_directory('f1tenth_stack')
    cfg_joy      = os.path.join(f1tenth_share, 'config', 'joy_teleop.yaml')  # joy_node 하드웨어 설정에 사용
    cfg_vesc     = os.path.join(f1tenth_share, 'config', 'vesc.yaml')
    cfg_sensors  = os.path.join(f1tenth_share, 'config', 'sensors.yaml')

    # ofc 공용 YAML (enabled 스위칭 등)
    cfg_ofc = str(Path(get_package_share_directory('stack_master')) / 'config' / 'controller_enabled.yaml')

    # 외부에서 파일 교체 가능하도록 런치 인자
    joy_arg     = DeclareLaunchArgument('joy_config',     default_value=cfg_joy,     description='joy_node config')
    vesc_arg    = DeclareLaunchArgument('vesc_config',    default_value=cfg_vesc,    description='vesc config')
    sensors_arg = DeclareLaunchArgument('sensors_config', default_value=cfg_sensors, description='sensors config')

    # ====== 1) 입력(조이스틱) ======
    # 하드웨어 조이스틱 이벤트 생성
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
        parameters=[LaunchConfiguration('joy_config')]
    )

    # ====== 2) 라이다/센서 ======
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[LaunchConfiguration('sensors_config')]
         env={'LD_LIBRARY_PATH': f'{urg2_lib}:' + os.environ.get('LD_LIBRARY_PATH', '')}
    )

    # ====== 3) VESC 체인  ======
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        output='screen',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    
    # 컨트롤러들이 곧장 "/ackermann_cmd"로 퍼블리시하도록 구성
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        output='screen',
        parameters=[LaunchConfiguration('vesc_config')]
        # remap 불필요: 컨트롤러 쪽을 /ackermann_cmd 로 맞춤
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        output='screen',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    # 필요 시만 사용
    # throttle_interpolator_node = Node(
    #     package='f1tenth_stack',
    #     executable='throttle_interpolator',
    #     name='throttle_interpolator',
    #     output='screen',
    #     parameters=[LaunchConfiguration('vesc_config')]
    # )

    # ====== 4) TF (실차 기본) ======
    # base_link -> laser (라이다 마운트 위치)
    static_tf_baselink_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        output='screen',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    # base_link -> imu (IMU 마운트 위치/자세) 
    static_tf_baselink_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        output='screen',
        arguments=['0.07', '0.0', '0.05', '0.0', '0.0', '0.7071068', '0.7071068', 'base_link', 'imu']
    )


    # ====== 5) 컨트롤러 (모두 /ackermann_cmd 로 직접 퍼블리시) ======
  
    tln_node = Node(
        package='ofc',
        executable='tln_inference',     # setup.py console_scripts
        name='tln_inference',
        output='screen',
        parameters=[cfg_ofc, {'drive_topic': '/ackermann_cmd'}]
    )
    ftg_node = Node(
        package='ofc',
        executable='ftg',                # setup.py console_scripts
        name='ftg_controller',
        output='screen',
        parameters=[cfg_ofc, {'drive_topic': '/ackermann_cmd'}]
    )
    enabled_guard = Node(
        package='ofc',
        executable='enabled_guard',
        name='enabled_guard',
        output='screen',
        parameters=[{
            'targets': ['tln_inference', 'ftg_controller'],
            'prefer':  'tln_inference',
        }]
    )
    # 수동 주행: /joy -> ofc_joy_controller -> /ackermann_cmd
    ofc_joy = Node(
        package='ofc',
        executable='joy_controller',
        name='ofc_joy_controller',
        output='screen',
        parameters=[{
            'drive_topic': '/ackermann_cmd',
            'deadman_button': 4,     # PS5 L1
            'axis_steer': 2,         # PS5 오른스틱 X
            'scale_steer': 0.34,
            'deadzone': 0.05,
            'fixed_speed': 2.5,
            'publish_hz': 50.0,
            'drive_publish_enabled': False,  # 시작 시 OFF, enabled_guard가 관리
        }]
    )

    # ====== 6) Bag 기록 ======
    bag_recorder = Node(
        package='ofc',
        executable='bag_recorder',
        name='bag_recorder',
        output='screen',
        parameters=[{
            'start_button': 2,
            'stop_button': 0,
        }]
    )

    # ====== 7) LaunchDescription ======
    return LaunchDescription([
        joy_arg, vesc_arg, sensors_arg,
        # I/O
        joy_node,
        urg_node,
        # VESC
        vesc_driver_node,
        ackermann_to_vesc_node,
        vesc_to_odom_node,
        # throttle_interpolator_node,  # 필요 시 활성화
        # TF
        static_tf_baselink_laser,
        static_tf_baselink_imu,
        # Controllers
        tln_node,
        ftg_node,
        enabled_guard,
        ofc_joy,
        # Bag recorder
        bag_recorder,
    ])


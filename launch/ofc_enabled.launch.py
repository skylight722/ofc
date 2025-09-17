#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ====== 0) 경로 설정 ======
    f1tenth_share = get_package_share_directory('f1tenth_stack')
    cfg_joy       = os.path.join(f1tenth_share, 'config', 'joy_teleop.yaml')  # joy_node 하드웨어 설정
    cfg_vesc      = os.path.join(f1tenth_share, 'config', 'vesc.yaml')
    
    
    # cfg_sensors   = os.path.join(f1tenth_share, 'config', 'sensors.yaml')

    # ofc 공용 YAML (enabled 스위칭 등)
    cfg_ofc = str(Path(get_package_share_directory('stack_master')) / 'config' / 'controller_enabled.yaml')

    # 외부에서 파일 교체 가능하도록 런치 인자
    joy_arg     = DeclareLaunchArgument('joy_config',     default_value=cfg_joy,     description='joy_node config')
    vesc_arg    = DeclareLaunchArgument('vesc_config',    default_value=cfg_vesc,    description='vesc config')
    
    # sensors_arg = DeclareLaunchArgument('sensors_config', default_value=cfg_sensors, description='sensors config')

    # ====== 1) 입력(조이스틱) ======
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
        parameters=[LaunchConfiguration('joy_config')]
    )

    # ====== 3) 라이다/센서 ======
    launch_urg_node2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('urg_node2'),
                'launch/urg_node2.launch.py'
            )
        )
    )

    # ====== 4) VESC 체인 (구동/오도메트리) ======
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        output='screen',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        output='screen',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        output='screen',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    # ====== 5) TF ======
    static_tf_baselink_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        output='screen',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    static_tf_baselink_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        output='screen',
        arguments=['0.07', '0.0', '0.05', '0.0', '0.0', '0.7071068', '0.7071068', 'base_link', 'imu']
    )
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )

    # ====== 6) 컨트롤러 (모두 /ackermann_cmd 로 퍼블리시) ======
    tln_node = Node(
        package='ofc',
        executable='tln_inference',
        name='tln_inference',
        output='screen',
        parameters=[cfg_ofc, {'drive_topic': '/ackermann_cmd'}]
    )
    ftg_node = Node(
        package='ofc',
        executable='ftg',
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
    ofc_joy = Node(
        package='ofc',
        executable='joy_controller',
        name='ofc_joy_controller',
        output='screen',
        parameters=[{
            'drive_topic': '/ackermann_cmd',
            'deadman_button': 4,
            'axis_steer': 2,
            'scale_steer': 0.34,
            'deadzone': 0.05,
            'fixed_speed': 2.5,
            'publish_hz': 50.0,
            'drive_publish_enabled': False,

            'tln_node_name': '/tln_inference',
            'vmax_inc_button': 3,
            'vmax_dec_button': 1,
            'vmin_inc_button': 9,
            'vmin_dec_button': 8,
            'tln_step': 0.5,
            'vmin_floor': 0.0,
            'vmax_ceil': 15.0
        }]
    )

    # ====== 7) Bag 기록 ======
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

    # ====== 8) LaunchDescription ======
    return LaunchDescription([
        joy_arg, 
        vesc_arg, 
        # I/O
        joy_node,
        launch_urg_node2, 
        # VESC
        vesc_driver_node,
        ackermann_to_vesc_node,
        vesc_to_odom_node,
        # TF
        static_tf_baselink_laser,
        static_tf_baselink_imu,
        static_tf_map_odom,
        # Controllers
        tln_node,
        ftg_node,
        enabled_guard,
        ofc_joy,
        # Bag recorder
        bag_recorder,
    ])


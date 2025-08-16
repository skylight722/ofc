from setuptools import setup
import os
from glob import glob

package_name = 'tiny_lidar_net'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'numpy',
        'tensorflow',
        'tflite-runtime',
        'scipy',
    ],
    zip_safe=True,
    maintainer='misys',
    maintainer_email='misys@todo.todo',
    description='TinyLidarNet inference for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 기존 inference 노드
            'inference_node = tiny_lidar_net.inference_node:main',
            # 추가: odom_to_ackermann 노드
            'odom_to_ackermann = tiny_lidar_net.odom_to_ackermann:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*.py')),
        ('share/' + package_name + '/models',
         glob('tiny_lidar_net/models/*')),
    ],
)


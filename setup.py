from setuptools import setup
import os
from glob import glob

package_name = 'ofc'

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
    description='ofc for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 기존 inference 노드
            'tln_inference = ofc.inference_node:main',
            'ftg = ofc.ftg:main',
            'joy_controller = ofc.joy_controller:main',
            #"drive_mux = ofc.drive_mux:main",
            "enabled_guard  = ofc.enabled_guard:main",
            "bag_recorder = ofc.bag_recorder:main",
            "bag_recorder_sim = ofc.bag_recorder_sim:main",
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*.py')),
        ('share/' + package_name + '/models',
         glob('ofc/models/*')),
    ],
)


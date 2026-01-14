import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gps'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'paths'), glob(os.path.join('paths', '*.csv'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools', 'utm', 'pandas'],
    zip_safe=True,
    maintainer='hong',
    maintainer_email='hong@todo.todo',
    description='Package for GPS sensor fusion and data processing for Pure Pursuit.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # [수정됨] .scripts 경로를 모두 제거하여 실제 파일 위치와 일치시킵니다.
            'path_manager_node = gps.path_manager_node:main',
            'sensor_fusion_node = gps.sensor_fusion_node:main',
            'sensor_fusion_node2 = gps.sensor_fusion_node2:main',
            'sensor_fusion_node3 = gps.sensor_fusion_node3:main',
            'path_recorder = gps.path_recorder:main',
            'path_publisher = gps.path_publisher:main',
            'mission_planner = scripts.mission_planner:main',
            'odom_tf_broadcaster = gps.odom_tf_broadcaster:main',
        ],
    },
)

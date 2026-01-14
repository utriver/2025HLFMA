from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vision_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeoungjun',
    maintainer_email='yeoungjun@example.com',
    description='Vision System with YOLOv8 for Autonomous Driving',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_publisher = vision_system.vision_publisher:main',
            'vision_subscriber = vision_system.vision_subscriber:main',
        ],
    },
)

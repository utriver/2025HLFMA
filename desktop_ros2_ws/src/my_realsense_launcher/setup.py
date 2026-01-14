import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_realsense_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 아래 줄이 launch 파일을 install 폴더로 복사하라는 명령입니다.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeoungjun', # 이 부분은 본인 정보에 맞게 수정하셔도 됩니다.
    maintainer_email='yeoungjun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'camera_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vision_test.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeoungjun',
    maintainer_email='yeoungjun@todo.todo',
    description='Camera test package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_test_publisher = camera_test.vision_test_publisher:main',
            'vision_test_subscriber = camera_test.vision_test_subscriber:main',
        ],
    },
)

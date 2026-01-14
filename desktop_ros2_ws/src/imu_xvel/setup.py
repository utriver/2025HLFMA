from setuptools import setup

package_name = 'imu_xvel'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name], 
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu_xvel.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryu',
    maintainer_email='ryu@todo.todo',
    description='IMU x-axis velocity integrator (ament_python).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'xvel_node = imu_xvel.imu_xvel_integrator:main',
        ],
    },
)


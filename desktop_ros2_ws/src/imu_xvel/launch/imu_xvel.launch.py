from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_xvel',
            executable='xvel_node',
            name='imu_xvel_node',
            output='screen',
            parameters=[{
                'imu_topic': '/imu/data',
                'publish_odom': False,
                'vel_topic': '/xvel/twist',
                'odom_topic': '/xvel/odom',
                'frame_id_odom': 'odom',
                'child_frame_id': 'base_link',
                'use_free_accel': False,
                'gravity': 9.80665,
                'bias_tau': 3.0,
                'zupt_enabled': True,
                'zupt_gyro_thresh': 0.15,
                'zupt_accel_thresh': 0.15,
                'max_dt': 0.05,
                'min_dt': 1e-4,
            }]
        )
    ])

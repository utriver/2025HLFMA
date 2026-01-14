#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pd_control',
            executable='pd_controller_node',
            name='pd_controller_node',
            output='screen',
            parameters=[
                # PD 게인 설정
                {'kp': 2.0},
                {'kd': 0.5},
                # 목표 속도 설정 (m/s)
                {'target_velocity': 5.0}
            ]
        )
    ])

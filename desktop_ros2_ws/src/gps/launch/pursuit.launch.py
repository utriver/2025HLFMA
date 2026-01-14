# pursuit.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ... (path_publisher, mission_planner 노드는 동일)
        Node(
            package='gps',
            executable='path_publisher',
            name='path_publisher_node',
            output='screen'
        ),
        Node(
            package='gps',
            executable='mission_planner',
            name='mission_planner_node',
            output='screen'
        ),
        # ✅ [수정] Pure Pursuit 노드의 파라미터를 Lf_straight와 k_c로 변경
        Node(
            package='pure_pursuit_planner',
            executable='pure_pursuit_planner',
            name='pure_pursuit_node',
            output='screen',
            parameters=[{
                # --- 새로운 Lf 제어 파라미터 ---
                'Lf_straight': 5.0, # 직선로 기본 Lf
                'k_c': 10.0,         # 곡률 가중치
                'Lf_min': 1.5,      # 최소 Lf 안전장치

                # --- 속도 및 기타 파라미터 ---
                'minVelocity': 0.0,
                'maxVelocity': 2.6, 
                'maxCurvature': 2.0, # 이 값은 이제 속도제어에 사용되지 않음
            }],
            remappings=[
                ('odom', '/vehicle_odometry'),
                ('tgt_path', '/path')
            ]
        ),
    ])

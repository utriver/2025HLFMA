# vision_test.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
    return LaunchDescription([
        # 'test_vision' 네임스페이스 그룹 시작
        GroupAction(
            actions=[
                PushRosNamespace('test_vision'),

                Node(
                    package='camera_test',
                    executable='vision_test_publisher',
                    name='vision_test_publisher_node',
                    output='screen'
                ),
                
                Node(
                    package='camera_test',
                    executable='vision_test_subscriber',
                    name='vision_test_subscriber_node',
                    output='screen'
                ),
            ]
        )
    ])

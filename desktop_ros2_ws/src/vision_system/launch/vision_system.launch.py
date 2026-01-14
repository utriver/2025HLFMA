# # vision_system.launch.py

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, GroupAction
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node, PushRosNamespace

# def generate_launch_description():
#     return LaunchDescription([
#         # main_vision 시스템에서 사용할 카메라 시리얼 번호 선언
#         DeclareLaunchArgument(
#             'camera_serial',
#             default_value='313522303259',
#             description='Serial number of the camera for the main vision system'
#         ),
        
#         # 1. 메인 비전 시스템 그룹 ('main_vision' 네임스페이스)
#         GroupAction(
#             actions=[
#                 PushRosNamespace('main_vision'),

#                 Node(
#                     package='vision_system',
#                     executable='vision_publisher',
#                     name='vision_publisher_node',
#                     parameters=[{
#                         'camera_serial': LaunchConfiguration('camera_serial')
#                     }],
#                     output='screen'
#                 ),
                
#                 Node(
#                     package='vision_system', 
#                     executable='vision_subscriber',
#                     name='vision_subscriber_node',
#                     output='screen'
#                 )
#             ]
#         ),

#         # =================================================================
#         # 2. 추가된 테스트 비전 시스템 그룹 ('test_vision' 네임스페이스)
#         # =================================================================
#         GroupAction(
#             actions=[
#                 PushRosNamespace('test_vision'),

#                 Node(
#                     # vision_test.launch.py를 참고하여 패키지 이름을 'camera_test'로 가정합니다.
#                     # 만약 패키지 이름이 다르다면 이 부분을 수정해야 합니다.
#                     package='camera_test',
#                     executable='vision_test_publisher',
#                     name='vision_test_publisher_node',
#                     output='screen'
#                 ),
                
#                 Node(
#                     # vision_test.launch.py를 참고하여 패키지 이름을 'camera_test'로 가정합니다.
#                     # 만약 패키지 이름이 다르다면 이 부분을 수정해야 합니다.
#                     package='camera_test',
#                     executable='vision_test_subscriber',
#                     name='vision_test_subscriber_node',
#                     output='screen'
#                 ),
#             ]
#         )
#     ])
# ~/ros2_ws/src/vision_system/launch/vision_system.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    vision_publisher와 vision_subscriber 노드를 실행하는 Launch 파일.
    vision_publisher에 camera_serial 파라미터를 문자열로 전달합니다.
    """
    
    # vision_publisher 노드 설정
    vision_publisher_node = Node(
        package='vision_system',
        executable='vision_publisher',
        name='vision_publisher_node',
        namespace='/main_vision',
        output='screen',
        emulate_tty=True,
        parameters=[{
            # camera_serial 값을 따옴표로 감싸 문자열로 전달
            'camera_serial': '313522303259'
        }]
    )

    # vision_subscriber 노드 설정
    vision_subscriber_node = Node(
        package='vision_system',
        executable='vision_subscriber',
        name='vision_subscriber_node',
        namespace='/main_vision',
        output='screen',
        emulate_tty=True
    )

    # LaunchDescription에 두 노드를 추가하여 반환
    return LaunchDescription([
        vision_publisher_node,
        vision_subscriber_node
    ])
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 템플릿 런치 파일의 경로를 지정합니다.
    camera_launch_file = os.path.join(
        get_package_share_directory('my_realsense_launcher'),
        'launch',
        'realsense_camera.launch.py'
    )

    # --- TF 설정: 로봇 중심(base_link)에서 각 카메라까지의 상대 위치 ---
    # 사용자의 실제 환경에 맞게 x, y, z, yaw, pitch, roll 값을 수정해야 합니다.
    # 아래는 예시입니다: 전방 카메라는 10cm 앞에, 후방 카메라는 10cm 뒤에 180도 회전하여 장착

    # base_link -> front_camera_link TF 발행
    static_tf_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_front',
        arguments=['0.82', '0.0', '0.75', '0.0', '0.0', '0.0', 'base_link', 'front_camera_link']
    )

    # base_link -> rear_camera_link TF 발행
    static_tf_rear = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_rear',
        arguments=['0.16', '0.0', '0.75', '0.0', '0.0', '3.14159', 'base_link', 'rear_camera_link']
    )

    # --- 카메라 노드 실행 ---
    # 전방 카메라 실행 (본인의 시리얼 번호로 수정!)
    front_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file),
        launch_arguments={
            'camera_name': 'front_camera',
            'serial_no': "'313522303259'"  # <--- 여기에 전방 카메라 시리얼 번호를 넣으세요! (작은따옴표 유지)
        }.items()
    )

    # 후방 카메라 실행 (본인의 시리얼 번호로 수정!)
    rear_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file),
        launch_arguments={
            'camera_name': 'rear_camera',
            'serial_no': "'324422300490'"  # <--- 여기에 후방 카메라 시리얼 번호를 넣으세요!
        }.items()
    )

    return LaunchDescription([
        static_tf_front,
        static_tf_rear,
        front_camera,
        rear_camera
    ])

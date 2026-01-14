from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    주행 경로 녹화를 위한 노드들을 실행하는 런치 파일입니다.
    """
    return LaunchDescription([
        # 주행 경로를 녹화하는 'path_recorder' 노드
        Node(
            package='gps',
            executable='path_recorder',
            name='path_recorder_node',
            output='screen'
        ),

        # GPS, IMU 등 센서 데이터를 융합하여 '/vehicle_odometry'를 발행하는 노드.
        # path_recorder가 이 토픽을 구독하여 경로를 기록합니다.
        #Node(
        #    package='gps',
        #    executable='sensor_fusion_node',
        #    name='sensor_fusion_node',
        #    output='screen'
        #),

        # 여기에 GPS, IMU 드라이버 등 sensor_fusion_node가
        # 필요로 하는 다른 노드들을 추가할 수 있습니다.
    ])


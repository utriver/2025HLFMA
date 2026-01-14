import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gps_share_dir = get_package_share_directory('gps') # gps 패키지 디렉토리 경로

    # 필요한 설정 파일들 (navsat_config.yaml은 삭제됨)
    ublox_config = os.path.join(gps_share_dir, 'config', 'ublox_config.yaml')
    ntrip_config = os.path.join(gps_share_dir, 'config', 'ntrip_config.yaml')

    return LaunchDescription([
        # 1. Ublox GPS 드라이버 노드 (수정됨)
        Node(
            package='ublox_gps', 
            executable='ublox_gps_node', 
            name='ublox_gps_node', 
            output='screen', 
            parameters=[ublox_config]
        ),
        
        # 2. NTRIP 클라이언트 노드 (변경 없음)
        Node(
            package='ntrip_client', 
            executable='ntrip_ros.py', 
            name='ntrip_client_node', 
            output='screen', 
            parameters=[ntrip_config], 
            remappings=[('/fix', '/ublox_gps_node/fix')]
        ),
        
        # 3. Static TF 발행 노드들 (변경 없음)
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_link_to_imu_link', 
            arguments=['0.67', '0', '0.465', '0', '0', '0', 'base_link', 'imu_link']
        ),
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_link_to_gps', 
            arguments=['0.75', '0', '0.122', '0', '0', '0', 'base_link', 'gps']
        ),
        
        # 4. 새로 추가한 센서 융합 노드 (변경 없음)
        Node(
            package='gps',
            executable='sensor_fusion_node',
            name='sensor_fusion_node',
            output='screen',
            remappings=[
                ('gps/fix', '/ublox_gps_node/fix'),
                ('gps/velocity', '/ublox_gps_node/fix_velocity'),
                ('imu/data', '/imu/data'),
                ('gps/navpvt', '/ublox_gps_node/navpvt')
            ]
        )
    ])

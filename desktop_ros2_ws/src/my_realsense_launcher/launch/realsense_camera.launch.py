import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 이 런치 파일을 실행할 때 넘겨줄 인자들을 선언합니다.
    # camera_name: 카메라의 별명 (예: front_camera)
    # serial_no: 카메라의 고유 시리얼 번호
    declare_camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Namespace for the camera node and topics')

    declare_serial_no_arg = DeclareLaunchArgument(
        'serial_no',
        default_value='',
        description='The serial number of the camera')

    # realsense2_camera 패키지의 기본 런치 파일 경로를 찾습니다.
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py')

    # IncludeLaunchDescription을 사용하여 기본 런치 파일을 포함시킵니다.
    # 여기서 launch_arguments를 통해 값을 '주입'해주는 것이 핵심입니다.
    realsense_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'pointcloud.enable': 'true',
            'camera_name': LaunchConfiguration('camera_name'),
            'serial_no': LaunchConfiguration('serial_no'),
        }.items()
    )

    return LaunchDescription([
        declare_camera_name_arg,
        declare_serial_no_arg,
        realsense_camera_node
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # 기존 realsense2_camera 패키지의 launch 파일을 가져옵니다.
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py')

    # 기존 런치 파일을 포함시키면서, 우리가 원하는 옵션을 덮어씌웁니다.
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_path),
            launch_arguments={'pointcloud.enable': 'true'}.items()
        )
    ])

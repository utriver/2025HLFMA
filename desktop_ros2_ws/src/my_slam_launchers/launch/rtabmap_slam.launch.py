import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # rtabmap_ros 패키지의 launch 파일을 포함
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rtabmap_ros'), 'launch', 'rtabmap.launch.py')
        ]),
        # 런치 파일에 전달할 인자 설정
        launch_arguments={'rviz': 'true'}.items()
    )

    return LaunchDescription([
        rtabmap_launch
    ])


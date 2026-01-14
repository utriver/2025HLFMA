import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 우리가 만든 dual_camera.launch.py 파일을 포함시킵니다.
    dual_camera_launch_file = os.path.join(
        get_package_share_directory('my_realsense_launcher'),
        'launch',
        'dual_camera.launch.py'
    )
    
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dual_camera_launch_file)
    )

    # 2. RTAB-Map 노드를 설정합니다.
    #    'rtabmap' 실행 파일은 'rtabmap_slam' 패키지에 있습니다.
    rtabmap_node = Node(
        package='rtabmap_slam',  # <-- [수정!] 'rtabmap_ros' -> 'rtabmap_slam'
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': False,
            'subscribe_rgbd': True,
            'rgbd_cameras': 2,
            'approx_sync': True,
            'queue_size': 10,
            'Reg/Strategy': '1',
            'Icp/VoxelSize': '0.05',
            'Icp/MaxCorrespondenceDistance': '0.1',
        }],
        remappings=[
            ('rgbd_image0', '/front_camera/rgbd_image'),
            ('rgbd_image1', '/rear_camera/rgbd_image'),
        ]
    )

    # 3. Odometry 노드를 설정합니다.
    #    'rgbd_odometry' 실행 파일은 'rtabmap_odom' 패키지에 있습니다.
    odometry_node = Node(
        package='rtabmap_odom',  # <-- [수정!] 'rtabmap_ros' -> 'rtabmap_odom'
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'approx_sync': True,
            'queue_size': 10,
        }],
        remappings=[
            ('rgbd_image', '/front_camera/rgbd_image'),
        ]
    )

    # 4. RTAB-Map용 RViz 실행 노드
    #    'rtabmap_viz' 실행 파일은 'rtabmap_viz' 패키지에 있습니다.
    rtabmap_viz_node = Node(
        package='rtabmap_viz',  # <-- [수정!] 'rtabmap_ros' -> 'rtabmap_viz'
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_rgbd': True,
            'approx_sync': True,
            'queue_size': 10,
        }],
        remappings=[
            ('rgbd_image', '/front_camera/rgbd_image'),
        ]
    )

    return LaunchDescription([
        camera_launch,
        odometry_node,
        rtabmap_node,
        rtabmap_viz_node
    ])

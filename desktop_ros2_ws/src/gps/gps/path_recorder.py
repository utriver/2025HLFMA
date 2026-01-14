import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import os
import csv
from ament_index_python.packages import get_package_share_directory

class PathRecorder(Node):
    """
    차량의 주행 경로를 기록하여 CSV 파일로 저장하는 노드입니다.
    'record', 'stop', 'save' 명령을 통해 경로 기록을 제어합니다.
    """
    def __init__(self):
        super().__init__('path_recorder_node')

        # 차량의 현재 위치(Odometry)를 받기 위한 구독
        self.odom_sub = self.create_subscription(Odometry, '/vehicle_odometry', self.odom_callback, 10)

        # 외부 명령 ('record', 'stop', 'save')을 받기 위한 구독
        self.command_sub = self.create_subscription(String, '/path_command', self.command_callback, 10)

        # 경로 기록을 위한 주기적인 타이머 (0.1초마다)
        self.record_timer = self.create_timer(0.1, self.record_waypoint)

        self.current_pose = None
        self.recorded_poses = []
        self.is_recording = False
        self.min_dist_between_points = 0.5  # waypoint 사이의 최소 거리 (미터)

        # CSV 파일을 저장할 경로 설정 (패키지 내 share/paths 폴더)
        try:
            self.save_directory = os.path.join(get_package_share_directory('gps'), 'paths')
        except Exception:
            # 기본 경로 설정 (패키지를 찾을 수 없을 경우)
            self.save_directory = os.path.join(os.getcwd(), 'paths')
            
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)
            self.get_logger().info(f"Created directory: {self.save_directory}")

        self.get_logger().info('Path Recorder Node is ready.')
        self.get_logger().info("Available commands on /path_command: record, stop, save <filename.csv>")

    def odom_callback(self, msg):
        """Odometry 메시지로부터 현재 위치(pose)를 업데이트합니다."""
        self.current_pose = msg.pose.pose

    def command_callback(self, msg):
        """외부 명령을 처리합니다."""
        command_parts = msg.data.split()
        command = command_parts[0]

        if command == 'record':
            if not self.is_recording:
                self.is_recording = True
                self.recorded_poses.clear()  # 새 녹화 시작 시 이전 경로 데이터 초기화
                self.get_logger().info('>> Started recording path...')
        elif command == 'stop':
            if self.is_recording:
                self.is_recording = False
                self.get_logger().info(f'>> Stopped recording. Total {len(self.recorded_poses)} waypoints recorded.')
        elif command == 'save' and len(command_parts) > 1:
            self.is_recording = False # 저장은 녹화 중지를 의미
            self.save_path_to_csv(command_parts[1])
        else:
            self.get_logger().warn(f"Unknown or incomplete command: {msg.data}")

    def record_waypoint(self):
        """타이머에 의해 주기적으로 호출되어 waypoint를 기록합니다."""
        if not self.is_recording or self.current_pose is None:
            return

        # 마지막으로 기록된 점과 충분히 멀리 떨어져 있는지 확인 후 추가
        if not self.recorded_poses or self.is_far_enough(self.current_pose):
            self.recorded_poses.append(self.current_pose)
            self.get_logger().info(f'Waypoint added. Total: {len(self.recorded_poses)}', throttle_duration_sec=1.0)

    def is_far_enough(self, new_pose):
        """새로운 waypoint가 마지막 waypoint로부터 충분히 멀리 떨어져 있는지 계산합니다."""
        last_pose = self.recorded_poses[-1]
        dist = math.hypot(new_pose.position.x - last_pose.position.x,
                          new_pose.position.y - last_pose.position.y)
        return dist > self.min_dist_between_points

    def save_path_to_csv(self, filename):
        """기록된 경로를 CSV 파일로 저장합니다."""
        if not self.recorded_poses:
            self.get_logger().warn('No path to save.')
            return

        # 파일명에 .csv 확장자가 없으면 추가
        if not filename.endswith('.csv'):
            filename += '.csv'
            
        filepath = os.path.join(self.save_directory, filename)
        
        try:
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                # 헤더 작성
                writer.writerow(['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
                # 데이터 작성
                for pose in self.recorded_poses:
                    pos = pose.position
                    ori = pose.orientation
                    writer.writerow([pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w])
            self.get_logger().info(f'Path with {len(self.recorded_poses)} waypoints saved to {filepath}')
            self.recorded_poses.clear() # 저장 후 데이터 초기화
        except IOError as e:
            self.get_logger().error(f"Failed to save path file: {e}")


def main(args=None):
    rclpy.init(args=args)
    path_recorder = PathRecorder()
    rclpy.spin(path_recorder)
    path_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import os
from ament_index_python.packages import get_package_share_directory
import csv # 더 안정적인 CSV 처리를 위해 csv 모듈 사용

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher_node')
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.command_sub = self.create_subscription(String, '/path_command', self.command_callback, 10)
        
        self.path_directory = os.path.join(get_package_share_directory('gps'), 'paths')
        self.get_logger().info('Path Publisher Node is ready.')
        self.get_logger().info(f"Looking for path files in: {self.path_directory}")
        self.get_logger().info("Usage: ros2 topic pub /path_command std_msgs/msg/String \"data: 'load <filename.csv>'\" --once")

    def command_callback(self, msg):
        command_parts = msg.data.split()
        command = command_parts[0]

        if command == 'load' and len(command_parts) > 1:
            self.load_path_from_csv(command_parts[1])
        else:
            self.get_logger().warn(f"Publisher ignores this command: {msg.data}")

    def load_path_from_csv(self, filename):
        filepath = os.path.join(self.path_directory, filename)

        if not os.path.exists(filepath):
            self.get_logger().error(f'Path file not found: {filepath}')
            return

        loaded_path = Path()
        loaded_path.header.stamp = self.get_clock().now().to_msg()
        loaded_path.header.frame_id = 'odom' # 또는 'map'

        try:
            with open(filepath, 'r', newline='') as f:
                reader = csv.reader(f)
                # 헤더가 있는지 확인하고 건너뛰기
                try:
                    # 첫 줄을 읽고 숫자로 변환 시도
                    first_row = next(reader)
                    [float(p) for p in first_row[:3]] # x, y, curvature가 숫자인지 확인
                    # 성공하면 다시 처음으로
                    f.seek(0)
                except (ValueError, IndexError):
                    self.get_logger().info(f"Detected and skipped header in {filename}")
                
                for row in reader:
                    if len(row) < 3: continue # x, y, curvature 데이터가 없는 줄은 건너뜀

                    pose = PoseStamped()
                    pose.header = loaded_path.header
                    
                    # [수정됨] x, y, curvature 값을 CSV 파일에서 직접 읽어옴
                    pose.pose.position.x = float(row[0])
                    pose.pose.position.y = float(row[1])
                    pose.pose.position.z = float(row[2]) # z 위치에 곡률(curvature) 저장
                    
                    pose.pose.orientation.w = 1.0
                    
                    loaded_path.poses.append(pose)

        except Exception as e:
            self.get_logger().error(f"Failed to read or parse CSV file: {e}")
            return

        if not loaded_path.poses:
            self.get_logger().error(f"No valid waypoints were loaded from {filename}.")
            return

        self.path_pub.publish(loaded_path)
        self.get_logger().info(f'Path with {len(loaded_path.poses)} waypoints loaded from {filepath} and published to /path.')

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

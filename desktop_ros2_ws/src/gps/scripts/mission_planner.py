import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from std_msgs.msg import Int32
import pandas as pd
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np

class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        
        self.stop_trigger_distance = 1.0
        self.search_window = 200 
        
        self.odom_sub = self.create_subscription(Odometry, '/vehicle_odometry', self.odom_callback, 10)
        self.speed_policy_pub = self.create_publisher(Float64, '/pure_pursuit/speed_policy', 10)
        self.park1_pub = self.create_publisher(Bool, '/parking_command_1', 10)
        self.park2_pub = self.create_publisher(Bool, '/parking_command_2', 10)
        self.index_pub = self.create_publisher(Int32, '/waypoint_index', 10)

        self.state = 'DRIVING'
        self.stop_end_time = None
        self.last_completed_event_idx = -1
        self.current_event_idx = -1
        
        self.path_df = None
        self.path_points = None
        self.stop_indices = []
        self.park1_indices = []
        self.park2_indices = []
        self.load_path()
        
        self.current_pose = None
        self.current_closest_idx = 0
        
        self.last_logged_idx = -1
        
        # ✅ [추가] 노드 시작 시 초기 위치를 찾았는지 확인하기 위한 상태 변수
        self.is_initialized = False
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f"Mission Planner (State Machine) is ready. Stop trigger distance: {self.stop_trigger_distance}m")

    def load_path(self):
        try:
            package_path = get_package_share_directory('gps')
            # 경로 파일 이름을 yongf1.csv로 수정
            file_path = os.path.join(package_path, 'paths', 'yongf1.csv')
            self.path_df = pd.read_csv(file_path)
            self.path_points = self.path_df[['x', 'y']].values
            self.stop_indices = self.path_df[self.path_df['mission_tag'] == 'STOP'].index.tolist()
            self.park1_indices = self.path_df[self.path_df['mission_tag'] == 'PARK_1'].index.tolist()
            self.park2_indices = self.path_df[self.path_df['mission_tag'] == 'PARK_2'].index.tolist()
            self.get_logger().info(f"'{file_path}'에서 {len(self.path_df)}개의 웨이포인트를 로드했습니다.")
            self.get_logger().info(f"Found {len(self.stop_indices)} STOP points at indices: {self.stop_indices}")
            self.get_logger().info(f"Found {len(self.park1_indices)} PARK_1 points at indices: {self.park1_indices}")
            self.get_logger().info(f"Found {len(self.park2_indices)} PARK_2 points at indices: {self.park2_indices}")
        except Exception as e:
            self.get_logger().error(f"경로 파일을 로드하는 데 실패했습니다: {e}")

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def timer_callback(self):
        if self.current_pose is None or self.path_df is None:
            return

        if self.state == 'STOPPING':
            # ... (이 부분은 수정 없음)
            if self.get_clock().now() >= self.stop_end_time:
                self.get_logger().info('Stop/Park mission finished, resuming driving.')
                self.last_completed_event_idx = self.current_event_idx
                self.state = 'DRIVING'
            else:
                stop_msg = Float64()
                stop_msg.data = 0.0
                self.speed_policy_pub.publish(stop_msg)
                return

        if self.state == 'DRIVING':
            current_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])

            # ✅ [수정] 초기화 로직 추가
            if not self.is_initialized:
                # 노드 시작 후 첫 실행 시, 전체 경로에서 가장 가까운 점을 찾습니다.
                distances = np.linalg.norm(self.path_points - current_pos, axis=1)
                self.current_closest_idx = int(np.argmin(distances))
                self.is_initialized = True # 초기화 완료 상태로 변경
                self.get_logger().info(f"Initialized. Starting tracking from nearest waypoint: {self.current_closest_idx}")
            
            # 초기화가 끝난 후에는 기존의 'search_window' 로직을 사용합니다.
            half_window = self.search_window // 2
            path_search_start_idx = max(0, self.current_closest_idx - half_window)
            path_search_end_idx = min(len(self.path_points), self.current_closest_idx + half_window)
            search_points = self.path_points[path_search_start_idx:path_search_end_idx]
            
            if len(search_points) == 0:
                self.get_logger().info('End of path search window reached. Stopping vehicle.')
                final_stop_msg = Float64()
                final_stop_msg.data = 0.0
                self.speed_policy_pub.publish(final_stop_msg)
                self.destroy_timer(self.timer)
                return

            distances = np.linalg.norm(search_points - current_pos, axis=1)
            relative_idx = np.argmin(distances)
            
            self.current_closest_idx = int(path_search_start_idx + relative_idx)
            
            index_msg = Int32()
            index_msg.data = self.current_closest_idx
            self.index_pub.publish(index_msg)

            if self.current_closest_idx != self.last_logged_idx:
                utm_x = self.path_df.loc[self.current_closest_idx, 'x']
                utm_y = self.path_df.loc[self.current_closest_idx, 'y']
                self.get_logger().info(
                    f"Closest Idx: {self.current_closest_idx} | "
                    f"UTM_X: {utm_x:.3f}, UTM_Y: {utm_y:.3f}"
                )
                self.last_logged_idx = self.current_closest_idx

            # ... (미션 처리 로직은 수정 없음)
            events = []
            for idx in self.stop_indices: events.append((idx, 'STOP'))
            for idx in self.park1_indices: events.append((idx, 'PARK_1'))
            for idx in self.park2_indices: events.append((idx, 'PARK_2'))

            next_event_idx = -1
            next_event_tag = None
            
            for idx, tag in sorted(events):
                if idx > self.last_completed_event_idx:
                    next_event_idx = idx
                    next_event_tag = tag
                    break
            
            if next_event_idx != -1:
                if next_event_idx > self.current_closest_idx:
                    distance_to_event = np.linalg.norm(self.path_points[next_event_idx] - current_pos)
                    if distance_to_event < self.stop_trigger_distance:
                        self.get_logger().info(f"Entering STOPPING state for event '{next_event_tag}' at waypoint {next_event_idx}.")
                        self.state = 'STOPPING'
                        self.current_event_idx = next_event_idx
                        
                        try:
                            stop_duration_sec = float(self.path_df.loc[next_event_idx, 'mission_param'])
                        except (ValueError, TypeError):
                            stop_duration_sec = 3.0
                        self.stop_end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=stop_duration_sec)
                        
                        trigger_msg = Bool()
                        trigger_msg.data = True
                        if next_event_tag == 'PARK_1':
                            self.park1_pub.publish(trigger_msg)
                        elif next_event_tag == 'PARK_2':
                            self.park2_pub.publish(trigger_msg)
                        
                        stop_msg = Float64()
                        stop_msg.data = 0.0
                        self.speed_policy_pub.publish(stop_msg)
                        return

            mission_tag = self.path_df.loc[self.current_closest_idx, 'mission_tag']
            if mission_tag == 'DRIVING':
                speed_param = float(self.path_df.loc[self.current_closest_idx, 'mission_param'])
                speed_msg = Float64()
                speed_msg.data = speed_param
                self.speed_policy_pub.publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    mission_planner_node = MissionPlanner()
    try:
        rclpy.spin(mission_planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        mission_planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



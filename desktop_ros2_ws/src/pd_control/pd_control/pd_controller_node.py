# # import rclpy
# # from rclpy.node import Node
# # import time
# # from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
# # from std_msgs.msg import Float32, Bool, Int32
# # import math
# # import csv
# # import glob
# # import os

# # class PIDController:
# #     """PID 제어기 클래스"""
# #     def __init__(self, kp=2.0, ki=0.3, kd=0.1):
# #         self.kp = kp  # 비례 게인
# #         self.kd = kd  # 미분 게인
# #         self.ki = ki  # 적분 게인
# #         self.previous_error = 0.0
# #         self.previous_time = None
# #         self.integral_error = 0.0  # 적분항 누적 변수 추가
# #         self.Ld = 0.7315
        
# #         # 미분항 필터링을 위한 변수들
# #         self.derivative_filter_alpha = 0.1  # 저주파 통과 필터 계수 (0~1)
# #         self.filtered_derivative = 0.0
        
# #     def compute(self, setpoint, measured_value):
# #         """PID 제어 신호 계산"""
# #         current_time = time.time()
        
# #         # 오차 계산 (setpoint - measured_value)
# #         error = setpoint - measured_value
        
# #         # 시간 초기화
# #         if self.previous_time is None:
# #             self.previous_time = current_time
# #             self.previous_error = error
# #             self.integral_error = 0.0
# #             return self.kp * error
        
# #         # 시간 간격 계산
# #         dt = current_time - self.previous_time
        
# #         # 적분항 계산 (오차의 누적합)
# #         if dt > 0:
# #             self.integral_error += error * dt
# #             # 적분항 윈드업 방지 (제한) - 더 안전한 범위로 설정
# #             self.integral_error = max(-10.0, min(10.0, self.integral_error))
# #         else:
# #             dt = 0.01  # 최소 시간 간격 설정
        
# #         # 미분항 계산 (오차의 변화율) + 노이즈 필터링
# #         if dt > 0:
# #             raw_derivative = (error - self.previous_error) / dt
# #             # 저주파 통과 필터 적용 (exponential moving average)
# #             self.filtered_derivative = (self.derivative_filter_alpha * raw_derivative + 
# #                                        (1 - self.derivative_filter_alpha) * self.filtered_derivative)
# #             derivative = self.filtered_derivative
# #         else:
# #             derivative = self.filtered_derivative
        
# #         # PID 제어 신호 계산
# #         control_signal = (self.kp * error + self.ki * self.integral_error + self.kd * derivative)
        
# #         # 이전 값들 업데이트
# #         self.previous_error = error
# #         self.previous_time = current_time
        
# #         return control_signal
    
# #     def reset(self):
# #         """PID 제어기 리셋 (목표 변경 시 사용)"""
# #         self.previous_error = 0.0
# #         self.previous_time = None
# #         self.integral_error = 0.0
# #         self.filtered_derivative = 0.0

# # class PDControllerNode(Node):
# #     """
# #     PD 제어기 ROS2 노드.
# #     - AUTONOMOUS_DRIVING: /cmd_vel을 받아 속도 및 조향 제어.
# #     - PARKING_MANEUVER: /parking_command_1 신호를 받으면 저장된 CSV 경로를 재생.
# #     """
# #     def __init__(self):
# #         super().__init__('pd_controller_node')
        
# #         # --- 상태 관리 변수 ---
# #         self.state = 'AUTONOMOUS_DRIVING'  # 'AUTONOMOUS_DRIVING' 또는 'PARKING_MANEUVER'
# #         self.playback_data = []
# #         self.playback_index = 0
# #         self.playback_timer = None
# #         self.post_playback_followup_done = False  # 첫 재생 이후 후속 재생 여부

# #         # --- 신호등 제어 상태 초기값 ---
# #         self.traffic_light_state = 'NORMAL'  # 'NORMAL' | 'PREPARING_STOP' | 'STOPPED'
# #         self.driving_control_signal = 1  # 0: stop, 1: go, 2: prepare
# #         self.current_waypoint_index = 0
# #         self.stop_target_waypoint = 0
# #         self.gradual_stop_start_waypoint = None
# #         self.original_target_velocity = 0.0
        
# #         # *** 고정 정지 웨이포인트 설정 ***
# #         self.FIXED_STOP_WAYPOINT = 1000000
# #         self.APPROACH_DISTANCE = 30  # waypoint 200에 가까워지는 거리 (150부터 감지)
        
# #         # *** driving_control 지속 체크를 위한 변수 ***
# #         self.driving_control_history = []  # 최근 신호 이력
# #         self.SIGNAL_CHECK_DURATION = 1.0  # 2초간 지속적으로 0이 와야 정지
# #         self.signal_check_timer = time.time()

# #         # PD 제어기 초기화 (안정성을 위해 게인 값 조정)
# #         self.pd_controller = PIDController(kp=1.8, ki=0.8, kd=0.2)
        
# #         # 목표 속도 설정 (m/s)
# #         self.target_velocity = 0.0
        
# #         # 현재 측정된 속도
# #         self.current_velocity = 0.0
        
# #         # 목표 각속도 초기화
# #         self.angular_target = 0.0
        
# #         # 차량 파라미터
# #         self.Ld = 0.7315  # 차축 거리
        
# #         # 램프업 관련 변수
# #         self.ramp_up_duration = 1.0  # 1초 동안 램프업
# #         self.ramp_up_start_time = None
# #         self.is_ramping_up = False
        
# #         # 제어 신호 스무딩을 위한 변수
# #         self.output_filter_alpha = 0.3  # 출력 스무딩 필터 계수 (0~1)
# #         self.filtered_control_output = 0.0

# #         # ================== [추가] 긴급 장애물 정지 상태 변수 ==================
# #         self.obstacle_stop_active = False
# #         # =================================================================

# #         # --- Subscribers ---
# #         self.velocity_sub = self.create_subscription(
# #             TwistWithCovarianceStamped,
# #             '/ublox_gps_node/fix_velocity',
# #             self.velocity_callback,
# #             10
# #         )
# #         self.cmd_vel_sub = self.create_subscription(
# #             Twist,
# #             'cmd_vel',
# #             self.cmd_vel_callback,
# #             10
# #         )
# #         self.parking_sub = self.create_subscription(
# #             Bool,
# #             '/parking_command_1',
# #             self.parking_command_callback,
# #             10
# #         )
        
# #         self.driving_control_sub = self.create_subscription(
# #             Int32,
# #             '/main_vision/driving_control',  # 토픽 이름은 이미 수정된 상태
# #             self.driving_control_callback,
# #             10
# #         )

# #         self.waypoint_index_sub = self.create_subscription(
# #             Int32,
# #             'waypoint_index',
# #             self.waypoint_index_callback,
# #             10
# #         )
        
# #         # --- Publishers ---
# #         self.control_pub = self.create_publisher(Float32, '/control_signal', 10)
# #         self.angular_target_pub = self.create_publisher(Float32, 'angular_target', 10)
        
# #         # 자율주행 제어 루프 타이머 (20Hz)
# #         self.control_timer = self.create_timer(0.05, self.control_loop)
        
# #         self.get_logger().info('PD Controller Node가 시작되었습니다. 현재 상태: AUTONOMOUS_DRIVING')
# #         self.get_logger().info(f'고정 정지 지점: waypoint {self.FIXED_STOP_WAYPOINT}')
# #         self.get_logger().info(f'정지 감지 범위: waypoint {self.FIXED_STOP_WAYPOINT - self.APPROACH_DISTANCE} ~ {self.FIXED_STOP_WAYPOINT + 10}')
# #         self.get_logger().info(f'PD 게인 - Kp: {self.pd_controller.kp}, Kd: {self.pd_controller.kd}')
# #         self.get_logger().info('신호등 제어: waypoint 200 근처에서 지속적인 driving_control=0 신호 시 정지')
        
# #     # --- 콜백 함수들 ---
# #     def velocity_callback(self, msg: TwistWithCovarianceStamped):
# #         linear_velocity = msg.twist.twist.linear
# #         self.current_velocity = (linear_velocity.x**2 + linear_velocity.y**2)**0.5
        
# #     def cmd_vel_callback(self, msg: Twist):
# #         if self.state != 'AUTONOMOUS_DRIVING':
# #             return
        
# #         if abs(self.target_velocity - msg.linear.x) > 0.1:
# #             self.pd_controller.reset()
# #             self.get_logger().info(f"목표 속도 변경: {self.target_velocity:.2f} -> {msg.linear.x:.2f} m/s, PID 리셋")
        
# #         self.target_velocity = msg.linear.x
# #         self.angular_target = msg.angular.z
        
# #         if self.traffic_light_state == 'NORMAL':
# #             self.original_target_velocity = msg.linear.x
    
# #     def parking_command_callback(self, msg: Bool):
# #         if msg.data and self.state == 'AUTONOMOUS_DRIVING':
# #             self.get_logger().info("주차 명령을 수신했습니다. 경로 재생을 시작합니다...")
# #             self.start_parking_maneuver()
    
# #     def driving_control_callback(self, msg: Int32):
# #         current_time = time.time()
# #         self.driving_control_signal = msg.data

# #         # ================== [수정] 긴급 장애물 정지 조건 감지 ==================
# #         # 1. 조건 정의: waypoint가 800-900 사이이고, 신호가 3인가?
# #         is_in_obstacle_zone = 800000 <= self.current_waypoint_index <= 9000000
        
# #         # 2. 긴급 정지 상태 활성화
# #         if msg.data == 3 and is_in_obstacle_zone:
# #             if not self.obstacle_stop_active:
# #                 self.get_logger().warn(
# #                     f"장애물 정지 신호(3) 수신! (waypoint: {self.current_waypoint_index}). 긴급 정지를 활성화합니다."
# #                 )
# #                 self.obstacle_stop_active = True
                
# #         # 3. 긴급 정지 상태 비활성화 (신호가 3이 아니거나, 구간을 벗어났을 때)
# #         elif self.obstacle_stop_active and (msg.data != 3 or not is_in_obstacle_zone):
# #             self.get_logger().info("장애물 신호가 해제되었거나 정지 구간을 이탈했습니다. 정상 주행을 재개합니다.")
# #             self.obstacle_stop_active = False
# #             self.pd_controller.reset()  # 부드러운 재출발을 위해 PID 리셋
# #         # ===================================================================

# #         # --- 기존 신호등 정지 로직 (수정 없음) ---
# #         self.driving_control_history.append((current_time, msg.data))
        
# #         self.driving_control_history = [
# #             (t, signal) for t, signal in self.driving_control_history 
# #             if current_time - t <= self.SIGNAL_CHECK_DURATION
# #         ]
        
# #         is_near_stop_point = (self.FIXED_STOP_WAYPOINT - self.APPROACH_DISTANCE) <= self.current_waypoint_index <= (self.FIXED_STOP_WAYPOINT)
        
# #         if is_near_stop_point:
# #             if self.is_continuous_stop_signal():
# #                 if self.traffic_light_state == 'NORMAL':
# #                     self.get_logger().info(f"waypoint 200 근처에서 지속적인 정지 신호 감지 (현재: {self.current_waypoint_index})")
# #                     self.handle_stop_signal()
# #             elif msg.data == 1 and self.traffic_light_state in ['PREPARING_STOP', 'STOPPED']:
# #                 self.get_logger().info("진행 신호 수신 - 정상 주행 재개")
# #                 self.handle_go_signal()
# #         else:
# #             if msg.data == 1 and self.traffic_light_state in ['PREPARING_STOP', 'STOPPED']:
# #                 self.get_logger().info("진행 신호 수신 - 정상 주행 재개")
# #                 self.handle_go_signal()

# #     def is_continuous_stop_signal(self):
# #         """지속적으로 정지 신호(0)가 들어오는지 확인"""
# #         if len(self.driving_control_history) < 5:  # 최소 5개의 신호가 필요
# #             return False
        
# #         # 최근 신호들이 모두 0인지 확인
# #         recent_signals = [signal for _, signal in self.driving_control_history[-5:]]  # 최근 5개
# #         return all(signal == 0 for signal in recent_signals)
    
# #     def waypoint_index_callback(self, msg: Int32):
# #         self.current_waypoint_index = msg.data
    
# #     # CSV 로더 유틸
# #     def _load_csv_into_playback(self, file_path: str) -> bool:
# #         try:
# #             with open(file_path, 'r') as csvfile:
# #                 reader = csv.reader(csvfile)
# #                 # 헤더 유무에 관계없이 첫 행이 숫자가 아니면 헤더로 간주하고 스킵
# #                 peek = next(reader, None)
# #                 if peek is None:
# #                     return False
# #                 try:
# #                     float(peek[1]); float(peek[2])
# #                     rows = [peek] + [row for row in reader]
# #                 except Exception:
# #                     rows = [row for row in reader]
# #                 if not rows:
# #                     return False
# #                 self.playback_data = rows
# #                 self.playback_index = 0
# #                 return True
# #         except Exception as e:
# #             self.get_logger().error(f"CSV 파일을 읽는 데 실패했습니다: {e}")
# #             return False
    
# #     # --- 신호등 제어 처리 함수들 ---
# #     def handle_stop_signal(self):
# #         """정지 신호 처리. waypoint 200에서 정지."""
# #         if self.traffic_light_state == 'NORMAL':
# #             self.gradual_stop_start_waypoint = self.current_waypoint_index
# #             self.stop_target_waypoint = self.FIXED_STOP_WAYPOINT  # 항상 200에서 정지
# #             self.traffic_light_state = 'PREPARING_STOP'
# #             self.original_target_velocity = self.target_velocity
            
# #             distance_to_stop = self.stop_target_waypoint - self.current_waypoint_index
# #             self.get_logger().info(f"waypoint 200에서 정지 예정. 현재 위치: {self.current_waypoint_index}, 남은 거리: {distance_to_stop}")
            
# #             if distance_to_stop <= 0:
# #                 # 이미 200을 지났으면 즉시 정지
# #                 self.traffic_light_state = 'STOPPED'
# #                 self.get_logger().warn(f"이미 정지 지점({self.FIXED_STOP_WAYPOINT})을 지났습니다. 즉시 정지합니다.")

# #     def handle_go_signal(self):
# #         """진행 신호 처리"""
# #         if self.traffic_light_state in ['PREPARING_STOP', 'STOPPED']:
# #             self.traffic_light_state = 'NORMAL'
# #             self.gradual_stop_start_waypoint = None
# #             if hasattr(self, 'original_target_velocity'):
# #                 self.target_velocity = self.original_target_velocity
# #             self.pd_controller.reset()
# #             self.get_logger().info("정상 주행 재개")
    
# #     def handle_prepare_signal(self):
# #         """준비 신호 처리 (노란불)"""
# #         self.handle_stop_signal()
    
# #     def calculate_traffic_light_velocity(self):
# #         """신호등 제어에 따른 목표 속도 계산"""
# #         if self.traffic_light_state == 'NORMAL':
# #             return self.target_velocity
        
# #         elif self.traffic_light_state == 'PREPARING_STOP':
# #             if self.current_waypoint_index >= self.stop_target_waypoint:
# #                 self.traffic_light_state = 'STOPPED'
# #                 return 0.0
            
# #             if self.gradual_stop_start_waypoint is not None:
# #                 remaining_waypoints = self.stop_target_waypoint - self.current_waypoint_index
# #                 total_waypoints = self.stop_target_waypoint - self.gradual_stop_start_waypoint
                
# #                 if total_waypoints > 0:
# #                     speed_ratio = remaining_waypoints / total_waypoints
# #                     target_speed = self.original_target_velocity * max(0.0, speed_ratio)
# #                     return target_speed
# #                 else:
# #                     # 감속할 거리가 없으면 즉시 정지
# #                     return 0.0
# #             else:
# #                 return self.target_velocity
        
# #         elif self.traffic_light_state == 'STOPPED':
# #             return 0.0
        
# #         return self.target_velocity

# #     # --- 주차 경로 재생 로직 (변경 없음) ---
# #     def start_parking_maneuver(self):
# #         self.state = 'PARKING_MANEUVER'
# #         self.control_timer.cancel()
# #         self.pd_controller.reset()
# #         self.get_logger().info("주차 모드 진입: PID 제어기를 리셋했습니다.")
# #         self.post_playback_followup_done = False
# #         list_of_files = glob.glob('recording_*.csv')
# #         if not list_of_files:
# #             self.get_logger().error("주차에 사용할 녹화 파일을 찾을 수 없습니다! 자율주행으로 복귀합니다.")
# #             self.finish_parking_maneuver()
# #             return
# #         latest_file = max(list_of_files, key=os.path.getctime)
# #         self.get_logger().info(f"주차 경로 파일을 로드합니다: '{latest_file}'")
# #         if not self._load_csv_into_playback(latest_file):
# #             self.get_logger().error("CSV 파일이 비어있습니다! 주차를 취소합니다.")
# #             self.finish_parking_maneuver()
# #             return
# #         self.playback_index = 0
# #         self.playback_timer = self.create_timer(0.05, self.playback_loop)

# #     def playback_loop(self):
# #         if self.playback_index >= len(self.playback_data):
# #             # 첫 번째 재생이 끝났고 후속 재생이 설정 가능하면 연결 재생
# #             if not self.post_playback_followup_done:
# #                 next_file = None
# #                 if self.driving_control_signal == 5:
# #                     next_file = '1.csv'
# #                 elif self.driving_control_signal == 6:
# #                     next_file = '2.csv'
# #                 if next_file and os.path.exists(next_file):
# #                     self.get_logger().info(f"후속 재생 조건 감지(driving_control={self.driving_control_signal}). '{next_file}' 재생 시작")
# #                     if self._load_csv_into_playback(next_file):
# #                         self.post_playback_followup_done = True
# #                         return  # 다음 루프부터 새 파일 재생
# #                     else:
# #                         self.get_logger().error(f"'{next_file}' 로드 실패. 주차를 종료합니다.")
# #                 else:
# #                     if next_file:
# #                         self.get_logger().warn(f"'{next_file}' 파일을 찾을 수 없습니다. 주차를 종료합니다.")
# #             self.get_logger().info("주차 경로 재생을 완료했습니다.")
# #             self.finish_parking_maneuver()
# #             return
# #         row = self.playback_data[self.playback_index]
# #         try:
# #             control_val = float(row[1])
# #             angular_val = float(row[2])
# #             control_msg = Float32()
# #             control_msg.data = control_val
# #             self.control_pub.publish(control_msg)
# #             angular_target_msg = Float32()
# #             angular_target_msg.data = angular_val
# #             self.angular_target_pub.publish(angular_target_msg)
# #             self.playback_index += 1
# #         except (ValueError, IndexError) as e:
# #             self.get_logger().error(f"주차 데이터 처리 중 오류 발생 (행 {self.playback_index}): {row}. 오류: {e}")
# #             self.finish_parking_maneuver()

# #     def finish_parking_maneuver(self):
# #         if self.playback_timer:
# #             self.playback_timer.destroy()
# #             self.playback_timer = None
# #         self.pd_controller.reset()
# #         self.is_ramping_up = False
# #         self.ramp_up_start_time = None
# #         self.target_velocity = 0.0
# #         self.angular_target = 0.0
# #         self.filtered_control_output = 0.0
# #         self.get_logger().info("자율주행 모드로 복귀합니다. PID 제어기와 램프업 상태를 리셋했습니다.")
# #         self.state = 'AUTONOMOUS_DRIVING'
# #         self.control_timer.reset()

# #     # --- 자율주행 로직 (변경 없음) ---
# #     def calculate_angle(self, target_velocity, angular_target):
# #         if abs(angular_target) < 1e-6:
# #             return 0.0
# #         if abs(target_velocity) < 0.1:
# #             return angular_target * 2.0
# #         return math.atan2(self.Ld * angular_target, target_velocity) * 5

# #     def calculate_ramp_up_velocity(self):
# #         if abs(self.current_velocity) < 0.1 and self.target_velocity > 0.1:
# #             if not self.is_ramping_up:
# #                 self.is_ramping_up = True
# #                 self.ramp_up_start_time = time.time()
# #                 self.get_logger().info('속도 램프업 시작!')
# #             elapsed_time = time.time() - self.ramp_up_start_time
# #             if elapsed_time < self.ramp_up_duration:
# #                 progress = elapsed_time / self.ramp_up_duration
# #                 return self.target_velocity * progress
# #             else:
# #                 self.is_ramping_up = False
# #                 return self.target_velocity
# #         else:
# #             self.is_ramping_up = False
# #             return self.target_velocity

# #     def control_loop(self):
# #         # ================== [수정] 긴급 장애물 정지 로직 (최우선 순위) ==================
# #         if self.obstacle_stop_active:
# #             # 긴급 정지 상태가 활성화되면 목표 속도를 즉시 0으로 설정
# #             effective_target_velocity = 0.0
# #         else:
# #             # 기존의 신호등 및 램프업 로직 수행
# #             traffic_controlled_velocity = self.calculate_traffic_light_velocity()
# #             original_target = self.target_velocity
# #             self.target_velocity = traffic_controlled_velocity
# #             effective_target_velocity = self.calculate_ramp_up_velocity()
# #             self.target_velocity = original_target
# #         # =========================================================================

# #         # --- PD 제어 및 퍼블리시 로직 (수정 없음) ---
# #         control_signal = self.pd_controller.compute(
# #             setpoint=effective_target_velocity,
# #             measured_value=self.current_velocity
# #         )
# #         raw_control_output = max(0.0, min(100.0, control_signal * 20.0))
# #         self.filtered_control_output = (self.output_filter_alpha * raw_control_output + 
# #                                         (1 - self.output_filter_alpha) * self.filtered_control_output)
# #         control_output = self.filtered_control_output
# #         control_msg = Float32()
# #         angular_target_msg = Float32()
        
# #         # effective_target_velocity를 기준으로 각도 계산
# #         angular_target_msg.data = self.calculate_angle(effective_target_velocity, self.angular_target)
# #         self.angular_target_pub.publish(angular_target_msg)
        
# #         control_msg.data = control_output
# #         self.control_pub.publish(control_msg)
        
# #         if self.traffic_light_state != 'NORMAL':
# #             self.get_logger().debug(f"신호등 제어: 상태={self.traffic_light_state}, "
# #                                   f"waypoint={self.current_waypoint_index}, "
# #                                   f"목표속도={effective_target_velocity:.2f}, "
# #                                   f"제어출력={control_output:.2f}")

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = PDControllerNode()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         node.destroy_node()
# #         rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()

# import rclpy
# from rclpy.node import Node
# import time
# from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
# from std_msgs.msg import Float32, Bool, Int32
# import math
# import csv
# import glob
# import os

# class PIDController:
#     """PID 제어기 클래스"""
#     def __init__(self, kp=2.0, ki=0.3, kd=0.1):
#         self.kp = kp  # 비례 게인
#         self.kd = kd  # 미분 게인
#         self.ki = ki  # 적분 게인
#         self.previous_error = 0.0
#         self.previous_time = None
#         self.integral_error = 0.0  # 적분항 누적 변수 추가
#         self.Ld = 0.7315
        
#         # 미분항 필터링을 위한 변수들
#         self.derivative_filter_alpha = 0.1  # 저주파 통과 필터 계수 (0~1)
#         self.filtered_derivative = 0.0
        
#     def compute(self, setpoint, measured_value):
#         """PID 제어 신호 계산"""
#         current_time = time.time()
        
#         # 오차 계산 (setpoint - measured_value)
#         error = setpoint - measured_value
        
#         # 시간 초기화
#         if self.previous_time is None:
#             self.previous_time = current_time
#             self.previous_error = error
#             self.integral_error = 0.0
#             return self.kp * error
        
#         # 시간 간격 계산
#         dt = current_time - self.previous_time
        
#         # 적분항 계산 (오차의 누적합)
#         if dt > 0:
#             self.integral_error += error * dt
#             # 적분항 윈드업 방지 (제한) - 더 안전한 범위로 설정
#             self.integral_error = max(-10.0, min(10.0, self.integral_error))
#         else:
#             dt = 0.01  # 최소 시간 간격 설정
        
#         # 미분항 계산 (오차의 변화율) + 노이즈 필터링
#         if dt > 0:
#             raw_derivative = (error - self.previous_error) / dt
#             # 저주파 통과 필터 적용 (exponential moving average)
#             self.filtered_derivative = (self.derivative_filter_alpha * raw_derivative + 
#                                        (1 - self.derivative_filter_alpha) * self.filtered_derivative)
#             derivative = self.filtered_derivative
#         else:
#             derivative = self.filtered_derivative
        
#         # PID 제어 신호 계산
#         control_signal = (self.kp * error + self.ki * self.integral_error + self.kd * derivative)
        
#         # 이전 값들 업데이트
#         self.previous_error = error
#         self.previous_time = current_time
        
#         return control_signal
    
#     def reset(self):
#         """PID 제어기 리셋 (목표 변경 시 사용)"""
#         self.previous_error = 0.0
#         self.previous_time = None
#         self.integral_error = 0.0
#         self.filtered_derivative = 0.0

# class PDControllerNode(Node):
#     """
#     PD 제어기 ROS2 노드.
#     - AUTONOMOUS_DRIVING: /cmd_vel을 받아 속도 및 조향 제어.
#     - PARKING_MANEUVER: /parking_command_1 신호를 받으면 저장된 CSV 경로를 재생.
#     """
#     def __init__(self):
#         super().__init__('pd_controller_node')
        
#         # --- 상태 관리 변수 ---
#         self.state = 'AUTONOMOUS_DRIVING'  # 'AUTONOMOUS_DRIVING' 또는 'PARKING_MANEUVER'
#         self.playback_data = []
#         self.playback_index = 0
#         self.playback_timer = None
#         self.post_playback_followup_done = False  # 첫 재생 이후 후속 재생 여부

#         # ================== [추가] waypoint 기반 자동 주차 변수 ==================
#         self.parking_waypoint_1276 = 1276  # 첫 번째 주차 구간
#         self.parking_waypoint_1800 = 1800  # 두 번째 주차 구간
#         self.parking_detection_start_1800 = 1750  # waypoint 1800의 감지 시작점
#         self.parking_1276_triggered = False  # waypoint 1276 주차 실행 여부
#         self.parking_1800_triggered = False  # waypoint 1800 주차 실행 여부
#         self.parking_1800_detection_active = False  # waypoint 1800 감지 활성 상태
#         self.parking_1800_detected_signal = None  # waypoint 1800에서 감지된 신호 저장
#         self.current_parking_sequence = []  # 현재 실행 중인 주차 시퀀스
#         # =======================================================================

#         # --- 신호등 제어 상태 초기값 ---
#         self.traffic_light_state = 'NORMAL'  # 'NORMAL' | 'PREPARING_STOP' | 'STOPPED'
#         self.driving_control_signal = 1  # 0: stop, 1: go, 2: prepare
#         self.current_waypoint_index = 0
#         self.stop_target_waypoint = 0
#         self.gradual_stop_start_waypoint = None
#         self.original_target_velocity = 0.0
        
#         # *** 다중 신호등 정지 웨이포인트 설정 ***
#         self.TRAFFIC_STOP_WAYPOINTS = [646, 1195]  # 신호등 정지점들
#         self.APPROACH_DISTANCE = 30  # 정지점에 가까워지는 거리 (30 waypoint 전부터 감지)
#         self.current_stop_target = None  # 현재 활성화된 정지점
        
#         # *** driving_control 지속 체크를 위한 변수 ***
#         self.driving_control_history = []  # 최근 신호 이력
#         self.SIGNAL_CHECK_DURATION = 1.0  # 2초간 지속적으로 0이 와야 정지
#         self.signal_check_timer = time.time()

#         # PD 제어기 초기화 (안정성을 위해 게인 값 조정)
#         self.pd_controller = PIDController(kp=1.8, ki=0.8, kd=0.2)
        
#         # 목표 속도 설정 (m/s)
#         self.target_velocity = 0.0
        
#         # 현재 측정된 속도
#         self.current_velocity = 0.0
        
#         # 목표 각속도 초기화
#         self.angular_target = 0.0
        
#         # 차량 파라미터
#         self.Ld = 0.7315  # 차축 거리
        
#         # 램프업 관련 변수
#         self.ramp_up_duration = 1.0  # 1초 동안 램프업
#         self.ramp_up_start_time = None
#         self.is_ramping_up = False
        
#         # 제어 신호 스무딩을 위한 변수
#         self.output_filter_alpha = 0.3  # 출력 스무딩 필터 계수 (0~1)
#         self.filtered_control_output = 0.0

#         # ================== [추가] 긴급 장애물 정지 상태 변수 ==================
#         self.obstacle_stop_active = False
#         self.emergency_stop_start_time = None
#         self.emergency_stop_initial_velocity = 0.0
#         self.emergency_stop_duration = 1.0  # 1초 동안 점진적 감속
#         # =================================================================

#         # ================== [추가] 경사면 하강 제어 변수 ==================
#         self.is_on_slope = False
#         self.slope_target_velocity = 1.5  # 경사면 하강 시 목표 속도 (m/s)
#         self.slope_start_waypoint = 65000000
#         self.slope_end_waypoint = 700000000
#         self.slope_entry_velocity = 0.0  # 경사면 진입 시 원래 속도 저장
#         # ================================================================

#         # ================== [추가] 특수 구간 제어 변수 (waypoint 185-300) ==================
#         self.is_in_special_zone = False
#         self.special_zone_trigger_waypoint = 185
#         self.special_zone_end_waypoint = 300
#         self.special_zone_approach_distance = 30  # 185 도달 30 waypoint 전부터 감속 시작 (waypoint 155부터)
#         self.special_zone_target_velocity = 1.5  # 특수 구간 목표 속도 (m/s)
#         self.special_zone_control_signal = 1.0  # 3초간 유지할 control_signal 값
#         self.special_zone_control_duration = 3.0  # 3초간 control_signal 유지
#         self.special_zone_control_start_time = None
#         self.special_zone_control_active = False
#         self.special_zone_entry_velocity = 0.0  # 특수 구간 진입 시 원래 속도 저장
#         self.is_approaching_special_zone = False  # 특수 구간 접근 중 여부 (waypoint 155-184)
#         # =================================================================================

#         # ================== [추가] 속도 제한 구간 제어 변수 (waypoint 3000-3050) ==================
#         self.is_in_speed_limit_zone = False
#         self.speed_limit_start_waypoint = 3000
#         self.speed_limit_end_waypoint = 3050
#         self.speed_limit_target_velocity = 1.5  # 속도 제한 구간 목표 속도 (m/s)
#         self.speed_limit_entry_velocity = 0.0  # 속도 제한 구간 진입 시 원래 속도 저장
#         # =====================================================================================

#         # --- Subscribers ---
#         self.velocity_sub = self.create_subscription(
#             TwistWithCovarianceStamped,
#             '/ublox_gps_node/fix_velocity',
#             self.velocity_callback,
#             10
#         )
#         self.cmd_vel_sub = self.create_subscription(
#             Twist,
#             'cmd_vel',
#             self.cmd_vel_callback,
#             10
#         )
#         self.parking_sub = self.create_subscription(
#             Bool,
#             '/parking_command_1',
#             self.parking_command_callback,
#             10
#         )
        
#         self.driving_control_sub = self.create_subscription(
#             Int32,
#             'main_vision/driving_control',  # 토픽 이름은 이미 수정된 상태
#             self.driving_control_callback,
#             10
#         )

#         self.waypoint_index_sub = self.create_subscription(
#             Int32,
#             'waypoint_index',
#             self.waypoint_index_callback,
#             10
#         )
        
#         # --- Publishers ---
#         self.control_pub = self.create_publisher(Float32, '/control_signal', 10)
#         self.angular_target_pub = self.create_publisher(Float32, 'angular_target', 10)
        
#         # 자율주행 제어 루프 타이머 (20Hz)
#         self.control_timer = self.create_timer(0.05, self.control_loop)
        
#         self.get_logger().info('PD Controller Node가 시작되었습니다. 현재 상태: AUTONOMOUS_DRIVING')
#         self.get_logger().info(f'다중 신호등 정지점: {self.TRAFFIC_STOP_WAYPOINTS}')
#         self.get_logger().info(f'신호등 정지 감지 범위: 각 정지점 -{self.APPROACH_DISTANCE} waypoint 전부터 감지')
#         self.get_logger().info(f'PD 게인 - Kp: {self.pd_controller.kp}, Kd: {self.pd_controller.kd}')
#         self.get_logger().info('신호등 제어: driving_control=0 (지속신호) 또는 driving_control=2 (즉시정지) 시 정지, driving_control=1 시 진행')
#         self.get_logger().info(f'경사면 하강 제어: waypoint {self.slope_start_waypoint}-{self.slope_end_waypoint} 구간에서 속도 {self.slope_target_velocity} m/s로 제한')
#         self.get_logger().info(f'특수 구간 제어: waypoint {self.special_zone_trigger_waypoint}에서 control_signal {self.special_zone_control_signal} 3초 유지, {self.special_zone_trigger_waypoint}-{self.special_zone_end_waypoint} 구간에서 속도 {self.special_zone_target_velocity} m/s로 제한')
#         self.get_logger().info(f'속도 제한 구간: waypoint {self.speed_limit_start_waypoint}-{self.speed_limit_end_waypoint} 구간에서 속도 {self.speed_limit_target_velocity} m/s로 제한')
#         self.get_logger().info('긴급 정지: waypoint 800-900 구간에서 driving_control=3 신호 시 1초간 점진적 정지')
#         self.get_logger().info(f'waypoint 기반 자동 주차: waypoint {self.parking_waypoint_1276}에서 1.csv→2.csv/3.csv, waypoint {self.parking_waypoint_1800}({self.parking_detection_start_1800}부터 감지)에서 4.csv/5.csv')
        
#     # --- 콜백 함수들 ---
#     def velocity_callback(self, msg: TwistWithCovarianceStamped):
#         linear_velocity = msg.twist.twist.linear
#         self.current_velocity = (linear_velocity.x**2 + linear_velocity.y**2)**0.5
        
#     def cmd_vel_callback(self, msg: Twist):
#         if self.state != 'AUTONOMOUS_DRIVING':
#             return
        
#         if abs(self.target_velocity - msg.linear.x) > 0.1:
#             self.pd_controller.reset()
#             self.get_logger().info(f"목표 속도 변경: {self.target_velocity:.2f} -> {msg.linear.x:.2f} m/s, PID 리셋")
        
#         self.target_velocity = msg.linear.x
#         self.angular_target = msg.angular.z
        
#         # ================== [수정] 모든 제한 구간 이탈 시 속도 복원 ==================
#         # 각 제한 구간이 아닐 때만 해당 entry_velocity 업데이트
#         if not self.is_on_slope:
#             self.slope_entry_velocity = msg.linear.x
#         if not self.is_in_special_zone and not self.is_approaching_special_zone:
#             self.special_zone_entry_velocity = msg.linear.x
#         if not self.is_in_speed_limit_zone:
#             self.speed_limit_entry_velocity = msg.linear.x
#         # =============================================================================
        
#         if self.traffic_light_state == 'NORMAL':
#             self.original_target_velocity = msg.linear.x
    
#     def parking_command_callback(self, msg: Bool):
#         if msg.data and self.state == 'AUTONOMOUS_DRIVING':
#             self.get_logger().info("주차 명령을 수신했습니다. 경로 재생을 시작합니다...")
#             self.start_parking_maneuver()

    
#     def driving_control_callback(self, msg: Int32):
#         current_time = time.time()
#         self.driving_control_signal = msg.data

#         # ================== [수정] 긴급 장애물 정지 조건 감지 ==================
#         # 1. 조건 정의: waypoint가 800-900 사이이고, 신호가 3인가?
#         is_in_obstacle_zone = 800 <= self.current_waypoint_index <= 900
        
#         # 2. 긴급 정지 상태 활성화
#         if msg.data == 3 and is_in_obstacle_zone:
#             if not self.obstacle_stop_active:
#                 self.get_logger().warn(
#                     f"장애물 정지 신호(3) 수신! (waypoint: {self.current_waypoint_index}). 1초간 점진적 긴급 정지를 시작합니다."
#                 )
#                 self.obstacle_stop_active = True
#                 self.emergency_stop_start_time = time.time()
#                 self.emergency_stop_initial_velocity = self.current_velocity
                
#         # 3. 긴급 정지 상태 비활성화 (신호가 3이 아니거나, 구간을 벗어났을 때)
#         elif self.obstacle_stop_active and (msg.data != 3 or not is_in_obstacle_zone):
#             self.get_logger().info("장애물 신호가 해제되었거나 정지 구간을 이탈했습니다. 정상 주행을 재개합니다.")
#             self.obstacle_stop_active = False
#             self.emergency_stop_start_time = None
#             self.pd_controller.reset()  # 부드러운 재출발을 위해 PID 리셋
#         # ===================================================================

#         # ================== [추가] waypoint 기반 주차 신호 감지 ==================
#         # waypoint 1750-1800 구간에서 driving_control 5/6 감지
#         if self.parking_1800_detection_active and msg.data in [5, 6]:
#             if self.parking_1800_detected_signal is None:  # 첫 번째 신호만 저장
#                 self.parking_1800_detected_signal = msg.data
#                 self.get_logger().info(f"waypoint 1800 주차 신호 감지: driving_control={msg.data}")
#         # =====================================================================

#         # --- 기존 신호등 정지 로직 (수정 없음) ---
#         self.driving_control_history.append((current_time, msg.data))
        
#         self.driving_control_history = [
#             (t, signal) for t, signal in self.driving_control_history 
#             if current_time - t <= self.SIGNAL_CHECK_DURATION
#         ]
        
#         # ================== [수정] 다중 정지점 및 driving_control=2 처리 ==================
#         # 현재 waypoint 근처의 정지점 찾기
#         nearby_stop_point = None
#         for stop_point in self.TRAFFIC_STOP_WAYPOINTS:
#             if (stop_point - self.APPROACH_DISTANCE) <= self.current_waypoint_index <= stop_point:
#                 nearby_stop_point = stop_point
#                 break
        
#         if nearby_stop_point is not None:
#             # 정지 신호 처리 (driving_control=0 또는 2)
#             if (self.is_continuous_stop_signal() or msg.data == 2):
#                 if self.traffic_light_state == 'NORMAL':
#                     signal_type = "지속적인 정지" if self.is_continuous_stop_signal() else "즉시 정지(2)"
#                     self.get_logger().info(f"waypoint {nearby_stop_point} 근처에서 {signal_type} 신호 감지 (현재: {self.current_waypoint_index})")
#                     self.current_stop_target = nearby_stop_point
#                     self.handle_stop_signal()
#             # 진행 신호 처리 (driving_control=1)
#             elif msg.data == 1 and self.traffic_light_state in ['PREPARING_STOP', 'STOPPED']:
#                 self.get_logger().info("진행 신호 수신 - 정상 주행 재개")
#                 self.handle_go_signal()
#         else:
#             # 정지점 근처가 아닐 때도 진행 신호 처리
#             if msg.data == 1 and self.traffic_light_state in ['PREPARING_STOP', 'STOPPED']:
#                 self.get_logger().info("진행 신호 수신 - 정상 주행 재개")
#                 self.handle_go_signal()
#         # ==============================================================================

#     def is_continuous_stop_signal(self):
#         """지속적으로 정지 신호(0)가 들어오는지 확인"""
#         if len(self.driving_control_history) < 5:  # 최소 5개의 신호가 필요
#             return False
        
#         # 최근 신호들이 모두 0인지 확인
#         recent_signals = [signal for _, signal in self.driving_control_history[-5:]]  # 최근 5개
#         return all(signal == 0 for signal in recent_signals)
    
#     def waypoint_index_callback(self, msg: Int32):
#         previous_waypoint = self.current_waypoint_index
#         self.current_waypoint_index = msg.data
        
#         # ================== [추가] 경사면 진입/이탈 감지 ==================
#         is_in_slope_zone = self.slope_start_waypoint <= self.current_waypoint_index <= self.slope_end_waypoint
#         was_in_slope_zone = self.slope_start_waypoint <= previous_waypoint <= self.slope_end_waypoint
        
#         # 경사면 진입 감지
#         if is_in_slope_zone and not was_in_slope_zone:
#             if not self.is_on_slope:
#                 self.slope_entry_velocity = self.target_velocity  # 현재 목표 속도 저장
#                 self.is_on_slope = True
#                 self.get_logger().info(f"경사면 진입 감지! waypoint {self.current_waypoint_index}. "
#                                      f"속도를 {self.slope_entry_velocity:.2f} -> {self.slope_target_velocity:.2f} m/s로 조정합니다.")
        
#         # 경사면 이탈 감지
#         elif not is_in_slope_zone and was_in_slope_zone:
#             if self.is_on_slope:
#                 self.is_on_slope = False
#                 # 경사면 이탈 시 목표 속도를 저장된 진입 속도로 복원
#                 self.target_velocity = self.slope_entry_velocity
#                 self.get_logger().info(f"경사면 이탈 감지! waypoint {self.current_waypoint_index}. "
#                                      f"속도를 {self.slope_target_velocity:.2f} -> {self.slope_entry_velocity:.2f} m/s로 복원합니다.")
#         # ==============================================================

#         # ================== [수정] 특수 구간 접근/진입/이탈 감지 ==================
#         approach_start = self.special_zone_trigger_waypoint - self.special_zone_approach_distance  # waypoint 155
        
#         # waypoint 155에서 접근 구간 시작 (점진적 감속 시작)
#         if self.current_waypoint_index == approach_start and previous_waypoint != approach_start:
#             if not self.is_approaching_special_zone and not self.is_in_special_zone:
#                 self.special_zone_entry_velocity = self.target_velocity  # 현재 목표 속도 저장 (예: 2.6)
#                 self.is_approaching_special_zone = True
#                 self.get_logger().info(f"특수 구간 접근 시작! waypoint {self.current_waypoint_index}. "
#                                      f"점진적 감속 시작: {self.special_zone_entry_velocity:.2f} -> {self.special_zone_target_velocity:.2f} m/s")

#         # waypoint 185에서 특수 구간 진입 (control_signal 제어 시작)
#         elif self.current_waypoint_index == self.special_zone_trigger_waypoint and previous_waypoint != self.special_zone_trigger_waypoint:
#             if self.is_approaching_special_zone and not self.is_in_special_zone:
#                 self.is_in_special_zone = True
#                 self.is_approaching_special_zone = False
#                 self.special_zone_control_active = True
#                 self.special_zone_control_start_time = time.time()
#                 self.get_logger().info(f"특수 구간 진입! waypoint {self.current_waypoint_index}. "
#                                      f"control_signal을 {self.special_zone_control_signal}로 3초간 유지합니다.")

#         # waypoint 300에서 특수 구간 종료
#         elif self.current_waypoint_index >= self.special_zone_end_waypoint and self.is_in_special_zone:
#             self.is_in_special_zone = False
#             self.special_zone_control_active = False
#             self.special_zone_control_start_time = None
#             # 특수 구간 이탈 시 목표 속도를 저장된 진입 속도로 복원
#             self.target_velocity = self.special_zone_entry_velocity
#             self.get_logger().info(f"특수 구간 이탈! waypoint {self.current_waypoint_index}. "
#                                  f"속도를 {self.special_zone_target_velocity:.2f} -> {self.special_zone_entry_velocity:.2f} m/s로 복원합니다.")
#         # ================================================================

#         # ================== [추가] waypoint 기반 자동 주차 감지 ==================
#         # waypoint 1276에서 자동으로 1.csv 실행
#         if self.current_waypoint_index == self.parking_waypoint_1276 and not self.parking_1276_triggered:
#             if self.state == 'AUTONOMOUS_DRIVING':
#                 self.parking_1276_triggered = True
#                 self.get_logger().info(f"waypoint {self.parking_waypoint_1276} 도달! 자동 주차 시작 (1.csv)")
#                 self.current_parking_sequence = ['1.csv']
#                 self.start_waypoint_based_parking('1.csv')

#         # waypoint 1750-1800 구간에서 driving_control 감지 활성화
#         elif self.parking_detection_start_1800 <= self.current_waypoint_index < self.parking_waypoint_1800:
#             self.parking_1800_detection_active = True
        
#         # waypoint 1800에서 감지된 신호에 따른 주차 실행
#         elif self.current_waypoint_index == self.parking_waypoint_1800 and not self.parking_1800_triggered:
#             if self.state == 'AUTONOMOUS_DRIVING' and self.parking_1800_detected_signal is not None:
#                 self.parking_1800_triggered = True
#                 if self.parking_1800_detected_signal == 5:
#                     csv_file = '4.csv'
#                 elif self.parking_1800_detected_signal == 6:
#                     csv_file = '5.csv'
#                 else:
#                     csv_file = None
                
#                 if csv_file:
#                     self.get_logger().info(f"waypoint {self.parking_waypoint_1800} 도달! 감지된 신호({self.parking_1800_detected_signal})에 따른 주차 시작 ({csv_file})")
#                     self.current_parking_sequence = [csv_file]
#                     self.start_waypoint_based_parking(csv_file)
                
#                 self.parking_1800_detection_active = False
#                 self.parking_1800_detected_signal = None
#         # ========================================================================

#         # ================== [추가] 속도 제한 구간 진입/이탈 감지 ==================
#         is_in_speed_limit_zone = self.speed_limit_start_waypoint <= self.current_waypoint_index <= self.speed_limit_end_waypoint
#         was_in_speed_limit_zone = self.speed_limit_start_waypoint <= previous_waypoint <= self.speed_limit_end_waypoint
        
#         # 속도 제한 구간 진입 감지
#         if is_in_speed_limit_zone and not was_in_speed_limit_zone:
#             if not self.is_in_speed_limit_zone:
#                 self.speed_limit_entry_velocity = self.target_velocity  # 현재 목표 속도 저장
#                 self.is_in_speed_limit_zone = True
#                 self.get_logger().info(f"속도 제한 구간 진입! waypoint {self.current_waypoint_index}. "
#                                      f"속도를 {self.speed_limit_entry_velocity:.2f} -> {self.speed_limit_target_velocity:.2f} m/s로 조정합니다.")
        
#         # 속도 제한 구간 이탈 감지
#         elif not is_in_speed_limit_zone and was_in_speed_limit_zone:
#             if self.is_in_speed_limit_zone:
#                 self.is_in_speed_limit_zone = False
#                 # 속도 제한 구간 이탈 시 목표 속도를 저장된 진입 속도로 복원
#                 self.target_velocity = self.speed_limit_entry_velocity
#                 self.get_logger().info(f"속도 제한 구간 이탈! waypoint {self.current_waypoint_index}. "
#                                      f"속도를 {self.speed_limit_target_velocity:.2f} -> {self.speed_limit_entry_velocity:.2f} m/s로 복원합니다.")
#         # ======================================================================
    
#     # CSV 로더 유틸
#     def _load_csv_into_playback(self, file_path: str) -> bool:
#         try:
#             with open(file_path, 'r') as csvfile:
#                 reader = csv.reader(csvfile)
#                 # 헤더 유무에 관계없이 첫 행이 숫자가 아니면 헤더로 간주하고 스킵
#                 peek = next(reader, None)
#                 if peek is None:
#                     return False
#                 try:
#                     float(peek[1]); float(peek[2])
#                     rows = [peek] + [row for row in reader]
#                 except Exception:
#                     rows = [row for row in reader]
#                 if not rows:
#                     return False
#                 self.playback_data = rows
#                 self.playback_index = 0
#                 return True
#         except Exception as e:
#             self.get_logger().error(f"CSV 파일을 읽는 데 실패했습니다: {e}")
#             return False
    
#     # --- 신호등 제어 처리 함수들 ---
#     def handle_stop_signal(self):
#         """정지 신호 처리. 현재 설정된 정지점에서 정지."""
#         if self.traffic_light_state == 'NORMAL':
#             self.gradual_stop_start_waypoint = self.current_waypoint_index
            
#             # 현재 활성화된 정지점 사용, 없으면 첫 번째 정지점 사용
#             if self.current_stop_target is not None:
#                 self.stop_target_waypoint = self.current_stop_target
#             else:
#                 self.stop_target_waypoint = self.TRAFFIC_STOP_WAYPOINTS[0]
            
#             self.traffic_light_state = 'PREPARING_STOP'
#             self.original_target_velocity = self.target_velocity
            
#             distance_to_stop = self.stop_target_waypoint - self.current_waypoint_index
#             self.get_logger().info(f"waypoint {self.stop_target_waypoint}에서 정지 예정. 현재 위치: {self.current_waypoint_index}, 남은 거리: {distance_to_stop}")
            
#             if distance_to_stop <= 0:
#                 # 이미 정지점을 지났으면 즉시 정지
#                 self.traffic_light_state = 'STOPPED'
#                 self.get_logger().warn(f"이미 정지 지점({self.stop_target_waypoint})을 지났습니다. 즉시 정지합니다.")

#     def handle_go_signal(self):
#         """진행 신호 처리"""
#         if self.traffic_light_state in ['PREPARING_STOP', 'STOPPED']:
#             self.traffic_light_state = 'NORMAL'
#             self.gradual_stop_start_waypoint = None
#             self.current_stop_target = None  # 정지점 타겟 리셋
#             if hasattr(self, 'original_target_velocity'):
#                 self.target_velocity = self.original_target_velocity
#             self.pd_controller.reset()
#             self.get_logger().info("정상 주행 재개")
    
#     def handle_prepare_signal(self):
#         """준비 신호 처리 (노란불)"""
#         self.handle_stop_signal()
    
#     def calculate_traffic_light_velocity(self):
#         """신호등 제어에 따른 목표 속도 계산"""
#         if self.traffic_light_state == 'NORMAL':
#             return self.target_velocity
        
#         elif self.traffic_light_state == 'PREPARING_STOP':
#             if self.current_waypoint_index >= self.stop_target_waypoint:
#                 self.traffic_light_state = 'STOPPED'
#                 return 0.0
            
#             if self.gradual_stop_start_waypoint is not None:
#                 remaining_waypoints = self.stop_target_waypoint - self.current_waypoint_index
#                 total_waypoints = self.stop_target_waypoint - self.gradual_stop_start_waypoint
                
#                 if total_waypoints > 0:
#                     speed_ratio = remaining_waypoints / total_waypoints
#                     target_speed = self.original_target_velocity * max(0.0, speed_ratio)
#                     return target_speed
#                 else:
#                     # 감속할 거리가 없으면 즉시 정지
#                     return 0.0
#             else:
#                 return self.target_velocity
        
#         elif self.traffic_light_state == 'STOPPED':
#             return 0.0
        
#         return self.target_velocity

#     def calculate_slope_velocity(self):
#         """경사면 하강 시 속도 제어"""
#         if self.is_on_slope:
#             # 경사면에서는 1.5 m/s로 속도 제한
#             return self.slope_target_velocity
#         else:
#             # 경사면이 아닐 때는 원래 목표 속도 사용
#             return self.target_velocity

#     def calculate_special_zone_velocity(self):
#         """특수 구간 속도 제어"""
#         if self.is_approaching_special_zone:
#             # 접근 구간 (waypoint 155-184): 점진적 감속
#             approach_start = self.special_zone_trigger_waypoint - self.special_zone_approach_distance  # 155
#             progress = (self.current_waypoint_index - approach_start) / self.special_zone_approach_distance  # 0~1
#             progress = max(0.0, min(1.0, progress))  # 0~1 범위로 제한
            
#             # 원래 속도에서 목표 속도로 선형 감속
#             target_velocity = self.special_zone_entry_velocity * (1 - progress) + self.special_zone_target_velocity * progress
#             return target_velocity
            
#         elif self.is_in_special_zone:
#             # 특수 구간 내부 (waypoint 185-300): 1.5 m/s로 속도 유지
#             return self.special_zone_target_velocity
#         else:
#             # 일반 구간: 원래 목표 속도 사용
#             return self.target_velocity

#     def calculate_speed_limit_zone_velocity(self):
#         """속도 제한 구간 속도 제어"""
#         if self.is_in_speed_limit_zone:
#             # 속도 제한 구간에서는 1.5 m/s로 속도 제한
#             return self.speed_limit_target_velocity
#         else:
#             # 속도 제한 구간이 아닐 때는 원래 목표 속도 사용
#             return self.target_velocity
            

#     # --- 주차 경로 재생 로직 ---
#     def start_waypoint_based_parking(self, csv_file):
#         """waypoint 기반 자동 주차 시작"""
#         self.state = 'PARKING_MANEUVER'
#         self.control_timer.cancel()
#         self.pd_controller.reset()
#         self.get_logger().info(f"waypoint 기반 주차 모드 진입: {csv_file} 실행을 시작합니다.")
#         self.post_playback_followup_done = False
        
#         if not os.path.exists(csv_file):
#             self.get_logger().error(f"주차 파일 '{csv_file}'을 찾을 수 없습니다! 자율주행으로 복귀합니다.")
#             self.finish_parking_maneuver()
#             return
            
#         self.get_logger().info(f"주차 경로 파일을 로드합니다: '{csv_file}'")
#         if not self._load_csv_into_playback(csv_file):
#             self.get_logger().error(f"CSV 파일 '{csv_file}'이 비어있습니다! 주차를 취소합니다.")
#             self.finish_parking_maneuver()
#             return
            
#         self.playback_index = 0
#         self.playback_timer = self.create_timer(0.05, self.playback_loop)

#     def start_parking_maneuver(self):
#         self.state = 'PARKING_MANEUVER'
#         self.control_timer.cancel()
#         self.pd_controller.reset()
#         self.get_logger().info("주차 모드 진입: PID 제어기를 리셋했습니다.")
#         self.post_playback_followup_done = False
#         list_of_files = glob.glob('recording_*.csv')
#         if not list_of_files:
#             self.get_logger().error("주차에 사용할 녹화 파일을 찾을 수 없습니다! 자율주행으로 복귀합니다.")
#             self.finish_parking_maneuver()
#             return
#         latest_file = max(list_of_files, key=os.path.getctime)
#         self.get_logger().info(f"주차 경로 파일을 로드합니다: '{latest_file}'")
#         if not self._load_csv_into_playback(latest_file):
#             self.get_logger().error("CSV 파일이 비어있습니다! 주차를 취소합니다.")
#             self.finish_parking_maneuver()
#             return
#         self.playback_index = 0
#         self.playback_timer = self.create_timer(0.05, self.playback_loop)

#     def playback_loop(self):
#         if self.playback_index >= len(self.playback_data):
#             # waypoint 기반 주차인지 확인
#             is_waypoint_based = len(self.current_parking_sequence) > 0
            
#             if is_waypoint_based:
#                 # waypoint 기반 주차 시퀀스 처리
#                 current_file = self.current_parking_sequence[0] if self.current_parking_sequence else None
#                 next_file = None
                
#                 # 현재 파일에 따른 후속 파일 결정
#                 if current_file == '1.csv' and not self.post_playback_followup_done:
#                     if self.driving_control_signal == 5:
#                         next_file = '2.csv'
#                     elif self.driving_control_signal == 6:
#                         next_file = '3.csv'
                
#                 if next_file and os.path.exists(next_file):
#                     self.get_logger().info(f"1.csv 완료. driving_control={self.driving_control_signal}에 따라 '{next_file}' 연결 재생")
#                     if self._load_csv_into_playback(next_file):
#                         self.current_parking_sequence = [next_file]  # 시퀀스 업데이트
#                         self.post_playback_followup_done = True
#                         return  # 다음 루프부터 새 파일 재생
#                     else:
#                         self.get_logger().error(f"'{next_file}' 로드 실패. 주차를 종료합니다.")
                
#                 self.get_logger().info(f"waypoint 기반 주차 시퀀스 완료: {current_file}")
#             else:
#                 # 기존 /parking_command_1 기반 주차 처리
#                 if not self.post_playback_followup_done:
#                     next_file = None
#                     if self.driving_control_signal == 5:
#                         next_file = '1.csv'
#                     elif self.driving_control_signal == 6:
#                         next_file = '2.csv'
#                     if next_file and os.path.exists(next_file):
#                         self.get_logger().info(f"후속 재생 조건 감지(driving_control={self.driving_control_signal}). '{next_file}' 재생 시작")
#                         if self._load_csv_into_playback(next_file):
#                             self.post_playback_followup_done = True
#                             return  # 다음 루프부터 새 파일 재생
#                         else:
#                             self.get_logger().error(f"'{next_file}' 로드 실패. 주차를 종료합니다.")
#                     else:
#                         if next_file:
#                             self.get_logger().warn(f"'{next_file}' 파일을 찾을 수 없습니다. 주차를 종료합니다.")
                
#                 self.get_logger().info("주차 경로 재생을 완료했습니다.")
            
#             self.finish_parking_maneuver()
#             return
#         row = self.playback_data[self.playback_index]
#         try:
#             control_val = float(row[1])
#             angular_val = float(row[2])
#             control_msg = Float32()
#             control_msg.data = control_val
#             self.control_pub.publish(control_msg)
#             angular_target_msg = Float32()
#             angular_target_msg.data = angular_val
#             self.angular_target_pub.publish(angular_target_msg)
#             self.playback_index += 1
#         except (ValueError, IndexError) as e:
#             self.get_logger().error(f"주차 데이터 처리 중 오류 발생 (행 {self.playback_index}): {row}. 오류: {e}")
#             self.finish_parking_maneuver()

#     def finish_parking_maneuver(self):
#         if self.playback_timer:
#             self.playback_timer.destroy()
#             self.playback_timer = None
        
#         # 기본 제어 상태 리셋
#         self.pd_controller.reset()
#         self.is_ramping_up = False
#         self.ramp_up_start_time = None
#         self.target_velocity = 0.0
#         self.angular_target = 0.0
#         self.filtered_control_output = 0.0
        
#         # waypoint 기반 주차 상태 리셋
#         self.current_parking_sequence = []
#         self.post_playback_followup_done = False
        
#         self.get_logger().info("자율주행 모드로 복귀합니다. PID 제어기와 주차 상태를 리셋했습니다.")
#         self.state = 'AUTONOMOUS_DRIVING'
#         self.control_timer.reset()

#     # --- 자율주행 로직 (변경 없음) ---
#     def calculate_angle(self, target_velocity, angular_target):
#         if abs(angular_target) < 1e-6:
#             return 0.0
#         if abs(target_velocity) < 0.1:
#             return angular_target * 2.0
#         return math.atan2(self.Ld * angular_target, target_velocity) * 5

#     def calculate_ramp_up_velocity(self):
#         # 램프업 조건: 정지 상태에서 가속할 때
#         if abs(self.current_velocity) < 0.1 and self.target_velocity > 0.1:
#             if not self.is_ramping_up:
#                 self.is_ramping_up = True
#                 self.ramp_up_start_time = time.time()
#                 self.get_logger().info('속도 램프업 시작!')
#             elapsed_time = time.time() - self.ramp_up_start_time
#             if elapsed_time < self.ramp_up_duration:
#                 progress = elapsed_time / self.ramp_up_duration
#                 return self.target_velocity * progress
#             else:
#                 self.is_ramping_up = False
#                 return self.target_velocity
        
#         # 램프다운 조건: 현재 속도가 목표 속도보다 0.3 이상 클 때 (모든 감속 상황)
#         elif self.current_velocity > self.target_velocity + 0.3:
#             # 램프다운 변수 초기화
#             if not hasattr(self, 'is_ramping_down'):
#                 self.is_ramping_down = False
#             if not hasattr(self, 'ramp_down_start_time'):
#                 self.ramp_down_start_time = None
#             if not hasattr(self, 'ramp_down_initial_velocity'):
#                 self.ramp_down_initial_velocity = 0.0
            
#             if not self.is_ramping_down:
#                 self.is_ramping_down = True
#                 self.ramp_down_start_time = time.time()
#                 self.ramp_down_initial_velocity = self.current_velocity
#                 self.get_logger().info(f'속도 람프다운 시작! {self.ramp_down_initial_velocity:.2f} -> {self.target_velocity:.2f}')
            
#             elapsed_time = time.time() - self.ramp_down_start_time
#             if elapsed_time < self.ramp_up_duration:  # 같은 지속 시간 사용
#                 progress = elapsed_time / self.ramp_up_duration
#                 # 초기 속도에서 목표 속도까지 선형적으로 감소
#                 return self.ramp_down_initial_velocity * (1 - progress) + self.target_velocity * progress
#             else:
#                 self.is_ramping_down = False
#                 return self.target_velocity
        
#         else:
#             # 일반적인 경우 - 램프 상태 리셋
#             self.is_ramping_up = False
#             if hasattr(self, 'is_ramping_down'):
#                 self.is_ramping_down = False
#             return self.target_velocity

#     def control_loop(self):
#         # ================== [수정] 긴급 장애물 정지 로직 (최우선 순위) ==================
#         if self.obstacle_stop_active:
#             # 1초간 점진적 긴급 정지 로직
#             if self.emergency_stop_start_time is not None:
#                 elapsed_time = time.time() - self.emergency_stop_start_time
#                 if elapsed_time < self.emergency_stop_duration:
#                     # 1초 동안 초기 속도에서 0으로 선형 감속
#                     progress = elapsed_time / self.emergency_stop_duration
#                     effective_target_velocity = self.emergency_stop_initial_velocity * (1.0 - progress)
#                     self.get_logger().debug(f"긴급 정지 진행 중: {effective_target_velocity:.2f} m/s (진행률: {progress*100:.1f}%)")
#                 else:
#                     # 1초 경과 후 완전 정지
#                     effective_target_velocity = 0.0
#             else:
#                 # 안전상 즉시 정지 (정상적이지 않은 상황)
#                 effective_target_velocity = 0.0
#         else:
#             # 기존의 신호등 및 램프업 로직 수행
#             traffic_controlled_velocity = self.calculate_traffic_light_velocity()
            
#             # ================== [추가] 모든 구간 속도 제어 적용 ==================
#             slope_controlled_velocity = self.calculate_slope_velocity()
#             special_zone_controlled_velocity = self.calculate_special_zone_velocity()
#             speed_limit_zone_controlled_velocity = self.calculate_speed_limit_zone_velocity()
            
#             # 신호등, 경사면, 특수 구간, 속도 제한 구간 제어 중 가장 제한적인 속도 선택
#             final_controlled_velocity = min(
#                 traffic_controlled_velocity, 
#                 slope_controlled_velocity, 
#                 special_zone_controlled_velocity,
#                 speed_limit_zone_controlled_velocity
#             )
#             # =========================================================================
            
#             original_target = self.target_velocity
#             self.target_velocity = final_controlled_velocity
#             effective_target_velocity = self.calculate_ramp_up_velocity()
#             self.target_velocity = original_target
#         # =========================================================================

#         # --- PD 제어 및 퍼블리시 로직 ---
#         control_signal = self.pd_controller.compute(
#             setpoint=effective_target_velocity,
#             measured_value=self.current_velocity
#         )
#         raw_control_output = max(0.0, min(100.0, control_signal * 20.0))
#         self.filtered_control_output = (self.output_filter_alpha * raw_control_output + 
#                                         (1 - self.output_filter_alpha) * self.filtered_control_output)
#         control_output = self.filtered_control_output

#         # ================== [추가] 특수 구간 control_signal 3초간 점진적 유지 ==================
#         if self.special_zone_control_active and self.special_zone_control_start_time is not None:
#             elapsed_time = time.time() - self.special_zone_control_start_time
#             if elapsed_time < self.special_zone_control_duration:
#                 # 점진적으로 control_signal을 목표값으로 변경 (부드러운 전환)
#                 transition_duration = 0.5  # 0.5초에 걸쳐 점진적 전환
#                 if elapsed_time < transition_duration:
#                     # 0.5초간 현재값에서 목표값으로 점진적 변화
#                     progress = elapsed_time / transition_duration
#                     if not hasattr(self, 'special_zone_initial_control_output'):
#                         self.special_zone_initial_control_output = control_output
                    
#                     target_control = self.special_zone_control_signal
#                     control_output = (self.special_zone_initial_control_output * (1 - progress) + 
#                                     target_control * progress)
#                     self.get_logger().debug(f"특수 구간 control_signal 점진적 전환: {control_output:.1f} (진행률: {progress*100:.1f}%)")
#                 else:
#                     # 0.5초 후부터는 목표값 유지
#                     control_output = self.special_zone_control_signal
#                     self.get_logger().debug(f"특수 구간 control_signal 유지: {control_output:.1f}")
#             else:
#                 # 3초 경과 후 특수 구간 control_signal 제어 비활성화
#                 self.special_zone_control_active = False
#                 if hasattr(self, 'special_zone_initial_control_output'):
#                     delattr(self, 'special_zone_initial_control_output')  # 초기값 정리
#                 self.get_logger().info("특수 구간 3초 control_signal 유지 완료. 정상 제어로 전환합니다.")
#         # =====================================================================================
#         control_msg = Float32()
#         angular_target_msg = Float32()
        
#         # effective_target_velocity를 기준으로 각도 계산
#         angular_target_msg.data = self.calculate_angle(effective_target_velocity, self.angular_target)
#         self.angular_target_pub.publish(angular_target_msg)
        
#         control_msg.data = control_output
#         self.control_pub.publish(control_msg)
        
#         if self.traffic_light_state != 'NORMAL':
#             self.get_logger().debug(f"신호등 제어: 상태={self.traffic_light_state}, "
#                                   f"waypoint={self.current_waypoint_index}, "
#                                   f"목표속도={effective_target_velocity:.2f}, "
#                                   f"제어출력={control_output:.2f}")
        
#         if self.is_on_slope:
#             self.get_logger().debug(f"경사면 주행: waypoint={self.current_waypoint_index}, "
#                                   f"목표속도={effective_target_velocity:.2f}, "
#                                   f"제어출력={control_output:.2f}")

#         if self.is_approaching_special_zone:
#             self.get_logger().debug(f"특수 구간 접근 중: waypoint={self.current_waypoint_index}, "
#                                   f"목표속도={effective_target_velocity:.2f}, "
#                                   f"제어출력={control_output:.2f}")
        
#         if self.is_in_special_zone:
#             self.get_logger().debug(f"특수 구간 주행: waypoint={self.current_waypoint_index}, "
#                                   f"목표속도={effective_target_velocity:.2f}, "
#                                   f"제어출력={control_output:.2f}")

#         if self.is_in_speed_limit_zone:
#             self.get_logger().debug(f"속도 제한 구간 주행: waypoint={self.current_waypoint_index}, "
#                                   f"목표속도={effective_target_velocity:.2f}, "
#                                   f"제어출력={control_output:.2f}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = PDControllerNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# """정지선 185 pwm 1
# 경사면  300
# ㄱ자 끝 567
# 1번 신호등 646
# S 1048
# 2번 신호등 1200 ->점검 필요
# T전 1276"""
import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from std_msgs.msg import Float32, Bool, Int32
import math
import csv
import glob
import os

class PIDController:
    """PID 제어기 클래스"""
    def __init__(self, kp=2.0, ki=0.3, kd=0.1):
        self.kp = kp  # 비례 게인
        self.kd = kd  # 미분 게인
        self.ki = ki  # 적분 게인
        self.previous_error = 0.0
        self.previous_time = None
        self.integral_error = 0.0  # 적분항 누적 변수 추가
        self.Ld = 0.7315

        # 미분항 필터링을 위한 변수들
        self.derivative_filter_alpha = 0.1  # 저주파 통과 필터 계수 (0~1)
        self.filtered_derivative = 0.0

    def compute(self, setpoint, measured_value):
        """PID 제어 신호 계산"""
        current_time = time.time()

        # 오차 계산 (setpoint - measured_value)
        error = setpoint - measured_value

        # 시간 초기화
        if self.previous_time is None:
            self.previous_time = current_time
            self.previous_error = error
            self.integral_error = 0.0
            return self.kp * error

        # 시간 간격 계산
        dt = current_time - self.previous_time

        # 적분항 계산 (오차의 누적합)
        if dt > 0:
            self.integral_error += error * dt
            # 적분항 윈드업 방지 (제한) - 더 안전한 범위로 설정
            self.integral_error = max(-10.0, min(10.0, self.integral_error))
        else:
            dt = 0.01  # 최소 시간 간격 설정

        # 미분항 계산 (오차의 변화율) + 노이즈 필터링
        if dt > 0:
            raw_derivative = (error - self.previous_error) / dt
            # 저주파 통과 필터 적용 (exponential moving average)
            self.filtered_derivative = (self.derivative_filter_alpha * raw_derivative +
                                       (1 - self.derivative_filter_alpha) * self.filtered_derivative)
            derivative = self.filtered_derivative
        else:
            derivative = self.filtered_derivative

        # PID 제어 신호 계산
        control_signal = (self.kp * error + self.ki * self.integral_error + self.kd * derivative)

        # 이전 값들 업데이트
        self.previous_error = error
        self.previous_time = current_time

        return control_signal

    def reset(self):
        """PID 제어기 리셋 (목표 변경 시 사용)"""
        self.previous_error = 0.0
        self.previous_time = None
        self.integral_error = 0.0
        self.filtered_derivative = 0.0

class PDControllerNode(Node):
    """
    PD 제어기 ROS2 노드.
    - AUTONOMOUS_DRIVING: /cmd_vel을 받아 속도 및 조향 제어.
    - PARKING_MANEUVER: /parking_command_1 신호를 받으면 저장된 CSV 경로를 재생.
    """
    def __init__(self):
        super().__init__('pd_controller_node')

        # --- 상태 관리 변수 ---
        self.state = 'AUTONOMOUS_DRIVING'  # 'AUTONOMOUS_DRIVING' 또는 'PARKING_MANEUVER'
        self.playback_data = []
        self.playback_index = 0
        self.playback_timer = None
        self.post_playback_followup_done = False  # 첫 재생 이후 후속 재생 여부
        self.is_ramping_down = False
        self.ramp_down_start_time = None
        self.ramp_down_initial_velocity = 0.0

        # ================== [추가] waypoint 기반 자동 주차 변수 ==================
        self.parking_waypoint_1276 = 1266  # 첫 번째 주차 구간
        self.parking_waypoint_1800 = 2554  # 두 번째 주차 구간
        self.parking_detection_start_1800 = 1740  # waypoint 1800의 감지 시작점
        self.parking_1276_triggered = False  # waypoint 1276 주차 실행 여부
        self.parking_1800_triggered = False  # waypoint 1800 주차 실행 여부
        self.parking_1800_detection_active = False  # waypoint 1800 감지 활성 상태
        self.parking_1800_detected_signal = None  # waypoint 1800에서 감지된 신호 저장
        self.current_parking_sequence = []  # 현재 실행 중인 주차 시퀀스
        # =======================================================================

        # --- 신호등 제어 상태 초기값 ---
        self.traffic_light_state = 'NORMAL'  # 'NORMAL' | 'PREPARING_STOP' | 'STOPPED'
        self.driving_control_signal = 1  # 0: stop, 1: go, 2: prepare
        self.current_waypoint_index = 0
        self.stop_target_waypoint = 0
        self.gradual_stop_start_waypoint = None
        self.original_target_velocity = 0.0

        # *** 다중 신호등 정지 웨이포인트 설정 ***
        self.TRAFFIC_STOP_WAYPOINTS = [642, 1183,1660]  # 신호등 정지점들
        self.APPROACH_DISTANCE = 30  # 정지점에 가까워지는 거리 (30 waypoint 전부터 감지)
        self.current_stop_target = None  # 현재 활성화된 정지점

        # *** driving_control 지속 체크를 위한 변수 ***
        self.driving_control_history = []  # 최근 신호 이력
        self.SIGNAL_CHECK_DURATION = 1.0  # 2초간 지속적으로 0이 와야 정지
        self.signal_check_timer = time.time()

        # PD 제어기 초기화 (안정성을 위해 게인 값 조정)
        self.pd_controller = PIDController(kp=1.8, ki=0.8, kd=0.2)

        # 목표 속도 설정 (m/s)
        self.target_velocity = 0.0

        # 현재 측정된 속도
        self.current_velocity = 0.0

        # 목표 각속도 초기화
        self.angular_target = 0.0

        # 차량 파라미터
        self.Ld = 0.7315  # 차축 거리

        # 램프업 관련 변수
        self.ramp_up_duration = 1.0  # 1초 동안 램프업
        self.ramp_up_start_time = None
        self.is_ramping_up = False

        # 제어 신호 스무딩을 위한 변수
        self.output_filter_alpha = 0.3  # 출력 스무딩 필터 계수 (0~1)
        self.filtered_control_output = 0.0

        # ================== [추가] 긴급 장애물 정지 상태 변수 ==================
        self.obstacle_stop_active = False
        self.emergency_stop_start_time = None
        self.emergency_stop_initial_velocity = 0.0
        self.emergency_stop_duration = 1.0  # 1초 동안 점진적 감속
        # =================================================================

        # ================== [추가] 경사면 하강 제어 변수 ==================
        self.is_on_slope = False
        self.slope_target_velocity = 1.5  # 경사면 하강 시 목표 속도 (m/s)
        self.slope_start_waypoint = 65000000
        self.slope_end_waypoint = 700000000
        self.slope_entry_velocity = 0.0  # 경사면 진입 시 원래 속도 저장
        # ================================================================

        # ================== [추가] 특수 구간 제어 변수 (waypoint 185-300) ==================
        self.is_in_special_zone = False
        self.special_zone_trigger_waypoint = 175
        self.special_zone_end_waypoint = 300
        self.special_zone_approach_distance = 30  # 185 도달 30 waypoint 전부터 감속 시작 (waypoint 155부터)
        self.special_zone_target_velocity = 1.5  # 특수 구간 목표 속도 (m/s)
        self.special_zone_control_signal = 1.0  # 3초간 유지할 control_signal 값
        self.special_zone_control_duration = 3.0  # 3초간 control_signal 유지
        self.special_zone_control_start_time = None
        self.special_zone_control_active = False
        self.special_zone_entry_velocity = 0.0  # 특수 구간 진입 시 원래 속도 저장
        self.is_approaching_special_zone = False  # 특수 구간 접근 중 여부 (waypoint 155-184)
        # =================================================================================

        # ================== [추가] 속도 제한 구간 제어 변수 (waypoint 3000-3050) ==================
        self.is_in_speed_limit_zone = False
        self.speed_limit_start_waypoint = 3000000000000
        self.speed_limit_end_waypoint = 305000000000
        self.speed_limit_target_velocity = 1.5  # 속도 제한 구간 목표 속도 (m/s)
        self.speed_limit_entry_velocity = 0.0  # 속도 제한 구간 진입 시 원래 속도 저장
        # =====================================================================================

        # --- Subscribers ---
        self.velocity_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            '/ublox_gps_node/fix_velocity',
            self.velocity_callback,
            10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.parking_sub = self.create_subscription(
            Bool,
            '/parking_command_1',
            self.parking_command_callback,
            10
        )

        self.driving_control_sub = self.create_subscription(
            Int32,
            'main_vision/driving_control',  # 토픽 이름은 이미 수정된 상태
            self.driving_control_callback,
            10
        )

        self.waypoint_index_sub = self.create_subscription(
            Int32,
            'waypoint_index',
            self.waypoint_index_callback,
            10
        )

        # --- Publishers ---
        self.control_pub = self.create_publisher(Float32, '/control_signal', 10)
        self.angular_target_pub = self.create_publisher(Float32, 'angular_target', 10)

        # 자율주행 제어 루프 타이머 (20Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('PD Controller Node가 시작되었습니다. 현재 상태: AUTONOMOUS_DRIVING')
        self.get_logger().info(f'다중 신호등 정지점: {self.TRAFFIC_STOP_WAYPOINTS}')
        self.get_logger().info(f'신호등 정지 감지 범위: 각 정지점 -{self.APPROACH_DISTANCE} waypoint 전부터 감지')
        self.get_logger().info(f'PD 게인 - Kp: {self.pd_controller.kp}, Kd: {self.pd_controller.kd}')
        self.get_logger().info('신호등 제어: driving_control=0 (지속신호) 또는 driving_control=2 (즉시정지) 시 정지, driving_control=1 시 진행')
        self.get_logger().info(f'경사면 하강 제어: waypoint {self.slope_start_waypoint}-{self.slope_end_waypoint} 구간에서 속도 {self.slope_target_velocity} m/s로 제한')
        self.get_logger().info(f'특수 구간 제어: waypoint {self.special_zone_trigger_waypoint}에서 control_signal {self.special_zone_control_signal} 3초 유지, {self.special_zone_trigger_waypoint}-{self.special_zone_end_waypoint} 구간에서 속도 {self.special_zone_target_velocity} m/s로 제한')
        self.get_logger().info(f'속도 제한 구간: waypoint {self.speed_limit_start_waypoint}-{self.speed_limit_end_waypoint} 구간에서 속도 {self.speed_limit_target_velocity} m/s로 제한')
        self.get_logger().info('긴급 정지: waypoint 800-900 구간에서 driving_control=3 신호 시 1초간 점진적 정지')
        self.get_logger().info(f'waypoint 기반 자동 주차: waypoint {self.parking_waypoint_1276}에서 1.csv→2.csv/3.csv, waypoint {self.parking_waypoint_1800}({self.parking_detection_start_1800}부터 감지)에서 4.csv/5.csv')

    # --- 콜백 함수들 ---
    def velocity_callback(self, msg: TwistWithCovarianceStamped):
        linear_velocity = msg.twist.twist.linear
        self.current_velocity = (linear_velocity.x**2 + linear_velocity.y**2)**0.5

    def cmd_vel_callback(self, msg: Twist):
        if self.state != 'AUTONOMOUS_DRIVING':
            return

        if abs(self.target_velocity - msg.linear.x) > 0.1:
            self.pd_controller.reset()
            self.get_logger().info(f"목표 속도 변경: {self.target_velocity:.2f} -> {msg.linear.x:.2f} m/s, PID 리셋")

        self.target_velocity = msg.linear.x
        self.angular_target = msg.angular.z

        # ================== [수정] 모든 제한 구간 이탈 시 속도 복원 ==================
        # 각 제한 구간이 아닐 때만 해당 entry_velocity 업데이트
        if not self.is_on_slope:
            self.slope_entry_velocity = msg.linear.x
        if not self.is_in_special_zone and not self.is_approaching_special_zone:
            self.special_zone_entry_velocity = msg.linear.x
        if not self.is_in_speed_limit_zone:
            self.speed_limit_entry_velocity = msg.linear.x
        # =============================================================================

        if self.traffic_light_state == 'NORMAL':
            self.original_target_velocity = msg.linear.x

    def parking_command_callback(self, msg: Bool):
        if msg.data and self.state == 'AUTONOMOUS_DRIVING':
            self.get_logger().info("주차 명령을 수신했습니다. 경로 재생을 시작합니다...")
            self.start_parking_maneuver()


    def driving_control_callback(self, msg: Int32):
        current_time = time.time()
        self.driving_control_signal = msg.data

        # ================== [수정] 긴급 장애물 정지 조건 감지 ==================
        # 1. 조건 정의: waypoint가 800-900 사이이고, 신호가 3인가?
        is_in_obstacle_zone = 2087 <= self.current_waypoint_index <= 2400
        # 태홍이 시각화 바탕으로 처리 예정 => 가속 구간

        # 2. 긴급 정지 상태 활성화
        if msg.data == 3 and is_in_obstacle_zone:
            if not self.obstacle_stop_active:
                self.get_logger().warn(
                    f"장애물 정지 신호(3) 수신! (waypoint: {self.current_waypoint_index}). 1초간 점진적 긴급 정지를 시작합니다."
                )
                self.obstacle_stop_active = True
                self.emergency_stop_start_time = time.time()
                self.emergency_stop_initial_velocity = self.current_velocity

        # 3. 긴급 정지 상태 비활성화 (신호가 3이 아니거나, 구간을 벗어났을 때)
        elif self.obstacle_stop_active and (msg.data != 3 or not is_in_obstacle_zone):
            self.get_logger().info("장애물 신호가 해제되었거나 정지 구간을 이탈했습니다. 정상 주행을 재개합니다.")
            self.obstacle_stop_active = False
            self.emergency_stop_start_time = None
            self.pd_controller.reset()  # 부드러운 재출발을 위해 PID 리셋
        # ===================================================================

        # ================== [추가] waypoint 기반 주차 신호 감지 ==================
        # waypoint 1750-1800 구간에서 driving_control 5/6 감지
        if self.parking_1800_detection_active and msg.data in [5, 6]:
            if self.parking_1800_detected_signal is None:  # 첫 번째 신호만 저장
                self.parking_1800_detected_signal = msg.data
                self.get_logger().info(f"waypoint 1800 주차 신호 감지: driving_control={msg.data}")
        # =====================================================================

        # --- 기존 신호등 정지 로직 (수정 없음) ---
        self.driving_control_history.append((current_time, msg.data))

        self.driving_control_history = [
            (t, signal) for t, signal in self.driving_control_history
            if current_time - t <= self.SIGNAL_CHECK_DURATION
        ]

        # ================== [수정] 다중 정지점 및 driving_control=2 처리 ==================
        # 현재 waypoint 근처의 정지점 찾기
        nearby_stop_point = None
        for stop_point in self.TRAFFIC_STOP_WAYPOINTS:
            if (stop_point - self.APPROACH_DISTANCE) <= self.current_waypoint_index <= stop_point:
                nearby_stop_point = stop_point
                break

        if nearby_stop_point is not None:
            # 정지 신호 처리 (driving_control=0 또는 2)
            if (self.is_continuous_stop_signal() or msg.data == 2):
                if self.traffic_light_state == 'NORMAL':
                    signal_type = "지속적인 정지" if self.is_continuous_stop_signal() else "즉시 정지(2)"
                    self.get_logger().info(f"waypoint {nearby_stop_point} 근처에서 {signal_type} 신호 감지 (현재: {self.current_waypoint_index})")
                    self.current_stop_target = nearby_stop_point
                    self.handle_stop_signal()
            # 진행 신호 처리 (driving_control=1)
            elif msg.data == 1 and self.traffic_light_state in ['PREPARING_STOP', 'STOPPED']:
                self.get_logger().info("진행 신호 수신 - 정상 주행 재개")
                self.handle_go_signal()
        else:
            # 정지점 근처가 아닐 때도 진행 신호 처리
            if msg.data == 1 and self.traffic_light_state in ['PREPARING_STOP', 'STOPPED']:
                self.get_logger().info("진행 신호 수신 - 정상 주행 재개")
                self.handle_go_signal()
        # ==============================================================================

    def is_continuous_stop_signal(self):
        """지속적으로 정지 신호(0)가 들어오는지 확인"""
        if len(self.driving_control_history) < 5:  # 최소 5개의 신호가 필요
            return False

        # 최근 신호들이 모두 0인지 확인
        recent_signals = [signal for _, signal in self.driving_control_history[-5:]]  # 최근 5개
        return all(signal == 0 for signal in recent_signals)

    def waypoint_index_callback(self, msg: Int32):
        previous_waypoint = self.current_waypoint_index
        self.current_waypoint_index = msg.data

        # ================== [추가] 경사면 진입/이탈 감지 ==================
        is_in_slope_zone = self.slope_start_waypoint <= self.current_waypoint_index <= self.slope_end_waypoint
        was_in_slope_zone = self.slope_start_waypoint <= previous_waypoint <= self.slope_end_waypoint

        # 경사면 진입 감지
        if is_in_slope_zone and not was_in_slope_zone:
            if not self.is_on_slope:
                self.slope_entry_velocity = self.target_velocity  # 현재 목표 속도 저장
                self.is_on_slope = True
                self.get_logger().info(f"경사면 진입 감지! waypoint {self.current_waypoint_index}. "
                                     f"속도를 {self.slope_entry_velocity:.2f} -> {self.slope_target_velocity:.2f} m/s로 조정합니다.")

        # 경사면 이탈 감지
        elif not is_in_slope_zone and was_in_slope_zone:
            if self.is_on_slope:
                self.is_on_slope = False
                # 경사면 이탈 시 목표 속도를 저장된 진입 속도로 복원
                self.target_velocity = self.slope_entry_velocity
                self.get_logger().info(f"경사면 이탈 감지! waypoint {self.current_waypoint_index}. "
                                     f"속도를 {self.slope_target_velocity:.2f} -> {self.slope_entry_velocity:.2f} m/s로 복원합니다.")
        # ==============================================================

        # ================== [수정] 특수 구간 접근/진입/이탈 감지 ==================
        approach_start = self.special_zone_trigger_waypoint - self.special_zone_approach_distance  # waypoint 155

        # waypoint 155에서 접근 구간 시작 (점진적 감속 시작)
        if self.current_waypoint_index == approach_start and previous_waypoint != approach_start:
            if not self.is_approaching_special_zone and not self.is_in_special_zone:
                self.special_zone_entry_velocity = self.target_velocity  # 현재 목표 속도 저장 (예: 2.6)
                self.is_approaching_special_zone = True
                self.get_logger().info(f"오르막길 접근 시작! waypoint {self.current_waypoint_index}. "
                                     f"점진적 감속 시작: {self.special_zone_entry_velocity:.2f} -> {self.special_zone_target_velocity:.2f} m/s")

        # waypoint 185에서 특수 구간 진입 (control_signal 제어 시작)
        elif self.current_waypoint_index == self.special_zone_trigger_waypoint and previous_waypoint != self.special_zone_trigger_waypoint:
            if self.is_approaching_special_zone and not self.is_in_special_zone:
                self.is_in_special_zone = True
                self.is_approaching_special_zone = False
                self.special_zone_control_active = True
                self.special_zone_control_start_time = time.time()
                self.get_logger().info(f"특수 구간 진입! waypoint {self.current_waypoint_index}. "
                                     f"control_signal을 {self.special_zone_control_signal}로 3초간 유지합니다.")

        # waypoint 300에서 특수 구간 종료
        elif self.current_waypoint_index >= self.special_zone_end_waypoint and self.is_in_special_zone:
            self.is_in_special_zone = False
            self.special_zone_control_active = False
            self.special_zone_control_start_time = None
            # 특수 구간 이탈 시 목표 속도를 저장된 진입 속도로 복원
            self.target_velocity = self.special_zone_entry_velocity
            self.get_logger().info(f"특수 구간 이탈! waypoint {self.current_waypoint_index}. "
                                 f"속도를 {self.special_zone_target_velocity:.2f} -> {self.special_zone_entry_velocity:.2f} m/s로 복원합니다.")
        # ================================================================

        # ================== [추가] waypoint 기반 자동 주차 감지 ==================
        # waypoint 1276에서 자동으로 1.csv 실행
        if self.current_waypoint_index == self.parking_waypoint_1276 and not self.parking_1276_triggered:
            if self.state == 'AUTONOMOUS_DRIVING':
                self.parking_1276_triggered = True
                self.get_logger().info(f"waypoint {self.parking_waypoint_1276} 도달! 자동 주차 시작 (1.csv)")
                self.current_parking_sequence = ['t_in.csv']
                self.start_waypoint_based_parking('t_in.csv')

        # waypoint 1750-1800 구간에서 driving_control 감지 활성화
        elif self.parking_detection_start_1800 <= self.current_waypoint_index < self.parking_waypoint_1800:
            self.parking_1800_detection_active = True

        # waypoint 1800에서 감지된 신호에 따른 주차 실행
        elif self.current_waypoint_index == self.parking_waypoint_1800 and not self.parking_1800_triggered:
            if self.state == 'AUTONOMOUS_DRIVING' and self.parking_1800_detected_signal is not None:
                self.parking_1800_triggered = True
                if self.parking_1800_detected_signal == 5:
                    csv_file = 'parallel_right.csv'
                elif self.parking_1800_detected_signal == 6:
                    csv_file = 'parallel_left.csv'
                else:
                    csv_file = None

                if csv_file:
                    self.get_logger().info(f"waypoint {self.parking_waypoint_1800} 도달! 감지된 신호({self.parking_1800_detected_signal})에 따른 주차 시작 ({csv_file})")
                    self.current_parking_sequence = [csv_file]
                    self.start_waypoint_based_parking(csv_file)

                self.parking_1800_detection_active = False
                self.parking_1800_detected_signal = None
        # ========================================================================

        # ================== [추가] 속도 제한 구간 진입/이탈 감지 ==================
        is_in_speed_limit_zone = self.speed_limit_start_waypoint <= self.current_waypoint_index <= self.speed_limit_end_waypoint
        was_in_speed_limit_zone = self.speed_limit_start_waypoint <= previous_waypoint <= self.speed_limit_end_waypoint

        # 속도 제한 구간 진입 감지
        if is_in_speed_limit_zone and not was_in_speed_limit_zone:
            if not self.is_in_speed_limit_zone:
                self.speed_limit_entry_velocity = self.target_velocity  # 현재 목표 속도 저장
                self.is_in_speed_limit_zone = True
                self.get_logger().info(f"속도 제한 구간 진입! waypoint {self.current_waypoint_index}. "
                                     f"속도를 {self.speed_limit_entry_velocity:.2f} -> {self.speed_limit_target_velocity:.2f} m/s로 조정합니다.")

        # 속도 제한 구간 이탈 감지
        elif not is_in_speed_limit_zone and was_in_speed_limit_zone:
            if self.is_in_speed_limit_zone:
                self.is_in_speed_limit_zone = False
                # 속도 제한 구간 이탈 시 목표 속도를 저장된 진입 속도로 복원
                self.target_velocity = self.speed_limit_entry_velocity
                self.get_logger().info(f"속도 제한 구간 이탈! waypoint {self.current_waypoint_index}. "
                                     f"속도를 {self.speed_limit_target_velocity:.2f} -> {self.speed_limit_entry_velocity:.2f} m/s로 복원합니다.")
        # ======================================================================

    # CSV 로더 유틸
    def _load_csv_into_playback(self, file_path: str) -> bool:
        try:
            with open(file_path, 'r') as csvfile:
                reader = csv.reader(csvfile)
                # 헤더 유무에 관계없이 첫 행이 숫자가 아니면 헤더로 간주하고 스킵
                peek = next(reader, None)
                if peek is None:
                    return False
                try:
                    float(peek[1]); float(peek[2])
                    rows = [peek] + [row for row in reader]
                except Exception:
                    rows = [row for row in reader]
                if not rows:
                    return False
                self.playback_data = rows
                self.playback_index = 0
                return True
        except Exception as e:
            self.get_logger().error(f"CSV 파일을 읽는 데 실패했습니다: {e}")
            return False

    # --- 신호등 제어 처리 함수들 ---
    def handle_stop_signal(self):
        """정지 신호 처리. 현재 설정된 정지점에서 정지."""
        if self.traffic_light_state == 'NORMAL':
            self.gradual_stop_start_waypoint = self.current_waypoint_index

            # 현재 활성화된 정지점 사용, 없으면 첫 번째 정지점 사용
            if self.current_stop_target is not None:
                self.stop_target_waypoint = self.current_stop_target
            else:
                self.stop_target_waypoint = self.TRAFFIC_STOP_WAYPOINTS[0]

            self.traffic_light_state = 'PREPARING_STOP'
            self.original_target_velocity = self.target_velocity

            distance_to_stop = self.stop_target_waypoint - self.current_waypoint_index
            self.get_logger().info(f"waypoint {self.stop_target_waypoint}에서 정지 예정. 현재 위치: {self.current_waypoint_index}, 남은 거리: {distance_to_stop}")

            if distance_to_stop <= 0:
                # 이미 정지점을 지났으면 즉시 정지
                self.traffic_light_state = 'STOPPED'
                self.get_logger().warn(f"이미 정지 지점({self.stop_target_waypoint})을 지났습니다. 즉시 정지합니다.")

    def handle_go_signal(self):
        """진행 신호 처리"""
        if self.traffic_light_state in ['PREPARING_STOP', 'STOPPED']:
            self.traffic_light_state = 'NORMAL'
            self.gradual_stop_start_waypoint = None
            self.current_stop_target = None  # 정지점 타겟 리셋
            if hasattr(self, 'original_target_velocity'):
                self.target_velocity = self.original_target_velocity
            self.pd_controller.reset()
            self.get_logger().info("정상 주행 재개")

    def handle_prepare_signal(self):
        """준비 신호 처리 (노란불)"""
        self.handle_stop_signal()

    def calculate_traffic_light_velocity(self):
        """신호등 제어에 따른 목표 속도 계산"""
        if self.traffic_light_state == 'NORMAL':
            return self.target_velocity

        elif self.traffic_light_state == 'PREPARING_STOP':
            if self.current_waypoint_index >= self.stop_target_waypoint:
                self.traffic_light_state = 'STOPPED'
                return 0.0

            if self.gradual_stop_start_waypoint is not None:
                remaining_waypoints = self.stop_target_waypoint - self.current_waypoint_index
                total_waypoints = self.stop_target_waypoint - self.gradual_stop_start_waypoint

                if total_waypoints > 0:
                    speed_ratio = remaining_waypoints / total_waypoints
                    target_speed = self.original_target_velocity * max(0.0, speed_ratio)
                    return target_speed
                else:
                    # 감속할 거리가 없으면 즉시 정지
                    return 0.0
            else:
                return self.target_velocity

        elif self.traffic_light_state == 'STOPPED':
            return 0.0

        return self.target_velocity

    def calculate_slope_velocity(self):
        """경사면 하강 시 속도 제어"""
        if self.is_on_slope:
            # 경사면에서는 1.5 m/s로 속도 제한
            return self.slope_target_velocity
        else:
            # 경사면이 아닐 때는 원래 목표 속도 사용
            return self.target_velocity

    def calculate_special_zone_velocity(self):
        """특수 구간 속도 제어"""
        if self.is_approaching_special_zone:
            # 접근 구간 (waypoint 155-184): 점진적 감속
            approach_start = self.special_zone_trigger_waypoint - self.special_zone_approach_distance  # 155
            progress = (self.current_waypoint_index - approach_start) / self.special_zone_approach_distance  # 0~1
            progress = max(0.0, min(1.0, progress))  # 0~1 범위로 제한

            # 원래 속도에서 목표 속도로 선형 감속
            target_velocity = self.special_zone_entry_velocity * (1 - progress) + self.special_zone_target_velocity * progress
            return target_velocity

        elif self.is_in_special_zone:
            # 특수 구간 내부 (waypoint 185-300): 1.5 m/s로 속도 유지
            return self.special_zone_target_velocity
        else:
            # 일반 구간: 원래 목표 속도 사용
            return self.target_velocity

    def calculate_speed_limit_zone_velocity(self):
        """속도 제한 구간 속도 제어"""
        if self.is_in_speed_limit_zone:
            # 속도 제한 구간에서는 1.5 m/s로 속도 제한
            return self.speed_limit_target_velocity
        else:
            # 속도 제한 구간이 아닐 때는 원래 목표 속도 사용
            return self.target_velocity


    # --- 주차 경로 재생 로직 ---
    def start_waypoint_based_parking(self, csv_file):
        """waypoint 기반 자동 주차 시작"""
        self.state = 'PARKING_MANEUVER'
        self.control_timer.cancel()
        self.pd_controller.reset()
        self.get_logger().info(f"waypoint 기반 주차 모드 진입: {csv_file} 실행을 시작합니다.")
        self.post_playback_followup_done = False

        if not os.path.exists(csv_file):
            self.get_logger().error(f"주차 파일 '{csv_file}'을 찾을 수 없습니다! 자율주행으로 복귀합니다.")
            self.finish_parking_maneuver()
            return

        self.get_logger().info(f"주차 경로 파일을 로드합니다: '{csv_file}'")
        if not self._load_csv_into_playback(csv_file):
            self.get_logger().error(f"CSV 파일 '{csv_file}'이 비어있습니다! 주차를 취소합니다.")
            self.finish_parking_maneuver()
            return

        self.playback_index = 0
        self.playback_timer = self.create_timer(0.05, self.playback_loop)

    def start_parking_maneuver(self):
        self.state = 'PARKING_MANEUVER'
        self.control_timer.cancel()
        self.pd_controller.reset()
        self.get_logger().info("주차 모드 진입: PID 제어기를 리셋했습니다.")
        self.post_playback_followup_done = False
        list_of_files = glob.glob('recording_*.csv')
        if not list_of_files:
            self.get_logger().error("주차에 사용할 녹화 파일을 찾을 수 없습니다! 자율주행으로 복귀합니다.")
            self.finish_parking_maneuver()
            return
        latest_file = max(list_of_files, key=os.path.getctime)
        self.get_logger().info(f"주차 경로 파일을 로드합니다: '{latest_file}'")
        if not self._load_csv_into_playback(latest_file):
            self.get_logger().error("CSV 파일이 비어있습니다! 주차를 취소합니다.")
            self.finish_parking_maneuver()
            return
        self.playback_index = 0
        self.playback_timer = self.create_timer(0.05, self.playback_loop)

    def playback_loop(self):
        if self.playback_index >= len(self.playback_data):
            # waypoint 기반 주차인지 확인
            is_waypoint_based = len(self.current_parking_sequence) > 0

            if is_waypoint_based:
                # waypoint 기반 주차 시퀀스 처리
                current_file = self.current_parking_sequence[0] if self.current_parking_sequence else None
                next_file = None

                # 현재 파일에 따른 후속 파일 결정
                if current_file == 't_in.csv' and not self.post_playback_followup_done:
                    if self.driving_control_signal == 5:
                        next_file = 't_right.csv'
                    elif self.driving_control_signal == 6:
                        next_file = 't_left.csv'

                if next_file and os.path.exists(next_file):
                    self.get_logger().info(f"t_in.csv 완료. driving_control={self.driving_control_signal}에 따라 '{next_file}' 연결 재생")
                    if self._load_csv_into_playback(next_file):
                        self.current_parking_sequence = [next_file]  # 시퀀스 업데이트
                        self.post_playback_followup_done = True
                        return  # 다음 루프부터 새 파일 재생
                    else:
                        self.get_logger().error(f"'{next_file}' 로드 실패. 주차를 종료합니다.")

                self.get_logger().info(f"waypoint 기반 주차 시퀀스 완료: {current_file}")
            else:
                # 기존 /parking_command_1 기반 주차 처리
                if not self.post_playback_followup_done:
                    next_file = None

                    if self.driving_control_signal == 5:
                        next_file = 't_right.csv'
                    elif self.driving_control_signal == 6:
                        next_file = 't_left.csv'
                    if next_file and os.path.exists(next_file):
                        self.get_logger().info(f"후속 재생 조건 감지(driving_control={self.driving_control_signal}). '{next_file}' 재생 시작")
                        if self._load_csv_into_playback(next_file):
                            self.post_playback_followup_done = True
                            return  # 다음 루프부터 새 파일 재생
                        else:
                            self.get_logger().error(f"'{next_file}' 로드 실패. 주차를 종료합니다.")
                    else:
                        if next_file:
                            self.get_logger().warn(f"'{next_file}' 파일을 찾을 수 없습니다. 주차를 종료합니다.")

                self.get_logger().info("주차 경로 재생을 완료했습니다.")

            self.finish_parking_maneuver()
            return
        row = self.playback_data[self.playback_index]
        try:
            control_val = float(row[1])
            angular_val = float(row[2])
            control_msg = Float32()
            control_msg.data = control_val
            self.control_pub.publish(control_msg)
            angular_target_msg = Float32()
            angular_target_msg.data = angular_val
            self.angular_target_pub.publish(angular_target_msg)
            self.playback_index += 1
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"주차 데이터 처리 중 오류 발생 (행 {self.playback_index}): {row}. 오류: {e}")
            self.finish_parking_maneuver()

    def finish_parking_maneuver(self):
        if self.playback_timer:
            self.playback_timer.destroy()
            self.playback_timer = None

        # 기본 제어 상태 리셋
        self.pd_controller.reset()
        self.is_ramping_up = False
        self.ramp_up_start_time = None
        self.target_velocity = 0.0
        self.angular_target = 0.0
        self.filtered_control_output = 0.0

        # waypoint 기반 주차 상태 리셋
        self.current_parking_sequence = []
        self.post_playback_followup_done = False

        self.get_logger().info("자율주행 모드로 복귀합니다. PID 제어기와 주차 상태를 리셋했습니다.")
        self.state = 'AUTONOMOUS_DRIVING'
        self.control_timer.reset()

    # --- 자율주행 로직 (변경 없음) ---
    def calculate_angle(self, target_velocity, angular_target):
        if abs(angular_target) < 1e-6:
            return 0.0
        if abs(target_velocity) < 0.1:
            return angular_target * 2.0
        return math.atan2(self.Ld * angular_target, target_velocity) * 5

    def calculate_ramp_up_velocity(self):
        # 램프업 조건: 정지 상태에서 가속할 때
        if abs(self.current_velocity) < 0.1 and self.target_velocity > 0.1:
            if not self.is_ramping_up:
                self.is_ramping_up = True
                self.ramp_up_start_time = time.time()
                self.get_logger().info('속도 램프업 시작!')
            elapsed_time = time.time() - self.ramp_up_start_time
            if elapsed_time < self.ramp_up_duration:
                progress = elapsed_time / self.ramp_up_duration
                return self.target_velocity * progress
            else:
                self.is_ramping_up = False
                return self.target_velocity
        # 램프다운 조건: 현재 속도가 목표 속도보다 0.3 이상 클 때 (모든 감속 상황)
        elif self.current_velocity > self.target_velocity + 0.3:
            # 램프다운 변수 초기화
            if not hasattr(self, 'is_ramping_down'):
                self.is_ramping_down = False
            if not hasattr(self, 'ramp_down_start_time'):
                self.ramp_down_start_time = None
            if not hasattr(self, 'ramp_down_initial_velocity'):
                self.ramp_down_initial_velocity = 0.0

            if not self.is_ramping_down:
                self.is_ramping_down = True
                self.ramp_down_start_time = time.time()
                self.ramp_down_initial_velocity = self.current_velocity
                self.get_logger().info(f'속도 람프다운 시작! {self.ramp_down_initial_velocity:.2f} -> {self.target_velocity:.2f}')

            elapsed_time = time.time() - self.ramp_down_start_time
            if elapsed_time < self.ramp_up_duration:  # 같은 지속 시간 사용
                progress = elapsed_time / self.ramp_up_duration
                # 초기 속도에서 목표 속도까지 선형적으로 감소
                return self.ramp_down_initial_velocity * (1 - progress) + self.target_velocity * progress
            else:
                self.is_ramping_down = False
                return self.target_velocity

        else:
            # 일반적인 경우 - 램프 상태 리셋
            self.is_ramping_up = False
            if hasattr(self, 'is_ramping_down'):
                self.is_ramping_down = False
            return self.target_velocity

    def control_loop(self):
        # ================== [수정] 긴급 장애물 정지 로직 (최우선 순위) ==================
        if self.obstacle_stop_active:
            # 1초간 점진적 긴급 정지 로직
            if self.emergency_stop_start_time is not None:
                elapsed_time = time.time() - self.emergency_stop_start_time
                if elapsed_time < self.emergency_stop_duration:
                    # 1초 동안 초기 속도에서 0으로 선형 감속
                    progress = elapsed_time / self.emergency_stop_duration
                    effective_target_velocity = self.emergency_stop_initial_velocity * (1.0 - progress)
                    self.get_logger().debug(f"긴급 정지 진행 중: {effective_target_velocity:.2f} m/s (진행률: {progress*100:.1f}%)")
                else:
                    # 1초 경과 후 완전 정지
                    effective_target_velocity = 0.0
            else:
                # 안전상 즉시 정지 (정상적이지 않은 상황)
                effective_target_velocity = 0.0
        else:
            # 기존의 신호등 및 램프업 로직 수행
            traffic_controlled_velocity = self.calculate_traffic_light_velocity()

            # ================== [추가] 모든 구간 속도 제어 적용 ==================
            slope_controlled_velocity = self.calculate_slope_velocity()
            special_zone_controlled_velocity = self.calculate_special_zone_velocity()
            speed_limit_zone_controlled_velocity = self.calculate_speed_limit_zone_velocity()

            # 신호등, 경사면, 특수 구간, 속도 제한 구간 제어 중 가장 제한적인 속도 선택
            final_controlled_velocity = min(
                traffic_controlled_velocity,
                slope_controlled_velocity,
                special_zone_controlled_velocity,
                speed_limit_zone_controlled_velocity
            )
            # =========================================================================

            original_target = self.target_velocity
            self.target_velocity = final_controlled_velocity
            effective_target_velocity = self.calculate_ramp_up_velocity()
            self.target_velocity = original_target
        # =========================================================================

        # --- PD 제어 및 퍼블리시 로직 ---
        control_signal = self.pd_controller.compute(
            setpoint=effective_target_velocity,
            measured_value=self.current_velocity
        )
        raw_control_output = max(0.0, min(100.0, control_signal * 20.0))
        self.filtered_control_output = (self.output_filter_alpha * raw_control_output +
                                        (1 - self.output_filter_alpha) * self.filtered_control_output)
        control_output = self.filtered_control_output

        # ================== [추가] 특수 구간 control_signal 3초간 점진적 유지 ==================
        if self.special_zone_control_active and self.special_zone_control_start_time is not None:
            elapsed_time = time.time() - self.special_zone_control_start_time
            if elapsed_time < self.special_zone_control_duration:
                # 점진적으로 control_signal을 목표값으로 변경 (부드러운 전환)
                transition_duration = 0.5  # 0.5초에 걸쳐 점진적 전환
                if elapsed_time < transition_duration:
                    # 0.5초간 현재값에서 목표값으로 점진적 변화
                    progress = elapsed_time / transition_duration
                    if not hasattr(self, 'special_zone_initial_control_output'):
                        self.special_zone_initial_control_output = control_output

                    target_control = self.special_zone_control_signal
                    control_output = (self.special_zone_initial_control_output * (1 - progress) +
                                    target_control * progress)
                    self.get_logger().debug(f"특수 구간 control_signal 점진적 전환: {control_output:.1f} (진행률: {progress*100:.1f}%)")
                else:
                    # 0.5초 후부터는 목표값 유지
                    control_output = self.special_zone_control_signal
                    self.get_logger().debug(f"특수 구간 control_signal 유지: {control_output:.1f}")
            else:
                # 3초 경과 후 특수 구간 control_signal 제어 비활성화
                self.special_zone_control_active = False
                if hasattr(self, 'special_zone_initial_control_output'):
                    delattr(self, 'special_zone_initial_control_output')  # 초기값 정리
                self.get_logger().info("특수 구간 3초 control_signal 유지 완료. 정상 제어로 전환합니다.")
        # =====================================================================================
        control_msg = Float32()
        angular_target_msg = Float32()

        # effective_target_velocity를 기준으로 각도 계산
        angular_target_msg.data = self.calculate_angle(effective_target_velocity, self.angular_target)
        self.angular_target_pub.publish(angular_target_msg)

        control_msg.data = control_output
        self.control_pub.publish(control_msg)

        if self.traffic_light_state != 'NORMAL':
            self.get_logger().debug(f"신호등 제어: 상태={self.traffic_light_state}, "
                                  f"waypoint={self.current_waypoint_index}, "
                                  f"목표속도={effective_target_velocity:.2f}, "
                                  f"제어출력={control_output:.2f}")

        if self.is_on_slope:
            self.get_logger().debug(f"경사면 주행: waypoint={self.current_waypoint_index}, "
                                  f"목표속도={effective_target_velocity:.2f}, "
                                  f"제어출력={control_output:.2f}")

        if self.is_approaching_special_zone:
            self.get_logger().debug(f"특수 구간 접근 중: waypoint={self.current_waypoint_index}, "
                                  f"목표속도={effective_target_velocity:.2f}, "
                                  f"제어출력={control_output:.2f}")

        if self.is_in_special_zone:
            self.get_logger().debug(f"특수 구간 주행: waypoint={self.current_waypoint_index}, "
                                  f"목표속도={effective_target_velocity:.2f}, "
                                  f"제어출력={control_output:.2f}")

        if self.is_in_speed_limit_zone:
            self.get_logger().debug(f"속도 제한 구간 주행: waypoint={self.current_waypoint_index}, "
                                  f"목표속도={effective_target_velocity:.2f}, "
                                  f"제어출력={control_output:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""정지선 185 pwm 1
경사면  300
ㄱ자 끝 567
1번 신호등 646
S 1048
2번 신호등 1200 ->점검 필요
T전 1276"""
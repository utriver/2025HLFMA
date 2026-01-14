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
#                                       (1 - self.derivative_filter_alpha) * self.filtered_derivative)
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

#         # --- 신호등 제어 상태 초기값 ---
#         self.traffic_light_state = 'NORMAL'  # 'NORMAL' | 'PREPARING_STOP' | 'STOPPED'
#         self.driving_control_signal = 1  # 0: stop, 1: go, 2: prepare
#         self.current_waypoint_index = 0
#         self.stop_target_waypoint = 0
#         self.gradual_stop_start_waypoint = None
#         self.original_target_velocity = 0.0

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
#         # 주차 명령을 받기 위한 새로운 Subscriber
#         self.parking_sub = self.create_subscription(
#             Bool,
#             '/parking_command_1',
#             self.parking_command_callback,
#             10
#         )
        
#         # 신호등 제어를 위한 Subscribers
#         self.driving_control_sub = self.create_subscription(
#             Int32,
#             'driving_control',
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
        
#         # 자율주행 제어 루프 타이머 (100Hz)
#         self.control_timer = self.create_timer(0.05, self.control_loop)
        
#         self.get_logger().info('PD Controller Node가 시작되었습니다. 현재 상태: AUTONOMOUS_DRIVING')
#         self.get_logger().info(f'PD 게인 - Kp: {self.pd_controller.kp}, Kd: {self.pd_controller.kd}')
#         self.get_logger().info('신호등 제어 시스템이 활성화되었습니다. (driving_control, waypoint_index 구독 중)')
    
#     # --- 콜백 함수들 ---
#     def velocity_callback(self, msg: TwistWithCovarianceStamped):
#         linear_velocity = msg.twist.twist.linear
#         self.current_velocity = (linear_velocity.x**2 + linear_velocity.y**2)**0.5
        
#     def cmd_vel_callback(self, msg: Twist):
#         # 자율주행 상태가 아니면 /cmd_vel 메시지를 무시
#         if self.state != 'AUTONOMOUS_DRIVING':
#             return
        
#         # 목표 속도가 변경되면 PID 제어기 리셋
#         if abs(self.target_velocity - msg.linear.x) > 0.1:
#             self.pd_controller.reset()
#             self.get_logger().info(f"목표 속도 변경: {self.target_velocity:.2f} -> {msg.linear.x:.2f} m/s, PID 리셋")
        
#         self.target_velocity = msg.linear.x
#         self.angular_target = msg.angular.z
        
#         # 정상 주행 상태일 때만 원래 목표 속도 업데이트
#         if self.traffic_light_state == 'NORMAL':
#             self.original_target_velocity = msg.linear.x
    
#     def parking_command_callback(self, msg: Bool):
#         # True 신호를 받고, 현재 자율주행 상태일 때만 주차 모드로 전환
#         if msg.data and self.state == 'AUTONOMOUS_DRIVING':
#             self.get_logger().info("주차 명령을 수신했습니다. 경로 재생을 시작합니다...")
#             self.start_parking_maneuver()
    
#     def driving_control_callback(self, msg: Int32):
#         """신호등 제어 신호 콜백"""
#         previous_signal = self.driving_control_signal
#         self.driving_control_signal = msg.data
        
#         # 신호 변화 감지 및 로깅
#         if previous_signal != self.driving_control_signal:
#             if self.driving_control_signal == 0:
#                 self.get_logger().info(f"신호등 정지 신호 수신 (waypoint: {self.current_waypoint_index})")
#                 self.handle_stop_signal()
#             elif self.driving_control_signal == 1:
#                 self.get_logger().info("신호등 진행 신호 수신 - 정상 주행 재개")
#                 self.handle_go_signal()
#             elif self.driving_control_signal == 2:
#                 self.get_logger().info("신호등 준비 신호 수신")
#                 self.handle_prepare_signal()
    
#     def waypoint_index_callback(self, msg: Int32):
#         """현재 waypoint index 콜백"""
#         self.current_waypoint_index = msg.data
    
#     # --- 신호등 제어 처리 함수들 ---
#     def handle_stop_signal(self):
#         """정지 신호 처리"""
#         if self.traffic_light_state == 'NORMAL':
#             # 현재 waypoint에서 120개 전까지 점진적 정차 계산
#             self.gradual_stop_start_waypoint = max(0, self.current_waypoint_index)
#             self.stop_target_waypoint = self.current_waypoint_index + 120
#             self.traffic_light_state = 'PREPARING_STOP'
#             self.original_target_velocity = self.target_velocity
#             self.get_logger().info(f"점진적 정차 시작: waypoint {self.gradual_stop_start_waypoint}에서 {self.stop_target_waypoint}까지")
    
#     def handle_go_signal(self):
#         """진행 신호 처리"""
#         if self.traffic_light_state in ['PREPARING_STOP', 'STOPPED']:
#             self.traffic_light_state = 'NORMAL'
#             self.gradual_stop_start_waypoint = None
#             # 원래 목표 속도로 복원
#             if hasattr(self, 'original_target_velocity'):
#                 self.target_velocity = self.original_target_velocity
#             self.pd_controller.reset()  # PID 제어기 리셋으로 부드러운 재가속
#             self.get_logger().info("정상 주행 재개")
    
#     def handle_prepare_signal(self):
#         """준비 신호 처리 (노란불)"""
#         # 현재는 정지 신호와 동일하게 처리
#         self.handle_stop_signal()
    
#     def calculate_traffic_light_velocity(self):
#         """신호등 제어에 따른 목표 속도 계산"""
#         if self.traffic_light_state == 'NORMAL':
#             return self.target_velocity
        
#         elif self.traffic_light_state == 'PREPARING_STOP':
#             # waypoint 120에 도달했거나 지났으면 완전 정지
#             if self.current_waypoint_index >= self.stop_target_waypoint:
#                 self.traffic_light_state = 'STOPPED'
#                 return 0.0
            
#             # 점진적 감속 계산
#             if self.gradual_stop_start_waypoint is not None:
#                 remaining_waypoints = self.stop_target_waypoint - self.current_waypoint_index
#                 total_waypoints = self.stop_target_waypoint - self.gradual_stop_start_waypoint
                
#                 if total_waypoints > 0:
#                     # 선형적으로 속도 감소
#                     speed_ratio = remaining_waypoints / total_waypoints
#                     target_speed = self.original_target_velocity * max(0.0, speed_ratio)
#                     self.get_logger().debug(f"감속 중: waypoint {self.current_waypoint_index}, 속도비율: {speed_ratio:.2f}, 목표속도: {target_speed:.2f}")
#                     return target_speed
#                 else:
#                     return 0.0
#             else:
#                 return 0.0
        
#         elif self.traffic_light_state == 'STOPPED':
#             return 0.0
        
#         return self.target_velocity

#     # --- 주차 경로 재생 로직 ---
#     def start_parking_maneuver(self):
#         self.state = 'PARKING_MANEUVER'
#         self.control_timer.cancel() # 자율주행 제어 루프를 일시정지
        
#         # 주차 시작 시 PID 제어기 리셋 (안전을 위해)
#         self.pd_controller.reset()
#         self.get_logger().info("주차 모드 진입: PID 제어기를 리셋했습니다.")

#         # 가장 최근에 녹화된 CSV 파일 찾기
#         list_of_files = glob.glob('recording_*.csv')
#         if not list_of_files:
#             self.get_logger().error("주차에 사용할 녹화 파일을 찾을 수 없습니다! 자율주행으로 복귀합니다.")
#             self.finish_parking_maneuver()
#             return

#         latest_file = max(list_of_files, key=os.path.getctime)
#         self.get_logger().info(f"주차 경로 파일을 로드합니다: '{latest_file}'")

#         try:
#             with open(latest_file, 'r') as csvfile:
#                 reader = csv.reader(csvfile)
#                 next(reader)  # 헤더 스킵
#                 self.playback_data = [row for row in reader]
#         except Exception as e:
#             self.get_logger().error(f"CSV 파일을 읽는 데 실패했습니다: {e}")
#             self.finish_parking_maneuver()
#             return

#         if not self.playback_data:
#             self.get_logger().error("CSV 파일이 비어있습니다! 주차를 취소합니다.")
#             self.finish_parking_maneuver()
#             return

#         # 경로 재생 루프 시작
#         self.playback_index = 0
#         self.playback_timer = self.create_timer(0.05, self.playback_loop) # 100Hz

#     def playback_loop(self):
#         # 모든 데이터를 재생했으면 주차 종료
#         if self.playback_index >= len(self.playback_data):
#             self.get_logger().info("주차 경로 재생을 완료했습니다.")
#             self.finish_parking_maneuver()
#             return

#         row = self.playback_data[self.playback_index]
#         try:
#             # CSV 파일 형식: timestamp_ns, control_signal, angular_target
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

#         # PID 제어기 완전 리셋 (적분항 누적 문제 해결)
#         self.pd_controller.reset()
        
#         # 램프업 상태 리셋
#         self.is_ramping_up = False
#         self.ramp_up_start_time = None
        
#         # 목표 속도 초기화 (안전을 위해)
#         self.target_velocity = 0.0
#         self.angular_target = 0.0
        
#         # 출력 필터 리셋
#         self.filtered_control_output = 0.0
        
#         self.get_logger().info("자율주행 모드로 복귀합니다. PID 제어기와 램프업 상태를 리셋했습니다.")
#         self.state = 'AUTONOMOUS_DRIVING'
#         self.control_timer.reset() # 자율주행 제어 루프를 다시 시작

#     # --- 자율주행 로직 ---
#     def calculate_angle(self, target_velocity, angular_target):
#         if abs(angular_target) < 1e-6:
#             return 0.0
#         # target_velocity가 0에 가까울 때 발산하는 것을 방지
#         if abs(target_velocity) < 0.1:
#             # 저속에서는 각속도를 직접 반영 (예: 제자리 회전)
#             return angular_target * 2.0 # 값은 튜닝 필요
#         return math.atan2(self.Ld * angular_target, target_velocity) * 5

#     def calculate_ramp_up_velocity(self):
#         # 현재 속도가 0에 가깝고 목표 속도가 있을 때 램프업 시작
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
#         else:
#             self.is_ramping_up = False
#             return self.target_velocity

#     def control_loop(self):
#         """자율주행 제어 루프 - 100Hz로 실행"""
#         # 신호등 제어에 따른 목표 속도 조정
#         traffic_controlled_velocity = self.calculate_traffic_light_velocity()
        
#         # 램프업 로직 (신호등 제어된 속도 기준)
#         # 임시로 target_velocity를 교체하여 기존 램프업 로직 활용
#         original_target = self.target_velocity
#         self.target_velocity = traffic_controlled_velocity
#         effective_target_velocity = self.calculate_ramp_up_velocity()
#         self.target_velocity = original_target  # 복원
        
#         # PD 제어 신호 계산
#         control_signal = self.pd_controller.compute(
#             setpoint=effective_target_velocity,
#             measured_value=self.current_velocity
#         )
        
#         # 제어 신호를 0~100 범위로 클램핑 (더 안전한 스케일링 팩터)
#         raw_control_output = max(0.0, min(100.0, control_signal * 20.0))
        
#         # 출력 스무딩 필터 적용 (급격한 변화 방지)
#         self.filtered_control_output = (self.output_filter_alpha * raw_control_output + 
#                                        (1 - self.output_filter_alpha) * self.filtered_control_output)
#         control_output = self.filtered_control_output
        
#         # 제어 신호 발행
#         control_msg = Float32()
#         angular_target_msg = Float32()
#         angular_target_msg.data = self.calculate_angle(traffic_controlled_velocity, self.angular_target)
#         self.angular_target_pub.publish(angular_target_msg)
#         control_msg.data = control_output
#         self.control_pub.publish(control_msg)
        
#         # 디버그 로그 (신호등 제어 활성화 시에만)
#         if self.traffic_light_state != 'NORMAL':
#             self.get_logger().debug(f"신호등 제어: 상태={self.traffic_light_state}, "
#                                   f"waypoint={self.current_waypoint_index}, "
#                                   f"목표속도={traffic_controlled_velocity:.2f}, "
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




# """

# ### **주요 변경 및 추가 사항**

# 1.  **상태 변수 추가 (`self.state`)**: 노드의 현재 상태를 `'AUTONOMOUS_DRIVING'` 또는 `'PARKING_MANEUVER'`로 관리합니다.
# 2.  **주차 명령 Subscriber 추가**: `/parking_command_1` 토픽을 `Bool` 타입으로 구독하여 `True` 메시지를 받았을 때 주차 로직을 실행하는 `parking_command_callback` 함수를 추가했습니다.
# 3.  **주차 로직 함수 추가**:
#     * `start_parking_maneuver()`: 주차 모드로 전환하고, 최신 CSV 파일을 찾아 읽어온 후, 100Hz 타이머(`playback_timer`)를 시작합니다.
#     * `playback_loop()`: `playback_timer`에 의해 100Hz로 호출되며, CSV 파일의 데이터를 한 줄씩 읽어 `/control_signal`과 `angular_target` 토픽으로 발행합니다.
#     * `finish_parking_maneuver()`: 경로 재생이 끝나면 `playback_timer`를 종료하고, 상태를 다시 `'AUTONOMOUS_DRIVING'`으로 변경한 뒤, 기존의 `control_timer`를 재시작하여 자율주행 모드로 복귀합니다.
# 4.  **기존 콜백 수정**: `cmd_vel_callback` 함수 시작 부분에 `if self.state != 'AUTONOMOUS_DRIVING': return` 구문을 추가하여, 주차 중에는 `/cmd_vel` 명령을 무시하도록 방어 코드를 넣었습니다.

# ### **사용 방법**

# 1.  평소처럼 `pd_controller.py` 노드를 실행하면 자율주행 모드로 시작합니다.
# 2.  터미널에서 아래 명령어를 입력하여 `/parking_command_1` 토픽에 `True` 신호를 보내면, 노드가 자동으로 가장 최근에 생성된 `recording_*.csv` 파일을 찾아 주차 경로 재생을 시작합니다.
#     ```bash
#     ros2 topic pub /parking_command_1 std_msgs/msg/Bool "{data: true}" --once
    
# """
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

        # --- 신호등 제어 상태 초기값 ---
        self.traffic_light_state = 'NORMAL'  # 'NORMAL' | 'PREPARING_STOP' | 'STOPPED'
        self.driving_control_signal = 1  # 0: stop, 1: go, 2: prepare
        self.current_waypoint_index = 0
        self.stop_target_waypoint = 0
        self.gradual_stop_start_waypoint = None
        self.original_target_velocity = 0.0
        
        # *** 고정 정지 웨이포인트 설정 ***
        self.FIXED_STOP_WAYPOINT = 400
        self.APPROACH_DISTANCE = 50  # waypoint 200에 가까워지는 거리 (150부터 감지)
        
        # *** driving_control 지속 체크를 위한 변수 ***
        self.driving_control_history = []  # 최근 신호 이력
        self.SIGNAL_CHECK_DURATION = 2.0  # 2초간 지속적으로 0이 와야 정지
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
            'driving_control',
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
        self.get_logger().info(f'고정 정지 지점: waypoint {self.FIXED_STOP_WAYPOINT}')
        self.get_logger().info(f'정지 감지 범위: waypoint {self.FIXED_STOP_WAYPOINT - self.APPROACH_DISTANCE} ~ {self.FIXED_STOP_WAYPOINT + 10}')
        self.get_logger().info(f'PD 게인 - Kp: {self.pd_controller.kp}, Kd: {self.pd_controller.kd}')
        self.get_logger().info('신호등 제어: waypoint 200 근처에서 지속적인 driving_control=0 신호 시 정지')
        
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
        
        if self.traffic_light_state == 'NORMAL':
            self.original_target_velocity = msg.linear.x
    
    def parking_command_callback(self, msg: Bool):
        if msg.data and self.state == 'AUTONOMOUS_DRIVING':
            self.get_logger().info("주차 명령을 수신했습니다. 경로 재생을 시작합니다...")
            self.start_parking_maneuver()
    
    def driving_control_callback(self, msg: Int32):
        current_time = time.time()
        self.driving_control_signal = msg.data
        
        # 신호 이력에 추가 (시간, 신호값)
        self.driving_control_history.append((current_time, msg.data))
        
        # 오래된 이력 제거 (SIGNAL_CHECK_DURATION 이전 것들)
        self.driving_control_history = [
            (t, signal) for t, signal in self.driving_control_history 
            if current_time - t <= self.SIGNAL_CHECK_DURATION
        ]
        
        # waypoint 200 근처에서만 정지 로직 적용
        is_near_stop_point = (self.FIXED_STOP_WAYPOINT - self.APPROACH_DISTANCE) <= self.current_waypoint_index <= (self.FIXED_STOP_WAYPOINT + 10)
        
        if is_near_stop_point:
            # 지속적으로 0이 들어오는지 확인
            if self.is_continuous_stop_signal():
                if self.traffic_light_state == 'NORMAL':
                    self.get_logger().info(f"waypoint 200 근처에서 지속적인 정지 신호 감지 (현재: {self.current_waypoint_index})")
                    self.handle_stop_signal()
            elif msg.data == 1 and self.traffic_light_state in ['PREPARING_STOP', 'STOPPED']:
                self.get_logger().info("진행 신호 수신 - 정상 주행 재개")
                self.handle_go_signal()
        else:
            # waypoint 200 근처가 아닐 때는 즉시 진행 신호만 처리
            if msg.data == 1 and self.traffic_light_state in ['PREPARING_STOP', 'STOPPED']:
                self.get_logger().info("진행 신호 수신 - 정상 주행 재개")
                self.handle_go_signal()
    
    def is_continuous_stop_signal(self):
        """지속적으로 정지 신호(0)가 들어오는지 확인"""
        if len(self.driving_control_history) < 5:  # 최소 5개의 신호가 필요
            return False
        
        # 최근 신호들이 모두 0인지 확인
        recent_signals = [signal for _, signal in self.driving_control_history[-10:]]  # 최근 10개
        return all(signal == 0 for signal in recent_signals)
    
    def waypoint_index_callback(self, msg: Int32):
        self.current_waypoint_index = msg.data
    
    # --- 신호등 제어 처리 함수들 ---
    def handle_stop_signal(self):
        """정지 신호 처리. waypoint 200에서 정지."""
        if self.traffic_light_state == 'NORMAL':
            self.gradual_stop_start_waypoint = self.current_waypoint_index
            self.stop_target_waypoint = self.FIXED_STOP_WAYPOINT  # 항상 200에서 정지
            self.traffic_light_state = 'PREPARING_STOP'
            self.original_target_velocity = self.target_velocity
            
            distance_to_stop = self.stop_target_waypoint - self.current_waypoint_index
            self.get_logger().info(f"waypoint 200에서 정지 예정. 현재 위치: {self.current_waypoint_index}, 남은 거리: {distance_to_stop}")
            
            if distance_to_stop <= 0:
                # 이미 200을 지났으면 즉시 정지
                self.traffic_light_state = 'STOPPED'
                self.get_logger().warn(f"이미 정지 지점({self.FIXED_STOP_WAYPOINT})을 지났습니다. 즉시 정지합니다.")

    def handle_go_signal(self):
        """진행 신호 처리"""
        if self.traffic_light_state in ['PREPARING_STOP', 'STOPPED']:
            self.traffic_light_state = 'NORMAL'
            self.gradual_stop_start_waypoint = None
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

    # --- 주차 경로 재생 로직 (변경 없음) ---
    def start_parking_maneuver(self):
        self.state = 'PARKING_MANEUVER'
        self.control_timer.cancel()
        self.pd_controller.reset()
        self.get_logger().info("주차 모드 진입: PID 제어기를 리셋했습니다.")
        list_of_files = glob.glob('recording_*.csv')
        if not list_of_files:
            self.get_logger().error("주차에 사용할 녹화 파일을 찾을 수 없습니다! 자율주행으로 복귀합니다.")
            self.finish_parking_maneuver()
            return
        latest_file = max(list_of_files, key=os.path.getctime)
        self.get_logger().info(f"주차 경로 파일을 로드합니다: '{latest_file}'")
        try:
            with open(latest_file, 'r') as csvfile:
                reader = csv.reader(csvfile)
                next(reader)
                self.playback_data = [row for row in reader]
        except Exception as e:
            self.get_logger().error(f"CSV 파일을 읽는 데 실패했습니다: {e}")
            self.finish_parking_maneuver()
            return
        if not self.playback_data:
            self.get_logger().error("CSV 파일이 비어있습니다! 주차를 취소합니다.")
            self.finish_parking_maneuver()
            return
        self.playback_index = 0
        self.playback_timer = self.create_timer(0.05, self.playback_loop)

    def playback_loop(self):
        if self.playback_index >= len(self.playback_data):
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
        self.pd_controller.reset()
        self.is_ramping_up = False
        self.ramp_up_start_time = None
        self.target_velocity = 0.0
        self.angular_target = 0.0
        self.filtered_control_output = 0.0
        self.get_logger().info("자율주행 모드로 복귀합니다. PID 제어기와 램프업 상태를 리셋했습니다.")
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
        else:
            self.is_ramping_up = False
            return self.target_velocity

    def control_loop(self):
        traffic_controlled_velocity = self.calculate_traffic_light_velocity()
        original_target = self.target_velocity
        self.target_velocity = traffic_controlled_velocity
        effective_target_velocity = self.calculate_ramp_up_velocity()
        self.target_velocity = original_target
        control_signal = self.pd_controller.compute(
            setpoint=effective_target_velocity,
            measured_value=self.current_velocity
        )
        raw_control_output = max(0.0, min(100.0, control_signal * 20.0))
        self.filtered_control_output = (self.output_filter_alpha * raw_control_output + 
                                       (1 - self.output_filter_alpha) * self.filtered_control_output)
        control_output = self.filtered_control_output
        control_msg = Float32()
        angular_target_msg = Float32()
        angular_target_msg.data = self.calculate_angle(traffic_controlled_velocity, self.angular_target)
        self.angular_target_pub.publish(angular_target_msg)
        control_msg.data = control_output
        self.control_pub.publish(control_msg)
        
        if self.traffic_light_state != 'NORMAL':
            self.get_logger().debug(f"신호등 제어: 상태={self.traffic_light_state}, "
                                  f"waypoint={self.current_waypoint_index}, "
                                  f"목표속도={traffic_controlled_velocity:.2f}, "
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
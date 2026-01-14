
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Float32, Bool, Int32
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2

# class VisionSubscriber(Node):
#     def __init__(self):
#         super().__init__('vision_subscriber')
#         self.get_logger().info('Vision Subscriber node started.')
        
#         self.bridge = CvBridge()
        
#         # Subscriber 설정
#         self.person_detected_sub = self.create_subscription(
#             Bool, '/main_vision/person_detected', self.person_detected_callback, 10)
        
#         self.person_distance_sub = self.create_subscription(
#             Float32, '/main_vision/person_distance', self.person_distance_callback, 10)
        
#         self.traffic_light_sub = self.create_subscription(
#             String, '/main_vision/traffic_light_state', self.traffic_light_callback, 10)
        
#         self.driving_command_sub = self.create_subscription(
#             String, '/main_vision/driving_command', self.driving_command_callback, 10)
        
#         self.processed_image_sub = self.create_subscription(
#             Image, '/main_vision/processed_image', self.image_callback, 10)
        
#         self.driving_control_sub = self.create_subscription(
#             Int32, '/main_vision/driving_control', lambda msg: self.get_logger().info(f'Driving Control: {msg.data}'), 10)
            
#         self.object_position_sub = self.create_subscription(
#             Int32, '/main_vision/object_position', self.object_position_callback, 10)

#         # 상태 변수 초기화
#         self.current_person_status = False
#         self.current_person_distance = float('inf')
#         self.current_traffic_state = "UNKNOWN"
#         self.current_command = "N/A"
        
#         self.get_logger().info('All subscribers initialized. Waiting for data...')

#     # ================== 수정된 부분: 로직 및 로그 메시지 변경 ==================
#     def object_position_callback(self, msg):
#         """콘과 박스 위치 신호를 받아 처리"""
#         # 신호 5: 왼쪽 콘 패턴, 신호 6: 오른쪽 콘 패턴
#         if msg.data == 5:
#             self.execute_cone_left_action()
#         elif msg.data == 6:
#             self.execute_cone_right_action()

#     def execute_cone_right_action(self):
#         """오른쪽 콘 패턴(신호 6) 감지 시 실행할 동작"""
#         self.get_logger().warn('>>> 오른쪽 콘 패턴 감지! (신호 6) <<<')
        
#     def execute_cone_left_action(self):
#         """왼쪽 콘 패턴(신호 5) 감지 시 실행할 동작"""
#         self.get_logger().warn('>>> 왼쪽 콘 패턴 감지! (신호 5) <<<')
#     # =======================================================================

#     def image_callback(self, msg):
#         """처리된 이미지 콜백 및 시각화"""
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#             cv2.imshow('Unified Vision System Output', cv_image)
#             cv2.waitKey(1)
#         except Exception as e:
#             self.get_logger().error(f'Image display failed: {str(e)}')
            
#     def person_detected_callback(self, msg):
#         if msg.data:
#             self.get_logger().warn('🚨 사람 발견!')
#         self.current_person_status = msg.data

#     def person_distance_callback(self, msg):
#         self.get_logger().info(f'사람과의 거리: {msg.data:.1f}m')
#         self.current_person_distance = msg.data

#     def traffic_light_callback(self, msg):
#         self.current_traffic_state = msg.data

#     def driving_command_callback(self, msg):
#         self.current_command = msg.data

# def main(args=None):
#     rclpy.init(args=args)
#     vision_subscriber = VisionSubscriber()
#     try:
#         rclpy.spin(vision_subscriber)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         vision_subscriber.destroy_node()
#         cv2.destroyAllWindows()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
# ~/ros2_ws/src/vision_system/vision_system/vision_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VisionSubscriber(Node):
    def __init__(self):
        super().__init__('vision_subscriber')
        self.get_logger().info('Vision Subscriber node started.')
        
        self.bridge = CvBridge()
        
        # Subscriber 설정
        # 토픽 이름은 publisher와 일치해야 합니다. (예: /driving_control)
        self.driving_control_sub = self.create_subscription(
            Int32, 'driving_control', self.driving_control_callback, 10)
        
        self.command_sub = self.create_subscription(
            String, 'driving_command', lambda msg: self.get_logger().info(f'Command String: {msg.data}'), 10)

        self.processed_image_sub = self.create_subscription(
            Image, 'processed_image', self.image_callback, 10)

        self.get_logger().info('Subscriber initialized. Waiting for data...')

    def driving_control_callback(self, msg):
        """수신된 제어 신호에 따라 로깅 또는 동작을 수행합니다."""
        signal = msg.data
        if signal == 0:
            self.get_logger().warn('🛑 제어 신호: 0 - 정지 명령 수신 (빨간불 또는 사람)')
            # self.execute_stop()
        elif signal == 1:
            self.get_logger().info('🟢 제어 신호: 1 - 진행 명령 수신 (초록불 또는 좌회전)')
            # self.execute_proceed()
        elif signal == 2:
            self.get_logger().warn('🟡 제어 신호: 2 - 정지 준비 명령 수신 (주황불)')
            # self.execute_prepare_stop()
        elif signal == 3:
            # <<<<<<<<<<<<<<<< [추가] 차량 감지 신호 처리
            self.get_logger().warn('🟠 제어 신호: 3 - 차량 감지! 주의/정지 명령 수신')
            # self.execute_car_detected_action()
        elif signal == 5:
            self.get_logger().info('🔷 제어 신호: 5 - 콘이 박스 왼쪽에 있음')
            # self.execute_cone_left_action()
        elif signal == 6:
            self.get_logger().info('🔶 제어 신호: 6 - 콘이 박스 오른쪽에 있음')
            # self.execute_cone_right_action()
        else:
            self.get_logger().error(f'❓ 알 수 없는 제어 신호 수신: {signal}')

    def image_callback(self, msg):
        """처리된 이미지 콜백 및 시각화"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Unified Vision System Output', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Image display failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    vision_subscriber = VisionSubscriber()
    try:
        rclpy.spin(vision_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        vision_subscriber.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ObjectSubscriber(Node):
    def __init__(self):
        super().__init__('object_subscriber')
        self.get_logger().info('객체 위치 구독자 시작됨')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 🚀 객체 위치 구독 (토픽 이름 변경: cone_position -> object_position)
        self.position_sub = self.create_subscription(
            Int32, 'object_position', self.position_callback, 10)
        
        # 이미지 구독
        self.processed_image_sub = self.create_subscription(
            Image, 'processed_image', self.image_callback, 10)
        
        # 현재 상태
        self.current_position = 0

    def position_callback(self, msg):
        """객체 위치 콜백"""
        self.current_position = msg.data
        
        # 🚀 로그 메시지 수정 (꼬깔콘, 박스 -> 의자, 스틱)
        if msg.data == 5:
            self.get_logger().info('🔶 의자가 스틱 오른쪽에 있음 (신호: 5)')
            self.execute_object_right_action()
        elif msg.data == 6:
            self.get_logger().info('🔷 의자가 스틱 왼쪽에 있음 (신호: 6)')  
            self.execute_object_left_action()
        elif msg.data == 0:
            self.get_logger().info('⚪ 의자/스틱 패턴 감지 안됨 (신호: 0)')
        else:
            self.get_logger().warn(f'❓ 알 수 없는 신호: {msg.data}')

    # 🚀 함수 이름 및 로그 메시지 수정
    def execute_object_right_action(self):
        """오른쪽 객체(의자) 감지 시 실행할 동작"""
        self.get_logger().info('>>> 오른쪽 의자 패턴 동작 실행 <<<')
        # 여기에 실제 제어 코드 추가
        # 예: self.motor_controller.turn_left()  # 오른쪽 의자를 피해 왼쪽으로
        
    def execute_object_left_action(self):
        """왼쪽 객체(의자) 감지 시 실행할 동작"""
        self.get_logger().info('>>> 왼쪽 의자 패턴 동작 실행 <<<')
        # 여기에 실제 제어 코드 추가  
        # 예: self.motor_controller.turn_right()  # 왼쪽 의자를 피해 오른쪽으로

    def image_callback(self, msg):
        """이미지 표시"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # 🚀 창 이름 변경
            cv2.imshow('Object Detection Result', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'이미지 표시 실패: {str(e)}')

    def get_current_status(self):
        """현재 상태 반환"""
        return {
            'position_signal': self.current_position,
            'object_side': 'right' if self.current_position == 5 else 'left' if self.current_position == 6 else 'none'
        }

def main(args=None):
    rclpy.init(args=args)
    
    try:
        subscriber = ObjectSubscriber()
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            subscriber.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
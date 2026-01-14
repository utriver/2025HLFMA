#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
from ultralytics import YOLO  # 🚀 YOLOv8 라이브러리 추가

class VisionPublisher(Node):
    def __init__(self):
        super().__init__('vision_test_publisher_node')
        self.get_logger().info('Vision Publisher node started with YOLOv8.')
        
        # 🚀 YOLOv8 모델 로드 ('best.pt' 파일이 스크립트와 같은 경로에 있어야 합니다)
        try:
            self.model = YOLO('best.pt')
            self.get_logger().info('YOLOv8 model "best.pt" loaded successfully.')
            # 모델 클래스 이름 확인 (의자: chair, 스틱: stick)
            self.class_names = self.model.names
            self.get_logger().info(f"Model classes: {self.class_names}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            # 모델 로드 실패 시 노드 종료
            rclpy.shutdown()
            return

        # 카메라 시리얼 파라미터 선언
        self.declare_parameter('camera_serial', '313522303259')
        self.camera_serial = self.get_parameter('camera_serial').get_parameter_value().string_value
        self.get_logger().info(f'Object detection camera serial: {self.camera_serial}')

        # 🚀 발행할 토픽 이름 변경 (의자 -> object)
        self.position_publisher = self.create_publisher(Int32, 'object_position', 10)
        self.processed_image_publisher = self.create_publisher(Image, 'processed_image', 10)
        
        self.bridge = CvBridge()
        self.init_realsense_camera()
        self.timer = self.create_timer(1/30.0, self.timer_callback)

    def init_realsense_camera(self):
        # (기존 RealSense 초기화 코드는 변경 없음)
        try:
            self.get_logger().info('RealSense 카메라 초기화 중...')
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                raise Exception("RealSense 카메라를 찾을 수 없습니다")
            target_serial = self.camera_serial
            selected_device = None
            for device in devices:
                serial = device.get_info(rs.camera_info.serial_number)
                self.get_logger().info(f'발견된 RealSense 카메라: {serial}')
                if serial == target_serial:
                    selected_device = device
                    self.get_logger().info(f'타겟 카메라 선택됨: {serial}')
                    break
            if selected_device is None:
                selected_device = devices[0]
                serial = selected_device.get_info(rs.camera_info.serial_number)
                self.get_logger().warn(f'타겟 카메라를 찾을 수 없어 첫 번째 카메라 사용: {serial}')
            self.pipeline = rs.pipeline()
            config = rs.config()
            selected_serial = selected_device.get_info(rs.camera_info.serial_number)
            config.enable_device(selected_serial)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            self.get_logger().info(f'RealSense 카메라 초기화 성공 - Serial: {selected_serial}')
            self.use_realsense = True
        except Exception as e:
            self.get_logger().error(f'RealSense 초기화 실패: {str(e)}')
            self.use_realsense = False
            self.pipeline = None

    def capture_frame(self):
        # (기존 프레임 캡처 코드는 변경 없음)
        if not self.use_realsense or self.pipeline is None:
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(dummy, 'NO CAMERA', (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            return dummy
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            if color_frame:
                return np.asanyarray(color_frame.get_data())
            return None
        except Exception as e:
            return None

    def timer_callback(self):
        try:
            color_image = self.capture_frame()
            if color_image is None:
                return

            # 🚀 YOLOv8 객체 탐지 로직 호출
            position_signal, stick_box, chair_boxes = self.detect_objects_yolo(color_image)
            
            msg = Int32()
            msg.data = position_signal
            self.position_publisher.publish(msg)
            
            if position_signal == 5:
                self.get_logger().info('의자가 스틱 오른쪽에 있음 (5)')
            elif position_signal == 6:
                self.get_logger().info('의자가 스틱 왼쪽에 있음 (6)')
            else:
                self.get_logger().info('의자/스틱 패턴 감지 안됨 (0)')
            
            # 🚀 시각화 함수 호출
            display_image = self.draw_yolo_detections(color_image, position_signal, stick_box, chair_boxes)
            
            img_msg = self.bridge.cv2_to_imgmsg(display_image, "bgr8")
            self.processed_image_publisher.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f'타이머 콜백 에러: {str(e)}')

    def detect_objects_yolo(self, image):
        """YOLOv8 모델을 사용하여 객체(의자, 스틱)를 탐지하고 위치 관계를 판단합니다."""
        results = self.model(image, verbose=False) # verbose=False로 로그 출력 억제
        
        stick_box = None
        chair_boxes = []

        # 탐지 결과에서 'stick'과 'chair'의 바운딩 박스를 찾습니다.
        for box in results[0].boxes:
            # 신뢰도가 0.5 이상인 경우만 사용
            if box.conf[0] > 0.5:
                class_id = int(box.cls[0])
                class_name = self.class_names[class_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                if class_name == 'stick':
                    # 가장 신뢰도 높은 스틱 하나만 사용
                    if stick_box is None or box.conf[0] > stick_box[4]:
                        stick_box = (x1, y1, x2 - x1, y2 - y1, box.conf[0])
                elif class_name == 'chair':
                    chair_boxes.append((x1, y1, x2 - x1, y2 - y1, box.conf[0]))

        # 스틱과 의자가 모두 1개 이상 감지되었는지 확인
        if stick_box is None or not chair_boxes:
            return 0, stick_box, chair_boxes

        # 스틱의 중심 x좌표 계산
        stick_center_x = stick_box[0] + stick_box[2] / 2
        
        # 모든 의자의 평균 중심 x좌표 계산
        chair_centers_x = [box[0] + box[2] / 2 for box in chair_boxes]
        avg_chair_x = sum(chair_centers_x) / len(chair_boxes)
        
        self.get_logger().info(f'스틱 중심: {stick_center_x:.1f}, 의자 평균 중심: {avg_chair_x:.1f}')
        
        # 위치 관계에 따라 신호 결정
        if avg_chair_x > stick_center_x:
            return 5, stick_box, chair_boxes  # 의자가 스틱의 오른쪽에 있음
        else:
            return 6, stick_box, chair_boxes  # 의자가 스틱의 왼쪽에 있음

    def draw_yolo_detections(self, image, position_signal, stick_box, chair_boxes):
        """YOLOv8 탐지 결과를 시각화합니다."""
        result_image = image.copy()
        
        # 스틱 바운딩 박스 그리기 (노란색)
        if stick_box:
            x, y, w, h, conf = stick_box
            cv2.rectangle(result_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(result_image, f"Stick {conf:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            center_x = int(x + w / 2)
            cv2.line(result_image, (center_x, 0), (center_x, result_image.shape[0]), (0, 255, 255), 1)

        # 의자 바운딩 박스 그리기 (파란색)
        for i, box in enumerate(chair_boxes):
            x, y, w, h, conf = box
            cv2.rectangle(result_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(result_image, f"Chair {conf:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        # 의자 평균 위치선 그리기
        if chair_boxes:
            chair_centers_x = [box[0] + box[2]/2 for box in chair_boxes]
            avg_x = int(sum(chair_centers_x) / len(chair_boxes))
            cv2.line(result_image, (avg_x, 0), (avg_x, result_image.shape[0]), (255, 100, 100), 2, cv2.LINE_AA)

        # 결과 텍스트 표시
        if position_signal == 5:
            text = "CHAIRS ON RIGHT (5)"
            color = (0, 255, 0)
        elif position_signal == 6:
            text = "CHAIRS ON LEFT (6)"
            color = (0, 0, 255)
        else:
            text = "NO PATTERN (0)"
            color = (128, 128, 128)
        
        cv2.putText(result_image, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
        
        info_text = f"Stick: {'Yes' if stick_box else 'No'}, Chairs: {len(chair_boxes)}"
        cv2.putText(result_image, info_text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        return result_image


def main(args=None):
    rclpy.init(args=args)
    
    try:
        vision_publisher = VisionPublisher()
        if vision_publisher.model:  # 모델 로드가 성공했을 때만 spin
            rclpy.spin(vision_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'vision_publisher' in locals() and rclpy.ok():
            vision_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
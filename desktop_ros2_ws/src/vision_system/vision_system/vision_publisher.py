
# import cv2
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Float32, Bool, Int32
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from ultralytics import YOLO
# import pyrealsense2 as rs
# import torch

# class UnifiedVisionNode(Node):
#     def __init__(self):
#         super().__init__('unified_vision_node')

#         self.declare_parameter('camera_serial', '313522303259')
#         self.camera_serial = self.get_parameter('camera_serial').get_parameter_value().string_value
#         self.get_logger().info(f'Using single camera with serial: {self.camera_serial}')

#         # Publisher 선언 (driving_control이 핵심 역할을 합니다)
#         self.driving_control_publisher = self.create_publisher(Int32, 'driving_control', 10)
#         self.processed_image_publisher = self.create_publisher(Image, 'processed_image', 10)
#         # (아래 Publisher들은 디버깅이나 다른 노드에서 상세 정보가 필요할 경우를 위해 남겨둡니다)
#         self.command_publisher = self.create_publisher(String, 'driving_command', 10)
#         self.person_detected_publisher = self.create_publisher(Bool, 'person_detected', 10)
#         self.person_distance_publisher = self.create_publisher(Float32, 'person_distance', 10)
        
#         self.bridge = CvBridge()
#         self.PERSON_STOP_DISTANCE = 4.0

#         try:
#             model_path = 'bestf.pt'
#             self.model = YOLO(model_path)
#             if torch.cuda.is_available():
#                 self.model.to('cuda')
#                 self.get_logger().info(f"YOLO model '{model_path}' loaded on CUDA.")
#             else:
#                 self.get_logger().warn('CUDA not available, using CPU.')
#                 self.get_logger().info(f"YOLO model '{model_path}' loaded on CPU.")
            
#             self.get_logger().info(f"Model classes: {self.model.names}")
#             self.yolo_available = True
#         except Exception as e:
#             self.get_logger().error(f"YOLO model loading failed: {str(e)}")
#             self.yolo_available = False

#         self.pipeline = None
#         self.align = None
#         self.init_camera()

#         self.timer = self.create_timer(1/30.0, self.timer_callback)

#     def init_camera(self):
#         # (카메라 초기화 코드는 이전과 동일)
#         try:
#             self.get_logger().info(f'Initializing RealSense Camera (Serial: {self.camera_serial})...')
#             ctx = rs.context()
#             devices = ctx.query_devices()
#             if not devices: raise Exception("No RealSense camera found")
#             selected_device = next((dev for dev in devices if dev.get_info(rs.camera_info.serial_number) == self.camera_serial), None)
#             if selected_device is None:
#                 self.get_logger().error(f'Target camera {self.camera_serial} not found. Using first available camera.')
#                 selected_device = devices[0]
#             serial = selected_device.get_info(rs.camera_info.serial_number)
#             self.pipeline = rs.pipeline()
#             config = rs.config()
#             config.enable_device(serial)
#             config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#             config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#             self.pipeline.start(config)
#             self.align = rs.align(rs.stream.color)
#             self.get_logger().info(f'RealSense camera initialized successfully - Serial: {serial}')
#         except Exception as e:
#             self.get_logger().error(f'Camera initialization failed: {str(e)}')
#             self.pipeline = None

#     def timer_callback(self):
#         if not self.pipeline or not self.yolo_available: return

#         try:
#             frames = self.pipeline.wait_for_frames(timeout_ms=5000)
#             aligned_frames = self.align.process(frames)
#             depth_frame = aligned_frames.get_depth_frame()
#             color_frame = aligned_frames.get_color_frame()
#             if not color_frame or not depth_frame: return
            
#             color_image = np.asanyarray(color_frame.get_data())
#             final_image = color_image.copy()

#             results = self.model(color_image, verbose=False)
            
#             # 감지 결과 저장을 위한 변수 초기화
#             is_person_detected = False
#             person_distance = float('inf')
#             traffic_light_class = None
#             max_conf_traffic = 0.0
#             yellow_box = None
#             blue_cones = []

#             for r in results:
#                 for box in r.boxes:
#                     cls = int(box.cls[0])
#                     confidence = float(box.conf[0])
#                     class_name = self.model.names[cls]
#                     x1, y1, x2, y2 = map(int, box.xyxy[0])

#                     if confidence < 0.4: continue # 신뢰도 임계값 조정

#                     # 모든 객체 감지 및 정보 수집
#                     if class_name == 'person':
#                         is_person_detected = True
#                         center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
#                         dist = depth_frame.get_distance(center_x, center_y)
#                         if 0 < dist < person_distance: person_distance = dist
#                         color = (0, 0, 255) if person_distance < self.PERSON_STOP_DISTANCE else (0, 255, 0)
#                         cv2.rectangle(final_image, (x1, y1), (x2, y2), color, 2)
#                         cv2.putText(final_image, f"Person: {person_distance:.2f}m", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    
#                     elif class_name in ['red', 'green', 'left', 'yellow']:
#                         if confidence > max_conf_traffic:
#                             max_conf_traffic = confidence
#                             traffic_light_class = class_name
#                         cv2.rectangle(final_image, (x1, y1), (x2, y2), (255, 255, 0), 2)
#                         cv2.putText(final_image, class_name, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

#                     elif class_name == 'box': 
#                         yellow_box = (x1, y1, x2 - x1, y2 - y1)
#                         cv2.rectangle(final_image, (x1, y1), (x2, y2), (0, 255, 255), 2)
#                         cv2.putText(final_image, "Box", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

#                     elif class_name == 'con':
#                         blue_cones.append((x1, y1, x2 - x1, y2 - y1))
#                         cv2.rectangle(final_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
#                         cv2.putText(final_image, "Cone", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            
#             # ================== 통합 명령 신호 결정 로직 ==================
#             final_control_signal = 1  # 기본값: 진행
#             command_string = "PROCEED (Default)"

#             # 우선순위 1: 콘 & 박스 패턴
#             if yellow_box and blue_cones:
#                 box_center_x = yellow_box[0] + yellow_box[2] / 2
#                 avg_cone_x = sum([c[0] + c[2] / 2 for c in blue_cones]) / len(blue_cones)
#                 if avg_cone_x < box_center_x:
#                     final_control_signal = 5  # 왼쪽
#                     command_string = "CONES ON LEFT"
#                 else:
#                     final_control_signal = 6  # 오른쪽
#                     command_string = "CONES ON RIGHT"
            
#             # 우선순위 2: 사람 (비상정지)
#             elif is_person_detected and person_distance < self.PERSON_STOP_DISTANCE:
#                 final_control_signal = 3
#                 command_string = f"EMERGENCY STOP (Person {person_distance:.1f}m)"

#             # 우선순위 3: 신호등
#             elif traffic_light_class is not None:
#                 if traffic_light_class == 'red':
#                     final_control_signal = 0
#                     command_string = "STOP (Red Light)"
#                 elif traffic_light_class == 'yellow':
#                     final_control_signal = 2
#                     command_string = "PREPARE STOP (Yellow Light)"
#                 elif traffic_light_class in ['green', 'left']:
#                     final_control_signal = 1
#                     command_string = f"PROCEED ({traffic_light_class})"
            
#             # ==========================================================

#             # 최종 결과 발행
#             self.driving_control_publisher.publish(Int32(data=final_control_signal))
#             self.command_publisher.publish(String(data=command_string))
            
#             # (디버깅용 상세 정보 발행)
#             self.person_detected_publisher.publish(Bool(data=is_person_detected))
#             if is_person_detected: self.person_distance_publisher.publish(Float32(data=person_distance))

#             # 시각화 및 이미지 발행
#             panel = np.zeros((60, final_image.shape[1], 3), dtype=np.uint8)
#             cv2.putText(panel, f"COMMAND: {command_string} (Signal: {final_control_signal})", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
#             final_display = np.vstack((final_image, panel))
#             self.processed_image_publisher.publish(self.bridge.cv2_to_imgmsg(final_display, "bgr8"))

#         except RuntimeError as e:
#             if "Frame didn't arrive" in str(e): self.get_logger().warn(f'Camera frame timeout: {str(e)}')
#             else: self.get_logger().error(f'Timer callback runtime error: {str(e)}')
#         except Exception as e:
#             self.get_logger().error(f'Timer callback general error: {str(e)}', exc_info=True)

# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         node = UnifiedVisionNode()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if 'node' in locals() and rclpy.ok():
#             if node.pipeline: node.pipeline.stop()
#             node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
# ~/ros2_ws/src/vision_system/vision_system/vision_publisher.py
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import pyrealsense2 as rs
import torch

class UnifiedVisionNode(Node):
    def __init__(self):
        super().__init__('unified_vision_node')

        self.declare_parameter('camera_serial', '313522303259')
        self.camera_serial = self.get_parameter('camera_serial').get_parameter_value().string_value
        self.get_logger().info(f'Using single camera with serial: {self.camera_serial}')

        # Publisher 선언 (driving_control이 핵심 역할을 합니다)
        self.driving_control_publisher = self.create_publisher(Int32, 'driving_control', 10)
        self.processed_image_publisher = self.create_publisher(Image, 'processed_image', 10)
        # (아래 Publisher들은 디버깅이나 다른 노드에서 상세 정보가 필요할 경우를 위해 남겨둡니다)
        self.command_publisher = self.create_publisher(String, 'driving_command', 10)
        self.person_detected_publisher = self.create_publisher(Bool, 'person_detected', 10)
        self.person_distance_publisher = self.create_publisher(Float32, 'person_distance', 10)
        
        self.bridge = CvBridge()
        self.PERSON_STOP_DISTANCE = 4.0

        try:
            model_path = 'jinmando.pt' # 사용하시는 모델 경로
            self.model = YOLO(model_path)
            if torch.cuda.is_available():
                self.model.to('cuda')
                self.get_logger().info(f"YOLO model '{model_path}' loaded on CUDA.")
            else:
                self.get_logger().warn('CUDA not available, using CPU.')
                self.get_logger().info(f"YOLO model '{model_path}' loaded on CPU.")
            
            self.get_logger().info(f"Model classes: {self.model.names}")
            self.yolo_available = True
        except Exception as e:
            self.get_logger().error(f"YOLO model loading failed: {str(e)}")
            self.yolo_available = False

        self.pipeline = None
        self.align = None
        self.init_camera()

        self.timer = self.create_timer(1/30.0, self.timer_callback)

    def init_camera(self):
        try:
            self.get_logger().info(f'Initializing RealSense Camera (Serial: {self.camera_serial})...')
            ctx = rs.context()
            devices = ctx.query_devices()
            if not devices: raise Exception("No RealSense camera found")
            selected_device = next((dev for dev in devices if dev.get_info(rs.camera_info.serial_number) == self.camera_serial), None)
            if selected_device is None:
                self.get_logger().error(f'Target camera {self.camera_serial} not found. Using first available camera.')
                selected_device = devices[0]
            serial = selected_device.get_info(rs.camera_info.serial_number)
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            self.get_logger().info(f'RealSense camera initialized successfully - Serial: {serial}')
        except Exception as e:
            self.get_logger().error(f'Camera initialization failed: {str(e)}')
            self.pipeline = None

    def timer_callback(self):
        if not self.pipeline or not self.yolo_available: return

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not color_frame or not depth_frame: return
            
            color_image = np.asanyarray(color_frame.get_data())
            final_image = color_image.copy()

            results = self.model(color_image, verbose=False)
            
            # 감지 결과 저장을 위한 변수 초기화
            is_person_detected = False
            is_car_detected = False  # <<<<<<<<<<<<<<<< [추가] 차량 감지 플래그
            person_distance = float('inf')
            traffic_light_class = None
            max_conf_traffic = 0.0
            yellow_box = None
            blue_cones = []

            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    confidence = float(box.conf[0])
                    class_name = self.model.names[cls]
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    if confidence < 0.4: continue # 신뢰도 임계값 조정

                    # 모든 객체 감지 및 정보 수집
                    if class_name == 'person':
                        is_person_detected = True
                        center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                        dist = depth_frame.get_distance(center_x, center_y)
                        if 0 < dist < person_distance: person_distance = dist
                        color = (0, 0, 255) if person_distance < self.PERSON_STOP_DISTANCE else (0, 255, 0)
                        cv2.rectangle(final_image, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(final_image, f"Person: {person_distance:.2f}m", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    
                    # <<<<<<<<<<<<<<<< [추가] 차량(car) 감지 로직
                    elif class_name == 'henes':
                        is_car_detected = True
                        color = (0, 165, 255) # 주황색으로 표시
                        cv2.rectangle(final_image, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(final_image, "Car", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                    elif class_name in ['red', 'green', 'left', 'yellow']:
                        if confidence > max_conf_traffic:
                            max_conf_traffic = confidence
                            traffic_light_class = class_name
                        cv2.rectangle(final_image, (x1, y1), (x2, y2), (255, 255, 0), 2)
                        cv2.putText(final_image, class_name, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

                    elif class_name == 'box': 
                        yellow_box = (x1, y1, x2 - x1, y2 - y1)
                        cv2.rectangle(final_image, (x1, y1), (x2, y2), (0, 255, 255), 2)
                        cv2.putText(final_image, "Box", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                    elif class_name == 'con':
                        blue_cones.append((x1, y1, x2 - x1, y2 - y1))
                        cv2.rectangle(final_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        cv2.putText(final_image, "Cone", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            
            # ================== [수정] 통합 명령 신호 결정 로직 (우선순위 기반) ==================
            final_control_signal = 1  # 기본값: 진행
            command_string = "PROCEED (Default)"

            # 우선순위 1: 정지 조건 (빨간불 또는 사람)
            if traffic_light_class == 'red':
                final_control_signal = 0
                command_string = "STOP (Red Light)"
            elif is_person_detected and person_distance < self.PERSON_STOP_DISTANCE:
                final_control_signal = 0 # 사람 감지 시에도 정지 신호 0 발행
                command_string = f"STOP (Person {person_distance:.1f}m)"
            
            # 우선순위 2: 차량 감지
            elif is_car_detected:
                final_control_signal = 3
                command_string = "CAUTION (Car Detected)"

            # 우선순위 3: 콘 & 박스 패턴
            elif yellow_box and blue_cones:
                box_center_x = yellow_box[0] + yellow_box[2] / 2
                avg_cone_x = sum([c[0] + c[2] / 2 for c in blue_cones]) / len(blue_cones)
                if avg_cone_x < box_center_x:
                    final_control_signal = 5  # 왼쪽
                    command_string = "CONES ON LEFT"
                else:
                    final_control_signal = 6  # 오른쪽
                    command_string = "CONES ON RIGHT"
            
            # 우선순위 4: 나머지 신호등
            elif traffic_light_class is not None:
                if traffic_light_class == 'yellow':
                    final_control_signal = 2
                    command_string = "PREPARE STOP (Yellow Light)"
                elif traffic_light_class in ['green', 'left']:
                    final_control_signal = 1
                    command_string = f"PROCEED ({traffic_light_class})"
            
            # ==============================================================================

            # 최종 결과 발행
            self.driving_control_publisher.publish(Int32(data=final_control_signal))
            self.command_publisher.publish(String(data=command_string))
            
            # (디버깅용 상세 정보 발행)
            self.person_detected_publisher.publish(Bool(data=is_person_detected))
            if is_person_detected: self.person_distance_publisher.publish(Float32(data=person_distance))

            # 시각화 및 이미지 발행
            panel = np.zeros((60, final_image.shape[1], 3), dtype=np.uint8)
            cv2.putText(panel, f"COMMAND: {command_string} (Signal: {final_control_signal})", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            final_display = np.vstack((final_image, panel))
            self.processed_image_publisher.publish(self.bridge.cv2_to_imgmsg(final_display, "bgr8"))

        except RuntimeError as e:
            if "Frame didn't arrive" in str(e): self.get_logger().warn(f'Camera frame timeout: {str(e)}')
            else: self.get_logger().error(f'Timer callback runtime error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Timer callback general error: {e}', exc_info=True)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = UnifiedVisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals() and rclpy.ok():
            if node.pipeline: node.pipeline.stop()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
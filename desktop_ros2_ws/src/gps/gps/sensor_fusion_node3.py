#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import utm
import math
import numpy as np

# 표준 메시지 타입
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from ublox_msgs.msg import NavPVT

def quaternion_to_yaw_deg(q: Quaternion) -> float:
    """
    쿼터니언을 오일러 각 Yaw (도, degree)로 변환합니다.
    """
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw_rad)

def normalize_angle_deg(degrees):
    """ 각도를 -180 ~ 180 범위로 정규화합니다. """
    return (degrees + 180) % 360 - 180

class HeadingFuser:
    """IMU의 Yaw와 GPS 헤딩을 융합하여 오프셋을 계산하고 보정합니다."""
    def __init__(self, alpha=0.05):
        self.heading_offset_deg = 0.0
        self.alpha = alpha  # 스무딩 팩터 (작을수록 부드럽게 변함)

    def update_offset(self, gps_heading_deg, imu_yaw_deg):
        """
        GPS와 IMU 헤딩 간의 차이를 계산하고, 오프셋을 부드럽게 업데이트합니다.
        """
        # 각도 차이를 -180 ~ 180 범위로 계산
        error = normalize_angle_deg(gps_heading_deg - imu_yaw_deg)
        
        # Low-pass filter를 이용해 오프셋을 부드럽게 갱신
        self.heading_offset_deg += self.alpha * normalize_angle_deg(error - self.heading_offset_deg)
        self.heading_offset_deg = normalize_angle_deg(self.heading_offset_deg)

    def get_fused_heading(self, imu_yaw_deg):
        """
        현재 IMU Yaw에 계산된 오프셋을 적용하여 최종 헤딩을 반환합니다.
        """
        fused = imu_yaw_deg + self.heading_offset_deg
        # 최종 헤딩을 0 ~ 360 범위로 변환하여 반환
        return fused % 360

class GpsImuFuserNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node3')

        self.last_lat = None
        self.last_lon = None
        self.last_velocity_ms = 0.0
        self.last_gps_heading_deg = None

        self.heading_fuser = HeadingFuser()

        # Subscribers
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.vel_sub = self.create_subscription(TwistWithCovarianceStamped, '/gps/velocity', self.velocity_callback, 10)
        self.navpvt_sub = self.create_subscription(NavPVT, '/gps/navpvt', self.navpvt_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/vehicle_odometry', 10)
        
        self.get_logger().info('GPS/IMU Fuser Node (Offset Correction) has been started.')

    def gps_callback(self, msg: NavSatFix):
        self.last_lat = msg.latitude
        self.last_lon = msg.longitude

    def velocity_callback(self, msg: TwistWithCovarianceStamped):
        self.last_velocity_ms = msg.twist.twist.linear.x

    def navpvt_callback(self, msg: NavPVT):
        # 차량이 일정 속도(예: 1.0 m/s) 이상으로 움직일 때만 GPS 헤딩을 신뢰
        if msg.g_speed > 1000: # g_speed 단위는 mm/s
            self.last_gps_heading_deg = msg.heading * 1e-5
        else:
            # 저속에서는 GPS 헤딩이 불안정하므로 사용하지 않음
            self.last_gps_heading_deg = None

    def imu_callback(self, msg: Imu):
        if self.last_lat is None:
            return

        # 1. IMU의 orientation에서 현재 Yaw(헤딩) 값을 도로(degree) 변환하여 추출
        current_imu_yaw_deg = quaternion_to_yaw_deg(msg.orientation)
        
        # GPS 헤딩이 유효할 때만 오프셋을 업데이트
        if self.last_gps_heading_deg is not None:
            # GPS 헤딩(북쪽 0도, 시계방향)을 IMU Yaw(동쪽 0도, 반시계방향)와 같은 기준으로 변환
            # GPS 헤딩 0(북) -> Yaw 90, GPS 헤딩 90(동) -> Yaw 0
            gps_yaw_equivalent_deg = normalize_angle_deg(90.0 - self.last_gps_heading_deg)
            
            self.heading_fuser.update_offset(gps_yaw_equivalent_deg, current_imu_yaw_deg)
            # 한 번 사용한 GPS 헤딩은 초기화하여 다음 신호를 기다림
            self.last_gps_heading_deg = None

        # 2. 보정된 최종 헤딩 값을 가져옴 (IMU Yaw + Offset)
        fused_heading_deg = self.heading_fuser.get_fused_heading(current_imu_yaw_deg)

        # 3. Odometry 메시지 생성 및 발행
        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        utm_coords = utm.from_latlon(self.last_lat, self.last_lon)
        odom_msg.pose.pose.position.x = utm_coords[0]
        odom_msg.pose.pose.position.y = utm_coords[1]

        # 최종 헤딩(fused_heading_deg)을 쿼터니언으로 변환하여 orientation 설정
        yaw_rad = math.radians(fused_heading_deg)
        q = Quaternion()
        q.z = math.sin(yaw_rad / 2.0)
        q.w = math.cos(yaw_rad / 2.0)
        odom_msg.pose.pose.orientation = q
        
        odom_msg.twist.twist.linear.x = self.last_velocity_ms
        odom_msg.twist.twist.angular = msg.angular_velocity

        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsImuFuserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




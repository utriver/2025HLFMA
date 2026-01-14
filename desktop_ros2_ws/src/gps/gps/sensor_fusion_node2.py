#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import utm
import time
import math

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, Quaternion
from ublox_msgs.msg import NavPVT
from nav_msgs.msg import Odometry

# PurePursuitDataProcessor와 수정된 SimpleComplementaryFuser 클래스
class PurePursuitDataProcessor:
    def __init__(self):
        self.heading_fuser = SimpleComplementaryFuser()
        self.current_state = {"x": None, "y": None, "velocity": 0.0, "heading": None}

    def update(self, imu_gyro_rate_dps, lat=None, lon=None, velocity_kmh=None, gps_heading_deg=None):
        if lat is not None and lon is not None:
            utm_coords = utm.from_latlon(lat, lon)
            self.current_state["x"] = utm_coords[0]
            self.current_state["y"] = utm_coords[1]
        if velocity_kmh is not None:
            self.current_state["velocity"] = velocity_kmh / 3.6
        
        fused_heading = self.heading_fuser.update(
            gyro_rate=imu_gyro_rate_dps, gps_heading=gps_heading_deg
        )
        self.current_state["heading"] = fused_heading
        return self.current_state

class SimpleComplementaryFuser:
    def __init__(self):
        self.current_heading = None
        self.last_time = None

    def update(self, gyro_rate, gps_heading=None):
        current_time = time.time()
        
        if self.last_time is None:
            self.last_time = current_time
            self.current_heading = gps_heading
            return self.current_heading

        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 1. 새로운 GPS 헤딩이 있으면, 현재 헤딩을 GPS 값으로 '리셋'
        if gps_heading is not None:
            self.current_heading = gps_heading
        
        # 2. GPS가 있든 없든, IMU의 회전량을 이용해 현재 헤딩을 업데이트
        if self.current_heading is not None:
            self.current_heading += gyro_rate * dt
            self.current_heading %= 360
            
        return self.current_heading

def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5); cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5); cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    q = Quaternion(); q.w = cr * cp * cy + sr * sp * sy; q.x = sr * cp * cy - cr * sp * sy; q.y = cr * sp * cy + sr * cp * sy; q.z = cr * cp * sy - sr * sp * cy
    return q

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node2')
        self.processor = PurePursuitDataProcessor()
        self.last_lat = None; self.last_lon = None; self.last_velocity_kmh = 0.0; self.last_gps_heading = None

        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.vel_sub = self.create_subscription(TwistWithCovarianceStamped, '/gps/velocity', self.velocity_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.navpvt_sub = self.create_subscription(NavPVT, '/gps/navpvt', self.navpvt_callback, 10)

        self.state_pub = self.create_publisher(Odometry, '/vehicle_odometry', 10)
        self.get_logger().info('Sensor Fusion Node (Corrected Logic) has been started.')

    def gps_callback(self, msg: NavSatFix):
        self.last_lat = msg.latitude; self.last_lon = msg.longitude

    def velocity_callback(self, msg: TwistWithCovarianceStamped):
        speed_ms = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        self.last_velocity_kmh = speed_ms * 3.6

    def navpvt_callback(self, msg: NavPVT):
        if msg.g_speed > 300: # 0.3m/s 이상으로 움직일 때만 헤딩 값 신뢰
            self.last_gps_heading = msg.heading * 1e-5

    def imu_callback(self, msg: Imu):
        gyro_rate_dps = msg.angular_velocity.z * (180.0 / math.pi)

        if self.last_lat is not None:
            gps_heading_to_use = self.last_gps_heading
            if gps_heading_to_use is not None:
                self.last_gps_heading = None

            final_state = self.processor.update(
                imu_gyro_rate_dps=gyro_rate_dps, lat=self.last_lat, lon=self.last_lon,
                velocity_kmh=self.last_velocity_kmh, gps_heading_deg=gps_heading_to_use
            )

            if final_state["x"] is not None and final_state["heading"] is not None:
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "odom"; odom_msg.child_frame_id = "base_link"
                odom_msg.pose.pose.position.x = final_state["x"]
                odom_msg.pose.pose.position.y = final_state["y"]
                odom_msg.pose.pose.position.z = 0.0
                
                heading_deg = final_state["heading"]
                yaw_enu_deg = (450.0 - heading_deg) % 360.0
                yaw_enu = math.radians(yaw_enu_deg)
                yaw_enu = math.atan2(math.sin(yaw_enu), math.cos(yaw_enu))

                odom_msg.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, yaw_enu)
                odom_msg.twist.twist.linear.x = final_state["velocity"]

                self.state_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args); node = SensorFusionNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()

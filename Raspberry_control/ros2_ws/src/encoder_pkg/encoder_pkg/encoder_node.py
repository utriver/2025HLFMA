import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import RPi.GPIO as GPIO
import math
import time

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        self.publisher = self.create_publisher(Odometry, '/encoder/odometry', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        GPIO.setmode(GPIO.BCM)
        self.ENC_FRONT_A = 18
        self.ENC_FRONT_B = 22
        self.ENC_STEERING = 16
        
        GPIO.setup(self.ENC_FRONT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENC_FRONT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENC_STEERING, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        GPIO.add_event_detect(self.ENC_FRONT_A, GPIO.FALLING, callback=self.encoder_callback)
        GPIO.add_event_detect(self.ENC_STEERING, GPIO.FALLING, callback=self.steering_callback)
        
        self.front_encoder_count = 0
        self.steering_encoder_count = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.omega = 0.0
        self.last_encoder_count = 0
        self.last_steering_count = 0
        self.last_time = time.time()
        
        self.wheel_diameter = 0.05
        self.encoder_ppr = 360
        self.steering_ratio = 0.5
        
        self.get_logger().info('✅ 엔코더 노드 시작!')
    
    def encoder_callback(self, channel):
        if GPIO.input(self.ENC_FRONT_B) == GPIO.LOW:
            self.front_encoder_count += 1
        else:
            self.front_encoder_count -= 1
    
    def steering_callback(self, channel):
        self.steering_encoder_count += 1
    
    def timer_callback(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        delta_front = self.front_encoder_count - self.last_encoder_count
        delta_steering = self.steering_encoder_count - self.last_steering_count
        
        self.last_encoder_count = self.front_encoder_count
        self.last_steering_count = self.steering_encoder_count
        
        distance_front = (delta_front / self.encoder_ppr) * math.pi * self.wheel_diameter
        steering_angle = delta_steering * self.steering_ratio * (math.pi / 180)
        
        if dt > 0:
            self.vx = distance_front / dt
            self.omega = steering_angle / dt
        
        if distance_front != 0:
            self.x += distance_front * math.cos(self.theta)
            self.y += distance_front * math.sin(self.theta)
            self.theta += steering_angle
            
            while self.theta > math.pi:
                self.theta -= 2 * math.pi
            while self.theta < -math.pi:
                self.theta += 2 * math.pi
        
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)
        
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.angular.z = self.omega
        
        self.publisher.publish(odom_msg)
        
        self.get_logger().info(
            f'📡 발행: x={self.x:.3f}m, y={self.y:.3f}m, θ={math.degrees(self.theta):.1f}°',
            throttle_duration_sec=1
        )
    
    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        encoder_node = EncoderNode()
        rclpy.spin(encoder_node)
    except KeyboardInterrupt:
        print('\n중단됨')
    finally:
        encoder_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

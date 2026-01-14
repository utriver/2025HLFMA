import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

class YawRatePublisher(Node):
    def __init__(self):
        super().__init__('yaw_rate_publisher_node')

        # 1. /imu/data 토픽을 구독할 Subscriber 생성
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)

        # 2. Z축 각속도를 발행할 Publisher 생성 (토픽 이름: /yaw_rate)
        self.yaw_rate_publisher = self.create_publisher(
            Float64,
            '/yaw_rate', # <-- 새로운 토픽 이름입니다.
            10)

        self.get_logger().info('Z-axis angular velocity extractor node has been started.')

    def imu_callback(self, msg):
        # Imu 메시지에서 z축 각속도(angular_velocity.z) 값을 추출
        z_angular_velocity = msg.angular_velocity.z

        # Float64 메시지 타입으로 새로운 메시지를 생성
        yaw_rate_msg = Float64()
        yaw_rate_msg.data = z_angular_velocity

        # /yaw_rate 토픽으로 메시지를 발행
        self.yaw_rate_publisher.publish(yaw_rate_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YawRatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

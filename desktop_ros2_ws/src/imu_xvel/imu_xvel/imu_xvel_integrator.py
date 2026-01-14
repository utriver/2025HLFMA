import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_matrix

class XVelIntegrator(Node):
    def __init__(self):
        super().__init__('imu_xvel_node')
        p = self.declare_parameters(
            namespace='',
            parameters=[
                ('imu_topic', '/imu/data'),
                ('vel_topic', '/xvel/twist'),
                ('odom_topic', '/xvel/odom'),
                ('publish_odom', False),
                ('frame_id_odom', 'odom'),
                ('child_frame_id', 'base_link'),
                ('use_free_accel', False),
                ('gravity', 9.80665),
                ('bias_tau', 3.0),
                ('zupt_enabled', True),
                ('zupt_gyro_thresh', 0.15),
                ('zupt_accel_thresh', 0.15),
                ('max_dt', 0.05),
                ('min_dt', 1e-4),
            ]
        )
        self.cfg = {pp.name: pp.value for pp in p}

        self.vel_pub = self.create_publisher(TwistStamped, self.cfg['vel_topic'], 10)
        self.odom_pub = self.create_publisher(Odometry, self.cfg['odom_topic'], 10) \
            if self.cfg['publish_odom'] else None
        self.sub = self.create_subscription(Imu, self.cfg['imu_topic'], self.cb_imu, 100)

        self.vx = 0.0
        self.bias_x = 0.0
        self.last_t = None

        self.x = 0.0
        self.y = 0.0
        self.get_logger().info('imu_xvel: integrating body-frame X velocity')

    def cb_imu(self, msg: Imu):
        # dt
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_t is None:
            self.last_t = t
            return
        dt = max(self.cfg['min_dt'], min(self.cfg['max_dt'], t - self.last_t))
        self.last_t = t

        # orientation
        q = msg.orientation
        if q.w == q.x == q.y == q.z == 0.0:
            return
        R = quaternion_matrix([q.w, q.x, q.y, q.z])[0:3, 0:3]  # body -> world

        # accel (body)
        a_body = np.array([msg.linear_acceleration.x,
                           msg.linear_acceleration.y,
                           msg.linear_acceleration.z])

        # gravity removal
        if self.cfg['use_free_accel']:
            a_lin = a_body
        else:
            g_world = np.array([0.0, 0.0, float(self.cfg['gravity'])])
            g_body = R.T @ g_world
            a_lin = a_body - g_body

        # ZUPT
        zupt = False
        if self.cfg['zupt_enabled']:
            w = msg.angular_velocity
            gyro_norm = math.sqrt(w.x*w.x + w.y*w.y + w.z*w.z)
            acc_resid = float(np.linalg.norm(a_lin))
            if gyro_norm < float(self.cfg['zupt_gyro_thresh']) and acc_resid < float(self.cfg['zupt_accel_thresh']):
                zupt = True

        # bias LPF
        tau = max(1e-3, float(self.cfg['bias_tau']))
        alpha = dt / tau
        beta = 1.0 if zupt else 0.1
        self.bias_x += beta * alpha * (a_lin[0] - self.bias_x)

        ax_corr = a_lin[0] - self.bias_x

        # integrate
        if zupt:
            self.vx = 0.0
        else:
            self.vx += ax_corr * dt

        # publish twist
        tw = TwistStamped()
        tw.header.stamp = msg.header.stamp
        tw.header.frame_id = self.cfg['child_frame_id']
        tw.twist.linear.x = float(self.vx)
        self.vel_pub.publish(tw)

        # optional dead-reckon odom (x만 world로 투영)
        if self.odom_pub is not None:
            yaw = math.atan2(R[1,0], R[0,0])
            self.x += self.vx * math.cos(yaw) * dt
            self.y += self.vx * math.sin(yaw) * dt

            od = Odometry()
            od.header.stamp = msg.header.stamp
            od.header.frame_id = self.cfg['frame_id_odom']
            od.child_frame_id = self.cfg['child_frame_id']
            od.pose.pose.position.x = self.x
            od.pose.pose.position.y = self.y
            od.pose.pose.position.z = 0.0
            od.pose.pose.orientation = msg.orientation
            od.twist.twist.linear.x = self.vx
            self.odom_pub.publish(od)

def main():
    rclpy.init()
    node = XVelIntegrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

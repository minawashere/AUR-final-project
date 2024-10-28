import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion
import math
import tf_transformations
from final_test.final_test.filters.madgwick import MadgwickAHRS
from final_test.final_test.filters.mahony import MahonyAHRS

class OdometryAndSensorFusionNode(Node):
    def __init__(self):
        super().__init__('odometry_sensor_fusion_node')

        self.subscription = self.create_subscription(
            Float32MultiArray, '/esp_data', self.sensor_data_callback, 10)
        
        self.publisher_ = self.create_publisher(Float32MultiArray, '/odom_data', 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw_imu = 0.0
        self.yaw_enc = 0.0
        self.pitch = 0.0

        
        self.wheel_radius = 0.06  
        self.wheel_base = 0.3     #>>>>>>>>>>>>>>>>>>
        self.ticks_per_rev = 1000  #>>>>>>>>>>>>>>>>>>

        
        self.madgwick = MadgwickAHRS(sampleperiod=0.05)
        self.mahony = MahonyAHRS(sampleperiod=0.05)

        self.left_last_count = 0
        self.right_last_count = 0
        self.dt = 0.05  #>>>>>>>>>>>>>>>>>>>

    def sensor_data_callback(self, msg):
        
        sensor_data = msg.data  #[acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, encoder_rpm_left, encoder_rpm_right]
        acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, encoder_rpm_left, encoder_rpm_right = sensor_data

        self.madgwick.update_imu(gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z)
        self.mahony.update_imu(gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z)

        q_madgwick = self.madgwick.quaternion
        q_mahony = self.mahony.quaternion
        pitch_mad, _, yaw_mad = tf_transformations.euler_from_quaternion(q_madgwick)
        pitch_mah, _, yaw_mah = tf_transformations.euler_from_quaternion(q_mahony)
        self.pitch = 0.5 * pitch_mad + 0.5 * pitch_mah
        self.yaw_imu = 0.5 * yaw_mad + 0.5 * yaw_mah

        #Encoder odometry calculations
        wheel_circumference = 2 * math.pi * self.wheel_radius
        left_dist = (encoder_rpm_left * self.dt) * wheel_circumference
        right_dist = (encoder_rpm_right * self.dt) * wheel_circumference

        # Linear and angular velocity
        linear_velocity = (left_dist + right_dist) / 2.0
        angular_velocity = (right_dist - left_dist) / self.wheel_base
        self.yaw_enc += angular_velocity * self.dt
        self.yaw = 0.9 * self.yaw_imu + 0.1 * self.yaw_enc
        self.x += linear_velocity * math.cos(self.yaw)
        self.y += linear_velocity * math.sin(self.yaw)
        odom_data = [self.x, self.y, self.yaw]
        self.publisher_.publish(Float32MultiArray(data=odom_data))
def main(args=None):
    rclpy.init(args=args)
    node = OdometryAndSensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import tf_transformations
from final_test.final_test.filters.madgwick import MadgwickAHRS
from final_test.final_test.filters.mahony import MahonyAHRS

class OdometryAndSensorFusionNode(Node):
    def __init__(self):
        super().__init__('odometry_sensor_fusion_node')

        # Subscribe to sensor data from Node 1
        self.subscription = self.create_subscription(
            Float32MultiArray, '/esp_data', self.sensor_data_callback, 10)
        
        # Publish odometry data to Node 3
        self.publisher_ = self.create_publisher(Float32MultiArray, '/odom_data', 10)
        self.pid_publisher=self.create_publisher(Float32MultiArray,"/pid_input",10)
        
        # Initialize parameters for position, orientation, and obstacle threshold
        self.x = 0.0
        self.y = 0.0
        self.yaw_imu = 0.0
        self.yaw_enc = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.obstacle_threshold = 0.3  # Adjust based on sensor range (meters)
        
        # Destination coordinates
        self.destination_x = 5.0
        self.destination_y = 5.0
        
        # Robot-specific parameters
        self.wheel_radius = 0.06  # meters
        self.wheel_base = 0.3     # meters
        self.ticks_per_rev = 1000  # Encoder ticks per revolution
        
        # IMU filters
        self.madgwick = MadgwickAHRS(sampleperiod=0.05)
        self.mahony = MahonyAHRS(sampleperiod=0.05)
        
        # Initialize last encoder values
        self.left_last_count = 0
        self.right_last_count = 0
        
        # Time step for odometry
        self.dt = 0.05  # assuming 50ms

    def sensor_data_callback(self, msg):
        # Unpack sensor data, with ultrasonic as the last element
        sensor_data = msg.data
        acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, encoder_rpm_left, encoder_rpm_right, ultrasonic = sensor_data

        # --- IMU sensor fusion ---
        self.madgwick.update_imu(gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z)
        self.mahony.update_imu(gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z)
        
        # Get quaternions from filters
        q_madgwick = self.madgwick.quaternion
        q_mahony = self.mahony.quaternion

        # Convert quaternions to Euler angles (pitch, yaw)
        pitch_mad, _, yaw_mad = tf_transformations.euler_from_quaternion(q_madgwick)
        pitch_mah, _, yaw_mah = tf_transformations.euler_from_quaternion(q_mahony)

        # Average pitch and yaw from both filters
        self.pitch = 0.5 * pitch_mad + 0.5 * pitch_mah
        self.yaw_imu = 0.5 * yaw_mad + 0.5 * yaw_mah

        # --- Encoder odometry calculations ---
        wheel_circumference = 2 * math.pi * self.wheel_radius
        left_dist = (encoder_rpm_left * self.dt) * wheel_circumference
        right_dist = (encoder_rpm_right * self.dt) * wheel_circumference

        # Linear and angular velocity
        linear_velocity = (left_dist + right_dist) / 2.0
        angular_velocity = (right_dist - left_dist) / self.wheel_base

        # Update encoder-based yaw
        self.yaw_enc += angular_velocity * self.dt

        # Fuse IMU and encoder yaw
        self.yaw = 0.9 * self.yaw_imu + 0.1 * self.yaw_enc

        # Update position (x, y)
        self.x += linear_velocity * math.cos(self.yaw)
        self.y += linear_velocity * math.sin(self.yaw)

        # Publish odometry data (x, y, yaw)
        odom_data = [self.x, self.y, self.yaw]
        self.publisher_.publish(Float32MultiArray(data=odom_data))
        self.pid_publisher.publish(Float32MultiArray(data=[self.x, self.y, self.yaw]))

        # Perform obstacle avoidance and continue moving if no obstacle
        if ultrasonic < self.obstacle_threshold:
            self.avoid_obstacle()
        else:
            self.continue_moving()
        
        # Logging for debugging
        self.get_logger().info(f"Odometry Data: x={self.x}, y={self.y}, yaw={self.yaw}, ultrasonic={ultrasonic}")

    def avoid_obstacle(self):
        # Rotate to avoid obstacle and try to find a clear path
        self.get_logger().info("Obstacle detected - Rotating to avoid")
        
        # Adjust yaw to rotate in place for obstacle avoidance
        self.yaw += 0.3  # Example rotation; adjust rate as needed
        self.publisher_.publish(Float32MultiArray(data=[self.x, self.y, self.yaw]))
        self.pid_publisher.publish(Float32MultiArray(data=[self.x, self.y, self.yaw]))

    def continue_moving(self):
        # Calculate distance to destination
        distance_to_goal = math.sqrt((self.destination_x - self.x)**2 + (self.destination_y - self.y)**2)
        angle_to_goal = math.atan2(self.destination_y - self.y, self.destination_x - self.x)
        angle_difference = angle_to_goal - self.yaw

        # Normalize angle difference to [-pi, pi]
        angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))
        
        # Adjust linear and angular velocity based on alignment
        if abs(angle_difference) > 0.1:
            # Adjust heading if not aligned with destination
            self.yaw += 0.5 if angle_difference > 0 else -0.5
        elif distance_to_goal > 0.1:
            # Move forward if aligned
            self.x += 0.1 * math.cos(self.yaw)  # Small forward step
            self.y += 0.1 * math.sin(self.yaw)

        # Publish updated position for movement
        self.publisher_.publish(Float32MultiArray(data=[self.x, self.y, self.yaw]))
        self.pid_publisher.publish(Float32MultiArray(data=[self.x, self.y, self.yaw]))
        self.get_logger().info("Moving toward goal")

def main(args=None):
    rclpy.init(args=args)

    # Create the odometry node
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

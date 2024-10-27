import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion, Twist
import math
import tf_transformations
from final_test.final_test.filters.madgwick import MadgwickAHRS
from final_test.final_test.filters.mahony import MahonyAHRS

class OdometryAndSensorFusionNode(Node):
    def _init_(self):
        super()._init_('odometry_sensor_fusion_node')

        # Subscribe to sensor data from Node 1
        self.subscription = self.create_subscription(
            Float32MultiArray, '/esp_data', self.sensor_data_callback, 10)
        
        # Publish odometry data to Node 3
        self.publisher_ = self.create_publisher(Float32MultiArray, '/odom_data', 10)
        
        # Publish velocity commands for semi-autonomous motion
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize odometry and sensor fusion parameters
        self.x = 0.0
        self.y = 0.0
        self.yaw_imu = 0.0
        self.yaw_enc = 0.0
        self.pitch = 0.0

        # Robot-specific parameters (adjust as necessary)
        self.wheel_radius = 0.06  # meters
        self.obstacle_threshold = 0.5  # meters; distance threshold for obstacle detection

        # Target coordinates for semi-autonomous navigation
        self.target_x = None
        self.target_y = None
        self.target_reached = False

        # Sensor fusion filters
        self.madgwick_filter = MadgwickAHRS(sample_period=1/50, beta=0.1)
        self.mahony_filter = MahonyAHRS(sample_period=1/50, kp=0.5)

    def sensor_data_callback(self, msg):
        # Process sensor data for odometry and sensor fusion here
        pass

    def obstacle_detection_callback(self, msg):
        # Obstacle detection logic using proximity sensor data
        distance = msg.data  # Example: assuming a single distance value in meters

        # If an obstacle is detected within the threshold, stop the robot
        if distance < self.obstacle_threshold:
            self.stop_robot()
            self.get_logger().info("Obstacle detected! Stopping robot.")

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.velocity_publisher.publish(stop_msg)

    def move_to_target(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y
        self.target_reached = False

        # Main control loop for navigating to the target
        while not self.target_reached:
            self.navigate_to_target()

    def navigate_to_target(self):
        if self.target_x is None or self.target_y is None:
            return  # No target set

        # Calculate distance to target
        distance_to_target = math.sqrt((self.target_x - self.x)*2 + (self.target_y - self.y)*2)
        
        # Check if the target has been reached
        if distance_to_target < 0.1:  # threshold for considering "reached"
            self.target_reached = True
            self.stop_robot()
            self.get_logger().info("Target reached!")
            return

        # Calculate angle to target
        angle_to_target = math.atan2(self.target_y - self.y, self.target_x - self.x)
        angle_diff = angle_to_target - self.yaw_imu

        # Create movement command
        move_msg = Twist()
        move_msg.linear.x = min(0.2, distance_to_target)  # Cap speed to 0.2 m/s
        move_msg.angular.z = angle_diff  # Turn towards target

        # Publish movement command
        self.velocity_publisher.publish(move_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryAndSensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
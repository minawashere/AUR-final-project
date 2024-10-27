import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import math

class NavigationObstacleNode(Node):
    def __init__(self):
        super().__init__('navigation_obstacle_node')

        # Initialize destination coordinates (set based on QR scan or desktop instructions)
        self.target_x = 2.0  # Replace with dynamic target as needed
        self.target_y = 3.0  # Replace with dynamic target as needed

        # Subscribe to odometry data
        self.odom_subscription = self.create_subscription(
            Float32MultiArray, '/odom_data', self.odom_callback, 10)

        # Subscribe to ultrasonic sensor data
        self.ultrasonic_subscription = self.create_subscription(
            Range, '/ultrasonic_data', self.ultrasonic_callback, 10)

        # Publisher to control robot movement
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters for movement
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.obstacle_distance_threshold = 0.5  # Meters, adjust for obstacle avoidance

    def odom_callback(self, msg):
        # Update current position and yaw
        self.current_x, self.current_y, self.current_yaw = msg.data

        # Check distance to target
        distance_to_target = math.sqrt(
            (self.target_x - self.current_x) ** 2 + (self.target_y - self.current_y) ** 2)

        # If close to the target, stop
        if distance_to_target < 0.1:
            self.stop_robot()
        else:
            # Navigate towards the target
            self.navigate_to_target()

    def ultrasonic_callback(self, msg):
        # Obstacle avoidance based on distance readings
        if msg.range < self.obstacle_distance_threshold:
            self.stop_robot()  # Stop if obstacle detected within threshold
            self.avoid_obstacle()
        else:
            # If no obstacle, continue navigating
            self.navigate_to_target()

    def navigate_to_target(self):
        # Calculate desired angle towards the target
        angle_to_target = math.atan2(
            self.target_y - self.current_y, self.target_x - self.current_x)

        # Calculate angular and linear velocities
        angular_error = angle_to_target - self.current_yaw
        linear_velocity = 0.2  # Base speed, adjust as needed
        angular_velocity = 0.5 * angular_error  # Proportional control for yaw

        # Create and publish movement command
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(twist)

    def avoid_obstacle(self):
        # Simple obstacle avoidance: stop, rotate, and then continue navigating
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.3  # Rotate to avoid obstacle
        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Stopping robot due to proximity to target or obstacle.')

def main(args=None):
    rclpy.init(args=args)

    # Create and spin navigation node
    node = NavigationObstacleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

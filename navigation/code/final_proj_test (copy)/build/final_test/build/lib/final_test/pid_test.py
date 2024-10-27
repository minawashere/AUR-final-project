import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # Subscribe to the odometry data published by the odometry node
        self.subscription = self.create_subscription(
            Float32MultiArray, '/pid_input', self.pid_input_callback, 10)

        # Publish left and right encoder velocities
        self.publisher_ = self.create_publisher(Float32MultiArray, '/encoder_velocities', 10)

        # PID gains (these should be tuned based on your system)
        self.kp = 1.0  # Proportional gain
        self.ki = 0.0  # Integral gain
        self.kd = 0.0  # Derivative gain

        self.prev_error = 0.0
        self.integral = 0.0

    def pid_input_callback(self, msg):
        x, y, yaw = msg.data  # Unpack the data

        # Implement PID control here (example for controlling x position)
        setpoint = 1.0  # Desired position
        error = setpoint - x
        self.integral += error
        derivative = error - self.prev_error

        # PID formula
        control_output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Update previous error
        self.prev_error = error

        # Calculate left and right velocities based on control output
        left_velocity = control_output
        right_velocity = control_output  # Modify this as needed for your control logic

        # Publish encoder velocities
        encoder_velocities = [left_velocity, right_velocity]
        self.publisher_.publish(Float32MultiArray(data=encoder_velocities))

        # Logging for debugging
        self.get_logger().info(f"Control Output: {control_output}, Left Velocity: {left_velocity}, Right Velocity: {right_velocity}")

def main(args=None):
    rclpy.init(args=args)

    # Create the PID controller node
    node = PIDControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

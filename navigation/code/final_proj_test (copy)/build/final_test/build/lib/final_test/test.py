import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import tf_transformations
from .filters.madgwick import MadgwickAHRS
from .filters.mahony import MahonyAHRS
import paho.mqtt.client as mqtt  # MQTT library
import json  # Library to handle JSON

class OdometryNode(Node):
    def __init__(self):
        super().__init__("OdometryNode")

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # Initialize parameters
        self.x = 0.0
        self.y = 0.0
        self.yaw_imu = 0.0
        self.yaw_enc = 0.0
        self.left_last_count = 0
        self.right_last_count = 0
        self.wheel_radius = 0.06
        self.wheel_base = 0.3
        self.ticks_per_rev = 1000
        self.madgwick = MadgwickAHRS(sampleperiod=0.05)
        self.mahony = MahonyAHRS(sampleperiod=0.05)

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect("broker.hivemq.com", 1883, 60)  # Replace with your broker

        # Start the MQTT client loop
        self.mqtt_client.loop_start()

    # MQTT callback for connection
    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT broker with result code {rc}")
        # Subscribe to the ESP32 data topic
        self.mqtt_client.subscribe("/esp32/data")

    # MQTT callback for received messages
    def on_message(self, client, userdata, msg):
        print(f"MQTT message received on topic {msg.topic}: {msg.payload.decode()}")
        try:
            # Parse the incoming JSON data
            data = json.loads(msg.payload.decode())

            # Extract encoder data
            left_encoder = data.get("left_encoder", 0)
            right_encoder = data.get("right_encoder", 0)

            # Process encoder data (e.g., calculate distance)
            self.process_encoder_data(left_encoder, right_encoder)

            # Extract IMU data
            imu_data = data.get("imu", {})
            if imu_data:
                acc = imu_data.get("acceleration", {})
                gyro = imu_data.get("gyroscope", {})
                self.process_imu_data(acc, gyro)
        except json.JSONDecodeError:
            print("Error: Invalid JSON data received")

    def process_encoder_data(self, left_encoder, right_encoder):
        delta_ticks_left = left_encoder - self.left_last_count
        self.left_last_count = left_encoder
        left_dist = 2 * math.pi * self.wheel_radius * (delta_ticks_left / self.ticks_per_rev)

        delta_ticks_right = right_encoder - self.right_last_count
        self.right_last_count = right_encoder
        right_dist = 2 * math.pi * self.wheel_radius * (delta_ticks_right / self.ticks_per_rev)

        # Calculate odometry
        linear_dist = (left_dist + right_dist) / 2.0
        delta_theta_enc = (right_dist - left_dist) / self.wheel_base
        self.yaw_enc += delta_theta_enc

        self.x += linear_dist * math.cos(self.yaw_enc)
        self.y += linear_dist * math.sin(self.yaw_enc)

        # Publish odometry
        self.publish_odometry()

    def process_imu_data(self, acc, gyro):
        # Extract accelerometer and gyroscope values
        ax = acc.get("x", 0)
        ay = acc.get("y", 0)
        az = acc.get("z", 0)

        gx = gyro.get("x", 0)
        gy = gyro.get("y", 0)
        gz = gyro.get("z", 0)

        # Update orientation using IMU data
        self.madgwick.update_imu(gx, gy, gz, ax, ay, az)
        self.mahony.update_imu(gx, gy, gz, ax, ay, az)

        q_madgwick = self.madgwick.quaternion
        q_mahony = self.mahony.quaternion

        _, _, yaw_mad = tf_transformations.euler_from_quaternion(q_madgwick)
        _, _, yaw_mah = tf_transformations.euler_from_quaternion(q_mahony)

        self.yaw_imu = 0.5 * yaw_mad + 0.5 * yaw_mah

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # Set orientation (using IMU and encoder data)
        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw_imu)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Publish the odometry message
        self.odom_pub.publish(odom)

        # Prepare data for MQTT
        mqtt_data = {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw_imu,
            "left_encoder": self.left_last_count,
            "right_encoder": self.right_last_count,
            "linear_velocity": (self.left_last_count + self.right_last_count) / 2,  # Example calculation
        }

        # Convert the data into JSON format
        json_data = json.dumps(mqtt_data)

        # Publish the JSON data to MQTT
        self.mqtt_client.publish("/robot/odometry", json_data)

        self.get_logger().info(f"Published MQTT message: {json_data}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import paho.mqtt.client as mqtt
import msgpack

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.subscription = self.create_subscription(
            Float32MultiArray, '/odom_data', self.odom_callback, 10)


        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("mqtt.eclipseprojects.io", 1883, 60)

    def odom_callback(self, msg):
        
        odom_data = msg.data  # [x, y, yaw]

        # Log the received odometry data
        self.get_logger().info(f"Received Odometry Data: {odom_data}")

        # Serialize the data using msgpack
        packed_data = msgpack.packb(odom_data)

        # Publish the serialized data to the MQTT topic
        self.mqtt_client.publish("/robot/odom_data", packed_data)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import msgpack

class MQTTPublisher(Node):
    def __init__(self):
        super().__init__('mqtt_publisher')
        self.client = mqtt.Client()
        
        # MQTT server connection (assuming broker is localhost, adjust as necessary)
        self.client.connect("mqtt.eclipseprojects.io", 1883, 60)
        
        self.create_timer(0.02, self.publish_data)

    def publish_data(self):
        # Example dictionary
        data =[0,0,0,0]
        
        # Serialize the data with msgpack
        packed_data = msgpack.packb(data)
        
        # Publish to the MQTT topic
        self.client.publish("Motion Commands", packed_data)
        
        self.get_logger().info(f"Published: {data}")

def main(args=None):
    rclpy.init(args=args)
    mqtt_publisher = MQTTPublisher()

    try:
        rclpy.spin(mqtt_publisher)
    except KeyboardInterrupt:
        mqtt_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

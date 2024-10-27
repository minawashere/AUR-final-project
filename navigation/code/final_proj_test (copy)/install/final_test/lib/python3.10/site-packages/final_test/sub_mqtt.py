import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import msgpack

class MQTTSubscriber(Node):
    def __init__(self):
        super().__init__('mqtt_subscriber')
        
        # Create the MQTT client
        self.client = mqtt.Client()
        
        # Set up callback for when a message is received
        self.client.on_message = self.on_message
        
        # Connect to the MQTT broker (adjust hostname if necessary)
        self.client.connect("mqtt.eclipseprojects.io", 1883, 60)
        
        # Subscribe to the topic
        self.client.subscribe("robot")
        
        # Start the MQTT client in the background
        self.client.loop_start()

    def on_message(self, client, userdata, message):
        try:
            # Unpack msgpack data
            packed_data = message.payload
            data = msgpack.unpackb(packed_data)
            
            # Log the received data
            self.get_logger().info(f"Received: {data}")
        except Exception as e:
            self.get_logger().error(f"Failed to unpack message: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    mqtt_subscriber = MQTTSubscriber()

    try:
        rclpy.spin(mqtt_subscriber)
    except KeyboardInterrupt:
        mqtt_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

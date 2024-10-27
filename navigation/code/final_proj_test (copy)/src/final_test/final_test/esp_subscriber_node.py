import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import paho.mqtt.client as mqtt
import msgpack

class Sub_node(Node):
    def __init__(self):
        super().__init__("sub_node")
        self.pub=self.create_publisher(Float32MultiArray,"/esp_data",10)
        self.client=mqtt.Client()
        self.client.on_connect=self.on_connect
        self.client.on_message=self.on_message
        self.client.connect("mqtt.eclipseprojects.io",1883,60)
        self.client.loop_start()


    def on_connect(self,client,userdata,flags,rc):
        self.get_logger().info("connect to MQTT")
        self.client.subscribe("from_esp")                   #edited

    def on_message(self,client,userdata,msg):
        payload=msg.payload
        sensor_data = msgpack.unpackb(payload, raw=False)
        data_msg = Float32MultiArray(data=sensor_data)
        self.publisher_.publish(data_msg)

        

def main(args=None):
        rclpy.init(args=args)
        node=Sub_node()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if (__name__=='__main__'):
        main()


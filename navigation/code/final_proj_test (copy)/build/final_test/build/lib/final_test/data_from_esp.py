import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math 
import tf_transformations
from geometry_msgs.msg import Quaternion
from final_test.final_test.filters.madgwick import MadgwickAHRS
from final_test.final_test.filters.mahony import MahonyAHRS

class Odometry(Node):
    def __init__(self):
        super().__init__("Odometry")
        self.odom_pub_=self.create_publisher(Odometry,"/odom",10)
        self.create_subscription(Int32,"/left_encoder",self.left_ecoder_callback,10)
        self.create_subscription(Int32,"/right_encoder",self.right_ecoder_callback,10)
        self.create_subscription(Imu,"/imu_data",self.imu_callback,10)
        self.timer=self.create_timer=self.create_timer(0.05,self.publish_odometry)
        self.create_subscription(Twist,"/cmd_vel",self.velocity_callback,10)

        self.x=0.0
        self.y=0.0
        self.theta_imu=0.0
        self.theta_encoder=0.0
        self.left_last_count=0
        self.right_last_count=0
        self.wheel_radius=0.06
        self.wheel_base = 0.3
        self.ticks_per_rev = 1000         #while be edited
        self.madgwick = MadgwickAHRS(sampleperiod=0.05)
        self.mahony = MahonyAHRS(sampleperiod=0.05)

        self.linear_velocity = 0.0  
        self.angular_velocity = 0.0

    def left_ecoder_callback(self,msg):
        delta_ticks=msg.data-self.left_last_count
        self.left_last_count=msg.data
        self.left_dist=2*math.pi*self.wheel_radius*(delta_ticks/self.ticks_per_rev)


    def right_ecoder_callback(self,msg):
        delta_ticks = msg.data - self.right_last_count
        self.right_last_count = msg.data
        self.right_dist=2*math.pi*self.wheel_radius* (delta_ticks / self.ticks_per_rev)

    def imu_callback(self,msg):
        ax=msg.linear_acceleration.x
        ay=msg.linear_acceleration.y
        az=msg.linear_acceleration.z
        gx=msg.angular_velocity.x
        gy=msg.angular_velocity.y
        gz=msg.angular_velocity.z

        self.madgwick.update_imu(gx,gy,gz,ax,ay,az)
        self.mahony.update_imu(gx,gy,gz,ax,ay,az)

        q_madgwick = self.madgwick.quaternion
        q_mahony = self.mahony.quaternion

        pitch_mad, _,yaw_mad=tf_transformations.euler_from_quaternion(q_madgwick)
        pitch_mah,_,yaw_mah= tf_transformations.euler_from_quaternion(q_mahony)

        self.pitch=0.5*pitch_mad+0.5*pitch_mah
        self.yaw_imu=0.5*yaw_mad+0.5*yaw_mah

    def velocity_callback(self,msg):
        self.linear_velocity=msg.linear.x
        self.angular_velocity = msg.angular.z

    def publish_data_Odometry(self):
        if hasattr(self,'left_dist') and hasattr(self,'right_dist'):
            linear_dist=(self.left_dist+self.right_dist)/2.0

            delta_theta_enc=(self.right_dist-self.left_dist)/self.wheel_base
            self.yaw_enc=self.yaw_enc+delta_theta_enc

            self.yaw=0.9*self.yaw_imu+0.1*self.yaw_enc
                        
            self.x +=linear_dist*math.cos(self.yaw)
            self.y +=linear_dist*math.sin(self.yaw)

            self.left_dist=0
            self.right_dist=0

        
        odom=Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = tf_transformations.quaternion_from_euler(self.pitch, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.odom_pub.publish(odom)

    def main(args=None):
        rclpy.init(args=args)
        node = Odometry()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

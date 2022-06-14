import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
import random


class OdomNoise(Node):

	def __init__(self):
		super().__init__('odom_noise')
		
		self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
		self.odom_subscriber # prevent unused variable warning
		
		self.odom_publisher = self.create_publisher(Odometry, '/noisy_odom', 10)
		
		self.odom_msg = Odometry()
		
		timer_period = 0.2  # seconds
		self.timer = self.create_timer(timer_period, self.publish_noisy_odom)
		
	def odom_callback(self, msg):
		self.odom_msg = msg
		self.add_noise()
		
	def add_noise(self):
		rand_float = random.uniform(-0.5,0.5)
		self.odom_msg.pose.pose.position.y = self.odom_msg.pose.pose.position.y + rand_float

	def publish_noisy_odom(self):
		self.odom_publisher.publish(self.odom_msg)



def main(args=None):
    rclpy.init(args=args)

    odom_noise = OdomNoise()

    rclpy.spin(odom_noise)

    odom_noise.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

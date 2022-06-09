import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
import random


class PoseNoise(Node):

	def __init__(self):
		super().__init__('pose_noise')
		
		self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
		self.pose_subscriber # prevent unused variable warning
		
		self.pose_publisher = self.create_publisher(Pose, '/turtle1/noisy_pose', 10)
		
		self.pose_msg = Pose()
		
		timer_period = 0.2  # seconds
		self.timer = self.create_timer(timer_period, self.publish_noisy_pose)
		
	def pose_callback(self, msg):
		self.pose_msg = msg
		self.add_noise()
		
	def add_noise(self):
		rand_float = random.uniform(-0.5,0.5)
		self.pose_msg.x = self.pose_msg.x + rand_float

	def publish_noisy_pose(self):
		self.pose_publisher.publish(self.pose_msg)



def main(args=None):
    rclpy.init(args=args)

    pose_noise = PoseNoise()

    rclpy.spin(pose_noise)

    pose_noise.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

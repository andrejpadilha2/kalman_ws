import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn, TeleportAbsolute
from turtlesim.msg import Pose
from numpy.random import randn
import math

class PoseNoise(Node):

	def __init__(self):
		super().__init__('pose_noise_turtlesim')
		
		# Subscribes to turtle1 pose
		self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
		self.pose_subscriber # prevent unused variable warning
		
		self.pose_msg = Pose()
		self.noisy_pose_msg = Pose()
		
		self.declare_parameter('topic_name')
		self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
		self.pose_publisher = self.create_publisher(Pose, self.topic_name, 10) #creates the topic with the noisy measurements
		
		self.declare_parameter('R')
		self.declare_parameter('dt')
		self.R = self.get_parameter('R').get_parameter_value().double_value # seconds
		dt = self.get_parameter('dt').get_parameter_value().double_value # seconds
		self.timer = self.create_timer(dt, self.publish_noisy_pose)	# timer that to set the frequency of sensor measurements
		
	def pose_callback(self, msg):
		self.pose_msg = msg
		
	def add_noise(self):
		self.noisy_pose_msg.x = self.pose_msg.x + randn()*math.sqrt(self.R)
		self.noisy_pose_msg.y = self.pose_msg.y + randn()*math.sqrt(self.R)
		self.noisy_pose_msg.theta = self.pose_msg.theta

	def publish_noisy_pose(self):
		self.add_noise()
		self.pose_publisher.publish(self.noisy_pose_msg)		

def main(args=None):
    rclpy.init(args=args)

    pose_noise = PoseNoise()

    rclpy.spin(pose_noise)

    pose_noise.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

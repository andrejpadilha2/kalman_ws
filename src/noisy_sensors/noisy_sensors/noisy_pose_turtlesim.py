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
		
		self.pose_publisher = self.create_publisher(Pose, '/turtle1/noisy_pose', 10) #creates the topic with the noisy measurements
		
		self.spawn_turtle(turtle_name='noisy_turtle1')
		
		self.declare_parameter('R')
		self.declare_parameter('dt')
		self.R = self.get_parameter('R').get_parameter_value().double_value # seconds
		dt = self.get_parameter('dt').get_parameter_value().double_value # seconds
		self.timer = self.create_timer(dt, self.publish_noisy_pose)	# timer that to set the frequency of sensor measurements
		
	def pose_callback(self, msg):
		self.pose_msg = msg
		
	def add_noise(self):
		self.noisy_pose_msg.x = self.pose_msg.x + randn()*math.sqrt(self.R)
		self.noisy_pose_msg.y = self.pose_msg.y # no noise in 1d version
		self.noisy_pose_msg.theta = self.pose_msg.theta

	def publish_noisy_pose(self):
		self.add_noise()
		self.pose_publisher.publish(self.noisy_pose_msg)
		self.teleport_turtle()
		
	def spawn_turtle(self, turtle_name):
		# Spawns a turtle to visualize noisy measurements
		self.spawn_client = self.create_client(Spawn, 'spawn')
		while not self.spawn_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Spawn service not available, waiting again...')
		self.spawn_req = Spawn.Request()
		self.spawn_req.x = 0. # hard coded
		self.spawn_req.y = 6.544445 # hard coded
		self.spawn_req.theta = self.pose_msg.theta
		self.spawn_req.name = turtle_name
		self.future = self.spawn_client.call_async(self.spawn_req)
		
		# Starts teleport service client to teleport noisy_turtle, hence reflecting noisy measurements on screen
		self.teleport_client = self.create_client(TeleportAbsolute, "/" + turtle_name + '/teleport_absolute')
		while not self.teleport_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Teleport service for ' + turtle_name + ' not available, waiting again...')
		self.teleport_req = TeleportAbsolute.Request()
		
	def teleport_turtle(self):
		self.teleport_req.x = self.noisy_pose_msg.x
		self.teleport_req.y = self.noisy_pose_msg.y + 1 # hard coded
		self.teleport_req.theta = self.noisy_pose_msg.theta
		self.future = self.teleport_client.call_async(self.teleport_req)

def main(args=None):
    rclpy.init(args=args)

    pose_noise = PoseNoise()

    rclpy.spin(pose_noise)

    pose_noise.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

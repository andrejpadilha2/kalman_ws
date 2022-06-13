import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn, TeleportAbsolute
from turtlesim.msg import Pose
import random


class PoseKalmanFilter(Node):

	def __init__(self):
		super().__init__('pose_kalman_filter')
		
		# Subscribes to turtle1 noisy_pose
		self.pose_subscriber = self.create_subscription(Pose, '/turtle1/noisy_pose', self.pose_callback, 10)
		self.pose_subscriber # prevent unused variable warning
		
		self.pose_msg = Pose()
		self.noisy_pose_msg = Pose()
		
		self.pose_publisher = self.create_publisher(Pose, '/turtle1/kf_pose', 10) #creates the topic with the filtered measurements
		
		# Spawns a turtle to visualize filtered measurements
		self.spawn_client = self.create_client(Spawn, 'spawn')
		while not self.spawn_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Spawn service not available, waiting again...')
		noisy_turtle_name = "kf_turtle1"
		self.spawn_req = Spawn.Request()
		self.spawn_req.x = 5.544445 # hard coded
		self.spawn_req.y = 5.544445 # hard coded
		self.spawn_req.theta = self.pose_msg.theta
		self.spawn_req.name = noisy_turtle_name
		self.future = self.spawn_client.call_async(self.spawn_req)
		
		
		# Starts teleport service client to teleport noisy_turtle, hence reflecting noisy measurements on screen
		self.teleport_client = self.create_client(TeleportAbsolute, "/" + noisy_turtle_name + '/teleport_absolute')
		while not self.teleport_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Teleport service for ' + noisy_turtle_name + ' not available, waiting again...')
		self.teleport_req = TeleportAbsolute.Request()
		
		
		timer_period = 0.2  # seconds
		self.timer = self.create_timer(timer_period, self.publish_noisy_pose)	# timer that to set the frequency of sensor measurements
		
	def pose_callback(self, msg):
		self.pose_msg = msg
		
	def add_noise(self):
		rand_float = random.uniform(-0.5,0.5)
		self.noisy_pose_msg.x = self.pose_msg.x + rand_float
		self.noisy_pose_msg.y = self.pose_msg.y # no noise in 1d version
		self.noisy_pose_msg.theta = self.pose_msg.theta

	def publish_noisy_pose(self):
		self.add_noise()
		self.pose_publisher.publish(self.noisy_pose_msg)
		self.teleport_req.x = self.noisy_pose_msg.x
		self.teleport_req.y = self.noisy_pose_msg.y
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
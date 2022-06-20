import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn, TeleportAbsolute
from turtlesim.msg import Pose
import random

from collections import namedtuple

class PoseKalmanFilter(Node):
	gaussian = namedtuple('Gaussian', ['mean', 'var'])
	gaussian.__repr__ = lambda s: f'𝒩(μ={s[0]:.3f}, 𝜎²={s[1]:.3f})'

	def __init__(self):
		self.process_var = .01 # variance in the turtle's movement
		self.sensor_var = .5 # variance in the sensor

		self.velocity = .3
		self.dt = 0.02 # time step in seconds
		self.x = PoseKalmanFilter.gaussian(0., 20.**2)  # initial turtle's position, N(0, 20**2)
		self.process_model = PoseKalmanFilter.gaussian(self.velocity*self.dt, self.process_var) # displacement to add to x CHECK HERE FIRST
		
		super().__init__('pose_kalman_filter_turtlesim')
		
		# Subscribes to turtle1 noisy_pose
		self.noisy_pose_subscriber = self.create_subscription(Pose, '/turtle1/noisy_pose', self.noisy_pose_callback, 10)
		self.noisy_pose_subscriber # prevent unused variable warning
		
		self.noisy_pose_msg = Pose()
		self.kf_pose_msg = Pose()
		
		self.kf_pose_publisher = self.create_publisher(Pose, '/turtle1/kf_pose', 10) # creates the topic with the filtered measurements
		
		# Spawns a turtle to visualize filtered measurements
		self.spawn_client = self.create_client(Spawn, 'spawn')
		while not self.spawn_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Spawn service not available, waiting again...')
		kf_turtle_name = "kf_turtle1"
		self.spawn_req = Spawn.Request()
		self.spawn_req.x = 5.544445 # hard coded
		self.spawn_req.y = 4.544445 # hard coded
		self.spawn_req.theta = self.noisy_pose_msg.theta
		self.spawn_req.name = kf_turtle_name
		self.future = self.spawn_client.call_async(self.spawn_req)
		
		
		# Starts teleport service client to teleport kf_turtle, hence reflecting filtered measurements on screen
		self.teleport_client = self.create_client(TeleportAbsolute, "/" + kf_turtle_name + '/teleport_absolute')
		while not self.teleport_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Teleport service for ' + kf_turtle_name + ' not available, waiting again...')
		self.teleport_req = TeleportAbsolute.Request()
		
		timer_period = self.dt  # seconds
		self.timer = self.create_timer(timer_period, self.publish_kf_pose)	# timer that to set the frequency of filter messages
		
	def noisy_pose_callback(self, msg):
		self.noisy_pose_msg = msg

	def publish_kf_pose(self):
		self.kalman_filter()
		self.kf_pose_publisher.publish(self.kf_pose_msg)
		self.teleport_req.x = self.kf_pose_msg.x
		self.teleport_req.y = self.kf_pose_msg.y - 1
		self.teleport_req.theta = self.kf_pose_msg.theta
		self.future = self.teleport_client.call_async(self.teleport_req)
	
	def gaussian_multiply(self, g1, g2):
		mean = (g1.var * g2.mean + g2.var * g1.mean) / (g1.var + g2.var)
		variance = (g1.var * g2.var) / (g1.var + g2.var)
		return PoseKalmanFilter.gaussian(mean, variance)
		
	def predict(self, pos, movement):
		return PoseKalmanFilter.gaussian(pos.mean + movement.mean, pos.var + movement.var)

	def update(self, prior, likelihood):
		posterior = self.gaussian_multiply(likelihood, prior)
		return posterior
    	
	def kalman_filter(self):
		self.prior = self.predict(self.x, self.process_model)
		self.likelihood = PoseKalmanFilter.gaussian(self.noisy_pose_msg.x, self.sensor_var)
		self.x = self.update(self.prior, self.likelihood)
		self.kf_pose_msg.x = self.x.mean
		self.kf_pose_msg.y = self.noisy_pose_msg.y
		self.kf_pose_msg.theta = self.noisy_pose_msg.theta


def main(args=None):
    rclpy.init(args=args)

    pose_kalman_filter = PoseKalmanFilter()

    rclpy.spin(pose_kalman_filter)

    pose_kalman_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn, TeleportAbsolute
from turtlesim.msg import Pose
import random
from .position_filter_1d import position_filter_1d

class PoseKalmanFilterNode(Node):

	def __init__(self):	
		super().__init__('pose_kalman_filter_turtlesim')
		
		# Subscribes to turtle1 noisy_pose
		self.noisy_pose_subscriber = self.create_subscription(Pose, '/turtle1/noisy_pose', self.noisy_pose_callback, 10)
		self.noisy_pose_subscriber # prevent unused variable warning
		
		self.noisy_pose_msg = Pose()
		self.kf_pose_msg = Pose()
		
		self.kf_pose_publisher = self.create_publisher(Pose, '/turtle1/kf_pose', 10) # creates the topic with the filtered measurements
		
		self.spawn_turtle(turtle_name="kf_turtle1")
		
		timer_period = 0.02  # seconds
		self.kf = position_filter_1d(
						x=0.,			# initial position mean
						P=20.**2,		# initial position variance
						vel=.3,		# velocity of the turtle
						R=.25,			# sensor/measurement variance
						Q=.001,		# process variance
						dt=timer_period)	# time step in seconds
		
		self.timer = self.create_timer(timer_period, self.publish_kf_pose)	# timer to set the frequency of filter messages
		
	def noisy_pose_callback(self, msg):
		self.noisy_pose_msg = msg

	def publish_kf_pose(self):
		# KALMAN FILTER ALGORITHM
		self.kf.predict() # PREDICT STEP
		self.kf.update(self.noisy_pose_msg.x) # UPDATE STEP
		#########################
		
		self.get_logger().info("P: %.3f, R: %.3f, K: %.3f\n" % (self.kf.P, self.kf.R, self.kf.K))
		
		self.kf_pose_msg.x = self.kf.x # writes kalman filter position mean on kf_pose_msg.x
		self.kf_pose_msg.y = self.noisy_pose_msg.y # in 1d we don't play with the y axis
		self.kf_pose_msg.theta = self.noisy_pose_msg.theta # in 1d we don't play with theta
		self.kf_pose_publisher.publish(self.kf_pose_msg) # publish filtered pose
		self.teleport_turtle()
		
	def spawn_turtle(self, turtle_name):
		# Spawns a turtle to visualize filtered measurements
		self.spawn_client = self.create_client(Spawn, 'spawn')
		while not self.spawn_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Spawn service not available, waiting again...')
		self.spawn_req = Spawn.Request()
		self.spawn_req.x = 5.544445 # hard coded
		self.spawn_req.y = 5.544445 # hard coded
		self.spawn_req.theta = self.noisy_pose_msg.theta
		self.spawn_req.name = turtle_name
		self.future = self.spawn_client.call_async(self.spawn_req)
		
		# Starts teleport service client to teleport kf_turtle, hence reflecting filtered measurements on screen
		self.teleport_client = self.create_client(TeleportAbsolute, "/" + turtle_name + '/teleport_absolute')
		while not self.teleport_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Teleport service for ' + turtle_name + ' not available, waiting again...')
		self.teleport_req = TeleportAbsolute.Request()

	def teleport_turtle(self):
		self.teleport_req.x = self.kf_pose_msg.x
		self.teleport_req.y = self.kf_pose_msg.y - 1
		self.teleport_req.theta = self.kf_pose_msg.theta
		self.future = self.teleport_client.call_async(self.teleport_req)

def main(args=None):
    rclpy.init(args=args)

    pose_kalman_filter = PoseKalmanFilterNode()

    rclpy.spin(pose_kalman_filter)

    pose_kalman_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

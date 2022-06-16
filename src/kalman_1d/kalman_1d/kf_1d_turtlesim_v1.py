import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn, TeleportAbsolute
from turtlesim.msg import Pose
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
		
		self.declare_parameter('x')
		self.declare_parameter('P')
		self.declare_parameter('v')
		self.declare_parameter('R')
		self.declare_parameter('Q')
		self.declare_parameter('dt')
		x = self.get_parameter('x').get_parameter_value().double_value
		P = self.get_parameter('P').get_parameter_value().double_value
		v = self.get_parameter('v').get_parameter_value().double_value
		R = self.get_parameter('R').get_parameter_value().double_value
		Q = self.get_parameter('Q').get_parameter_value().double_value
		dt = self.get_parameter('dt').get_parameter_value().double_value
		
		self.kf = position_filter_1d(
						x=x,			# initial position mean
						P=P,			# initial position variance
						vel=v,			# velocity of the turtle
						R=R,			# sensor/measurement variance
						Q=Q,			# process variance
						dt=dt)			# time step in seconds
		
		self.timer = self.create_timer(dt, self.publish_kf_pose)	# timer to set the frequency of filter messages
		
	def noisy_pose_callback(self, msg):
		self.noisy_pose_msg = msg

	def publish_kf_pose(self):
		# KALMAN FILTER ALGORITHM
		self.kf.predict() 			# PREDICT STEP
		self.kf.update(self.noisy_pose_msg.x) 	# UPDATE STEP
		#########################
		
		self.get_logger().info("P: %.3f, R: %.3f, K: %.3f\n" % (self.kf.P, self.kf.R, self.kf.K))
		
		self.kf_pose_msg.x = self.kf.x 			# writes kalman filter position mean on kf_pose_msg.x
		self.kf_pose_msg.y = self.noisy_pose_msg.y 		# in 1d we don't play with the y axis
		self.kf_pose_msg.theta = self.noisy_pose_msg.theta 	# in 1d we don't play with theta
		self.kf_pose_publisher.publish(self.kf_pose_msg)	 # publish filtered pose

def main(args=None):
    rclpy.init(args=args)

    pose_kalman_filter = PoseKalmanFilterNode()

    rclpy.spin(pose_kalman_filter)

    pose_kalman_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

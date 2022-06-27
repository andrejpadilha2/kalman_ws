import rclpy
from rclpy.node import Node
import numpy as np

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from .pos_vel_filter_sensor_fusion_in_x import pos_vel_filter_sensor_fusion_in_x

class PoseKalmanFilterNode(Node):

	def __init__(self):	
		super().__init__('pose_kalman_filter_turtlesim')
		self.z = np.array([[0, 0]]).T
		self.u = np.array([[0]])
		
		# Subscribes to turtle1 noisy1_pose
		self.noisy_pose_subscriber = self.create_subscription(Pose, '/turtle1/noisy_pose', self.noisy_pose_callback, 10)
		self.noisy_pose_subscriber # prevent unused variable warning
		
		# Subscribes to turtle1 noisy2_pose
		self.noisy2_pose_subscriber = self.create_subscription(Pose, '/turtle1/noisy2_pose', self.noisy2_pose_callback, 10)
		self.noisy2_pose_subscriber # prevent unused variable warning
		
		# Subscribes to keyboard_teleop commands
		self.keyboard_teleop_subscriber = self.create_subscription(Twist, '/turtle1/cmd_vel_no_noise', self.keyboard_teleop_callback, 10)
		self.keyboard_teleop_subscriber # prevent unused variable warning
		
		self.noisy_pose_msg = Pose()
		self.noisy2_pose_msg = Pose()
		self.kf_pose_msg = Pose()
		self.keyboard_teleop_msg = Twist()
		
		self.kf_pose_publisher = self.create_publisher(Pose, '/turtle1/kf_pose', 10) # creates the topic with the filtered measurements
		
		self.declare_parameter('Q_var')
		self.declare_parameter('R_var')
		self.declare_parameter('dt')
		Q_var = self.get_parameter('Q_var').get_parameter_value().double_value
		R_var = self.get_parameter('R_var').get_parameter_value().double_value
		dt = self.get_parameter('dt').get_parameter_value().double_value
		
		F = np.array([	[1., 0.],		# the state transition matrix (relation of variable states)
				[0., 0.]])

		B = np.array([	[dt],			# control function
				[1.]])
		
		self.kf = pos_vel_filter_sensor_fusion_in_x(
						Q_var=Q_var,	# process variance
						R_var=R_var,	# sensor/measurement covariance matrix
						dt=dt,		# time step in seconds
						F=F,
						B=B)
		
		self.timer = self.create_timer(dt, self.publish_kf_pose)	# timer to set the frequency of filter messages
		
	def keyboard_teleop_callback(self, msg):
		self.keyboard_teleop_msg = msg
		self.u = np.array([self.keyboard_teleop_msg.linear.x]).T	
		
	def noisy_pose_callback(self, msg):
		self.noisy_pose_msg = msg
		self.z = np.array([[self.noisy_pose_msg.x, self.noisy2_pose_msg.x]]).T # create msg with two sensor measurements on axis x and y (two sensors, four measurements)
		
	def noisy2_pose_callback(self, msg):
		self.noisy2_pose_msg = msg

	def publish_kf_pose(self):
		# KALMAN FILTER ALGORITHM
		self.kf.predict(self.u) 		# PREDICT STEP
		self.kf.update(self.z) 		# UPDATE STEP
		#########################
		
		self.get_logger().info("P_pos: %.3f, P_vel: %.3f, cov_pos_vel: %.3f, R: %.3f, K: %.3f, x_pos: %.3f, x_vel: %.3f\n" % (self.kf.P[0,0], self.kf.P[1,1], self.kf.P[1,0], self.kf.R[0,0], self.kf.K[0,0], self.kf.x[0,0], self.kf.x[1,0]))
		
		self.kf_pose_msg.x = self.kf.x[0,0]			# writes kalman filter position mean of x on kf_pose_msg.x
		self.kf_pose_msg.y = self.noisy_pose_msg.y		# we don't play with the y axis in this example
		self.kf_pose_msg.theta = self.noisy_pose_msg.theta 	# in 2d we don't play with theta
		self.kf_pose_publisher.publish(self.kf_pose_msg)	# publish filtered pose

def main(args=None):
    rclpy.init(args=args)

    pose_kalman_filter = PoseKalmanFilterNode()

    rclpy.spin(pose_kalman_filter)

    pose_kalman_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

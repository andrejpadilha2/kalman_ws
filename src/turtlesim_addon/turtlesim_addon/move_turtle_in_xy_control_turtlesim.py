import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from numpy.random import randn
import math

class MoveTurtle(Node):

	def __init__(self):	
		super().__init__('move_turtlesim')
			
		self.keyboard_teleop_msg = Twist()
		self.twist_msg = Twist()
		
		# Subscribes to keyboard_teleop commands with no noise
		self.keyboard_teleop_subscriber = self.create_subscription(Twist, '/turtle1/cmd_vel_no_noise', self.keyboard_teleop_callback, 10)
		self.keyboard_teleop_subscriber # prevent unused variable warning
		
		self.twist_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) # creates the publisher for twist messages
		
		self.declare_parameter('Q')
		self.Q = self.get_parameter('Q').get_parameter_value().double_value # process noise
				
		self.timer = self.create_timer(0.2, self.publish_twist)	# timer to set the frequency of twist messages
		
	def keyboard_teleop_callback(self, msg):
		self.keyboard_teleop_msg = msg	
		
	def publish_twist(self):
		if (self.keyboard_teleop_msg.linear.x == 0):
			vel_x = 0.
		else:
			vel_x = self.keyboard_teleop_msg.linear.x + (randn() * math.sqrt(self.Q))
		
		if (self.keyboard_teleop_msg.linear.y == 0):
			vel_y = 0.
		else:
			vel_y = self.keyboard_teleop_msg.linear.y + (randn() * math.sqrt(self.Q))
		
		self.twist_msg.linear.x = vel_x 
		self.twist_msg.linear.y = vel_y 
		self.twist_publisher.publish(self.twist_msg) # publish filtered pose

def main(args=None):
    rclpy.init(args=args)

    move_turtle = MoveTurtle()

    rclpy.spin(move_turtle)

    move_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

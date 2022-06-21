import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from numpy.random import randn
import math

class MoveTurtle(Node):

	def __init__(self):	
		super().__init__('move_turtlesim')
			
		self.twist_msg = Twist()
		
		self.twist_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) # creates the publisher for twist messages
		
		self.declare_parameter('v')
		self.declare_parameter('Q')
		self.v = self.get_parameter('v').get_parameter_value().double_value # velocity
		self.Q = self.get_parameter('Q').get_parameter_value().double_value # process noise
				
		self.timer = self.create_timer(0.2, self.publish_twist)	# timer to set the frequency of twist messages
		
	def publish_twist(self):
		vel_x = self.v + (randn() * math.sqrt(self.Q))
		vel_y = self.v + (randn() * math.sqrt(self.Q))
		
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

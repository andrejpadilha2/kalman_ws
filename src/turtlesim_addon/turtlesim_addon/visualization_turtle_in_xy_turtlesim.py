import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn, TeleportAbsolute, SetPen
from turtlesim.msg import Pose
from std_srvs.srv import Empty

import time

class VisualizationTurtle(Node):

	def __init__(self):
		super().__init__('name') # it will be overwritten by the name given in launch file
		self.declare_parameter('turtle_name')
		self.turtle_name = self.get_parameter('turtle_name').get_parameter_value().string_value
		self.declare_parameter('spawn_location')
		self.spawn_location = self.get_parameter('spawn_location').get_parameter_value().double_array_value
		self.declare_parameter('pen_color')
		self.pen_color = self.get_parameter('pen_color').get_parameter_value().integer_array_value
		self.declare_parameter('pen_width')
		self.pen_width = self.get_parameter('pen_width').get_parameter_value().integer_value
		self.declare_parameter('pen_off')
		self.pen_off = self.get_parameter('pen_off').get_parameter_value().integer_value	
			
		# Subscribes to self.turtle_name_pose
		self.pose_subscriber = self.create_subscription(Pose, '/turtle1/'+self.turtle_name+'_pose', self.pose_callback, 10)
		self.pose_subscriber # prevent unused variable warning
		self.pose_msg = Pose()
		
		# Spawns the turtle
		self.spawn_turtle(self.turtle_name)
		
		# Change pen characteristics
		self.pen_client = self.create_client(SetPen, '/' + self.turtle_name + '_visualization_turtle1/set_pen')
		while not self.pen_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Set pen service for ' + self.turtle_name + '_turtle1 not available, waiting again...')
		self.set_pen_req = SetPen.Request()
		self.set_pen()
				
		# Starts teleport service client to teleport self.turle_name, hence reflecting filtered measurements on screen
		self.teleport_client = self.create_client(TeleportAbsolute, '/' + self.turtle_name + '_visualization_turtle1/teleport_absolute')
		while not self.teleport_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Teleport service for ' + self.turtle_name + '_turtle1 not available, waiting again...')
		self.teleport_req = TeleportAbsolute.Request()
	
	def pose_callback(self, msg):
		self.pose_msg = msg
		self.teleport_turtle()
			
	def teleport_turtle(self):
		self.teleport_req.x = self.pose_msg.x
		self.teleport_req.y = self.pose_msg.y
		self.teleport_req.theta = self.pose_msg.theta
		self.future = self.teleport_client.call_async(self.teleport_req)
		
	def spawn_turtle(self, turtle_name):
		# Spawns a turtle for visualization
		self.spawn_client = self.create_client(Spawn, 'spawn')
		while not self.spawn_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Spawn service not available, waiting again...')
		self.spawn_req = Spawn.Request()
		self.spawn_req.x = self.spawn_location[0]
		self.spawn_req.y = self.spawn_location[1]
		self.spawn_req.theta = self.spawn_location[2]
		self.spawn_req.name = turtle_name + '_visualization_turtle1'
		self.future = self.spawn_client.call_async(self.spawn_req)
		
	def set_pen(self):		
		self.set_pen_req.r = self.pen_color[0]
		self.set_pen_req.g = self.pen_color[1]
		self.set_pen_req.b = self.pen_color[2]
		self.set_pen_req.width = self.pen_width
		self.set_pen_req.off = self.pen_off
		self.future = self.pen_client.call_async(self.set_pen_req)
		self.clear_req = Empty.Request()
		self.clear_client = self.create_client(Empty, '/clear')
		while not self.clear_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Clear service not available, waiting again...')
		time.sleep(2)
		self.future = self.clear_client.call_async(self.clear_req)

def main(args=None):
    rclpy.init(args=args)

    visualization_turtle = VisualizationTurtle()

    rclpy.spin(visualization_turtle)

    visualization_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

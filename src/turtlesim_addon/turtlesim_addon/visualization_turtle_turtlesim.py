import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn, TeleportAbsolute
from turtlesim.msg import Pose
from numpy.random import randn
import math

class VisualizationTurtle(Node):

	def __init__(self):
		super().__init__('name') # it will be overwritten by the name given in launch file
		self.declare_parameter('turtle_name')
		self.turtle_name = self.get_parameter('turtle_name').get_parameter_value().string_value	
		self.declare_parameter('y_offset')
		self.y_offset = self.get_parameter('y_offset').get_parameter_value().integer_value
			
		# Subscribes to self.turtle_name_pose
		self.pose_subscriber = self.create_subscription(Pose, '/turtle1/'+self.turtle_name+'_pose', self.pose_callback, 10) # change here
		self.pose_subscriber # prevent unused variable warning
		self.pose_msg = Pose()
		
		self.spawn_turtle(self.turtle_name)
				
		# Starts teleport service client to teleport self.turle_name, hence reflecting filtered measurements on screen
		self.teleport_client = self.create_client(TeleportAbsolute, '/' + self.turtle_name + '_turtle1/teleport_absolute')
		while not self.teleport_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Teleport service for ' + self.turtle_name + '_turtle1 not available, waiting again...')
		self.teleport_req = TeleportAbsolute.Request()
	
	def pose_callback(self, msg):
		self.pose_msg = msg
		self.teleport_turtle()
			
	def teleport_turtle(self):
		self.teleport_req.x = self.pose_msg.x
		self.teleport_req.y = self.pose_msg.y + self.y_offset
		self.teleport_req.theta = self.pose_msg.theta
		self.future = self.teleport_client.call_async(self.teleport_req)
		
	def spawn_turtle(self, turtle_name):
		# Spawns a turtle for visualization
		self.spawn_client = self.create_client(Spawn, 'spawn')
		while not self.spawn_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Spawn service not available, waiting again...')
		self.spawn_req = Spawn.Request()
		self.spawn_req.x = 0. # hard coded
		self.spawn_req.y = self.pose_msg.y + self.y_offset
		self.spawn_req.theta = self.pose_msg.theta
		self.spawn_req.name = turtle_name + '_turtle1'
		self.future = self.spawn_client.call_async(self.spawn_req)

def main(args=None):
    rclpy.init(args=args)

    visualization_turtle = VisualizationTurtle()

    rclpy.spin(visualization_turtle)

    visualization_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

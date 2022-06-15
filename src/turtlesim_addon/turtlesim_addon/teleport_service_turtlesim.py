import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn, TeleportAbsolute
from turtlesim.msg import Pose

class TeleportServiceTurtlesim(Node):
	def __init__(self):
		super().__init__('teleport_service_turtlesim')
		
		# Subscribes to turtle1 noisy_pose
		self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.noisy_pose_callback, 10)
		self.pose_subscriber # prevent unused variable warning
		
		self.pose_msg = Pose()		
		
		# Starts teleport service client to teleport main turtle
		self.teleport_client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
		while not self.teleport_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Teleport service for turtle1 not available, waiting again...')
		self.teleport_req = TeleportAbsolute.Request()
		
	def noisy_pose_callback(self, msg):
		self.pose_msg = msg
		if self.pose_msg.x >= 11:
			self.teleport_req.x = 0.
			self.teleport_req.y = self.pose_msg.y
			self.teleport_req.theta = self.pose_msg.theta
			self.future = self.teleport_client.call_async(self.teleport_req)


def main(args=None):
    rclpy.init(args=args)

    teleport_service_turtlesim = TeleportServiceTurtlesim()

    rclpy.spin(teleport_service_turtlesim)

    teleport_service_turtlesim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

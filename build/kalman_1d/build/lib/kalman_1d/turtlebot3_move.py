import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

#ros2 topic pub --oce /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
class Turtlebot3_Cmd_Vel_Publisher(Node):

    def __init__(self):
        super().__init__('turtlebot3_move_pub')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.
        #msg.linear.y = 0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing...')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    tlb3_cmd_vel_publisher = Turtlebot3_Cmd_Vel_Publisher()
    rclpy.spin(tlb3_cmd_vel_publisher )

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

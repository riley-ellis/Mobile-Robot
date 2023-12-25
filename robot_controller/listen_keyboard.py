#!/usr/bin/enc python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class keyboardSubscriber(Node):
	def __init__(self):
		super().__init__("laptop_keyboard")
		self.keyboard_subscriber = self.create_subscription(
			Twist, "/cmd_vel", self.keyboard_callback, 10)

	def keyboard_callback(self, msg: Twist):
		self.get_logger().info(str(msg.linear.x))
		self.get_logger().info(str(msg.angular.z))

def main(args=None):

	rclpy.init(args=args)
	node = keyboardSubscriber()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()

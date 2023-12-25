#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from .kevin_components import Robot, Motor

class RobotNode(Node):
    def __init__(self):

        super().__init__('ps4_control')

        #create robot instance
        self.robot = Robot()

        #create subscription
        self.ps4_subscriber = self.create_subscription(
            Joy, 'joy', self.control_callback, 10)

    def control_callback(self, msg):
        left_speed = msg.axes[1]
        right_speed = msg.axes[4]
        x_cmd = msg.buttons[0]
        square_cmd = msg.buttons[3]
        triangle_cmd = msg.buttons[2]

        if x_cmd > 0:
            self.robot.stop()
            self.get_logger().info(str(x_cmd))
        elif square_cmd > 0:
            self.robot.start()
        elif triangle_cmd > 0:
            self.robot.cleanup()
        else:
            self.robot.skid(left_speed, right_speed)

    def on_shutdown(self):
        self.robot.cleanup()


def main(args=None):
    rclpy.init(args=args)
    kevin_node = RobotNode()
    
    try:
        rclpy.spin(kevin_node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            kevin_node.on_shutdown()
            kevin_node.destroy_node()
        except Exception as e:
            kevin_node.get_logger().error(f'Error during shutdown: {e}')
        finally:
             if rclpy.ok():
                 rclpy.shutdown()

if __name__ == '__main__':
    main()

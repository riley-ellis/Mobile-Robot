#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
import gpsd
import time

class GPSCarNode(Node):
    def __init__(self):
        super().__init__('car_gps_node')
        self.car_gps_publisher = self.create_publisher(
            NavSatFix, 'car_gps', 10)
        self.car_gps_heading_vel_publisher = self.create_publisher(
            TwistStamped, 'car_heading_vel', 10)
        #timer to control how often i collect data:
        self.car_gps_timer = self.create_timer(1.0, self.get_gps_data)

        try:
            gpsd.connect()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to GPSD: {e}")
            rclpy.shutdown()
    
    def get_gps_data(self):
        try: 
            packet = gpsd.get_current()

            if packet.mode >=2:
                self.get_logger().info("Satellite Fix!")
                navsatfix_msg = NavSatFix()
                navsatfix_msg.latitude = packet.lat
                navsatfix_msg.longitude = packet.lon
                #utilizing TwistStamped message type to avoid creating custom message
                heading_vel_msg = TwistStamped()
                heading_vel_msg.twist.linear.x = packet.hspeed #horizontal speed
                heading_vel_msg.twist.angular.z = packet.track #track/course
                current_time = self.get_clock().now().to_msg()
                heading_vel_msg.header.stamp = current_time
                navsatfix_msg.header.stamp = current_time
                
                self.car_gps_publisher.publish(navsatfix_msg)
                self.car_gps_heading_vel_publisher.publish(heading_vel_msg)
                
                #logging
                self.get_logger().info(f"Latitude: {navsatfix_msg.latitude}")
                self.get_logger().info(f"Longitude: {navsatfix_msg.longitude}")
                self.get_logger().info(f"Speed: {heading_vel_msg.twist.linear.x}")
                self.get_logger().info(f"Track: {heading_vel_msg.twist.angular.z}")
            else:
                self.get_logger().info("Not enough Satellites")
                self.get_logger().info(f"Satellites: {packet.sats}")


        except Exception as e:
            self.get_logger().error(f"Error while getting GPS data: {e}")

def main(args=None):
    rclpy.init(args=args)
    car_gps_node = GPSCarNode()
    rclpy.spin(car_gps_node)
    car_gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
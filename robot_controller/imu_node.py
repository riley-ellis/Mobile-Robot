#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import math
import sys
sys.path.insert(0, '/home/riley/ros_projects/mobile_robot_ws/src/robot_controller/robot_controller/BerryIMU/')
import IMU
import datetime
import os
import sys
from std_msgs.msg import Float64

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.imu_publisher = self.create_publisher(
            Float64, 'imu/heading', 10)
        #create timer for calling callback
        self.timer = self.create_timer(
            0.05, self.timer_callback)

        #initialization of IMU:
        self.M_PI = 3.14159265358979323846

        ################# Compass Calibration values ############
        # Use calibrateBerryIMU.py to get calibration values

        self.magXmin = 550
        self.magYmin = -3396
        self.magZmin = 6982
        self.magXmax = 5326
        self.magYmax = 188
        self.magZmax = 10110


        IMU.detectIMU()     #Detect if BerryIMU is connected.
        if(IMU.BerryIMUversion == 99):
            self.get_logger().error("No BerryIMU found... exiting ")
            rclpy.shutdown()
        IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass

    def timer_callback(self):

        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()

        #Apply compass calibration
        MAGx -= (self.magXmin + self.magXmax) /2
        MAGy -= (self.magYmin + self.magYmax) /2
        MAGz -= (self.magZmin + self.magZmax) /2

        #Calculate heading
        heading = 180 * math.atan2(MAGy,MAGx)/self.M_PI

        #Only have our heading between 0 and 360
        if heading < 0:
            heading += 360

        msg = Float64()
       # msg.timestamp = self.get_clock().now().to_msg()
        msg.data = heading
        self.imu_publisher.publish(msg)
       # self.get_logger().info('Heading: "%f"' % msg.heading)
        self.get_logger().info('Heading: "%f"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

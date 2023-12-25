#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pandas as pd
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Float64
import logging
import std_msgs.msg

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('data_collection_node')

class DataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')
        self.car_gps_subscriber = self.create_subscription(
            NavSatFix, '/car_gps', self.car_gps_callback, 10)
        self.car_gps_heading_vel_subscriber = self.create_subscription(
            TwistStamped, '/car_heading_vel', self.car_gps_heading_vel_callback, 10)
        self.car_imu_subscriber = self.create_subscription(
            Float64, '/imu/heading', self.car_imu_heading_callback, 10)

        self.gps_df = pd.DataFrame(columns=['time', 'latitude', 'longitude', 'speed', 'track'])
        self.mag_df = pd.DataFrame(columns=['time', 'heading'])

        # Initialize buffer variables for GPS data
        self.gps_data = None
        self.heading_vel_data = None

        self.declare_parameter("file_path", "~/ros_projects/data_collection/data.xlsx")  # Default path

        rclpy.on_shutdown(self.on_shutdown)


    def car_gps_callback(self, msg):
        timestamp = msg.header.stamp
        latitude = msg.latitude
        longitude = msg.longitude
        self.gps_data = {'time': timestamp, 'latitude': latitude, 'longitude': longitude}
        self.write_gps_data()
    
    def car_gps_heading_vel_callback(self, msg):
        timestamp = msg.header.stamp
        speed = msg.twist.linear.x
        heading = msg.twist.angular.z
        self.heading_vel_data = {'time': timestamp, 'speed': speed, 'heading': heading}
        self.write_gps_data()

    def write_gps_data(self):
        if self.gps_data is not None and self.heading_vel_data is not None:
            combined_gps_data = {**self.gps_data, **self.heading_vel_data}
            #convert to df so i can use concat
            temp_df = pd.DataFrame([combined_gps_data])
            #now use concat function
            self.gps_df = pd.concat([self.gps_df, temp_df], ignore_index=True)
            #reset buffer
            self.gps_data = None
            self.heading_vel_data = None
    
    def car_imu_heading_callback(self, msg):
        timestamp = self.get_clock().now().to_msg()
        mag_heading = msg.data
        new_data = pd.DataFrame({'time': [timestamp], 'heading': [mag_heading]})
        self.mag_df = pd.concat([self.mag_df, new_data], ignore_index=True)

    def save_data(self):
        file_path = self.get_parameter("file_path").get_parameter_value().string_value
        try:
            with pd.ExcelWriter(file_path) as writer:
                self.gps_df.to_excel(writer, sheet_name='GPS Data', index=False)
                self.mag_df.to_excel(writer, sheet_name='IMU Data', index=False)
                logger.info(f"Saving data to {file_path}")
                return True  # Indicate success
        except (FileNotFoundError, PermissionError) as e:
            logger.error(f"Error saving data: {e}")
            return False  # Indicate failure

    
    def on_shutdown(self):
        self.save_data()
        super().on_shutdown()



def main(args=None):
    rclpy.init(args=args)
    data_collection_node = DataCollectionNode()
    rclpy.spin(data_collection_node)
    #data_collection_node.save_data()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
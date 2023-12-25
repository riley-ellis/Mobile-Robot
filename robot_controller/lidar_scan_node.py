#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rplidar import RPLidar
import numpy as np
import threading

class lidarScanNode(Node):
    def __init__(self):
        super().__init__('lidar_scan')

        #initialize lidar object
        self.lidar = RPLidar('/dev/ttyUSB0')
        #initialize publisher
        self.lidar_publisher = self.create_publisher(LaserScan, 'scan', 10)
        
        #adding is_active flag
        self.is_active = True

        #thread to process lidar data
        self.lidar_thread = threading.Thread(target=self.process_lidar_data)
        self.lidar_thread.daemon = True #thread exits when program ends
        self.lidar_thread.start()

        


    def process_lidar_data(self):
        try:
            while self.is_active: #check if node is still active
                for scan in self.lidar.iter_scans():
                    interpolated_scan = self.interpolate_scan(scan)
                    laserscan_msg = self.create_laserscan_msg(interpolated_scan)
                    self.lidar_publisher.publish(laserscan_msg)
                    if not self.is_active: #checks again before publishing:
                        break
        except Exception as e:
            if rclpy.ok(): #only log error if ros context still active...
                self.get_logger().error('Error while processing lidar data: %r' % (e,))

    def interpolate_scan(self, scan):
        # Initialize a full scan with NaNs
        full_scan = np.full(360, np.nan)

        # Populate the full_scan array with known distances
        for quality, angle, distance in scan:
            rounded_angle = int(round(angle))
            full_scan[rounded_angle % 360] = distance

        # Extend the scan to handle wrap-around at 0 and 359 degrees
        extended_scan = np.concatenate((full_scan[-5:], full_scan, full_scan[:5]))

        # Create an extended angle array
        extended_angles = np.arange(-5, 365)

        # Create a mask for known values in the extended scan
        known_mask = np.isfinite(extended_scan)

        # Interpolate missing values in the extended scan
        extended_scan[~known_mask] = np.interp(extended_angles[~known_mask], extended_angles[known_mask], extended_scan[known_mask])

        # Return the interpolated scan trimmed back to the original range
        return extended_scan[5:-5]



    def create_laserscan_msg(self, interpolated_scan):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_frame'  # Set the appropriate frame id

        # Set other necessary LaserScan fields
        msg.angle_min = 0.0
        msg.angle_max = 2 * np.pi
        msg.angle_increment = np.pi / 180  # 360 degrees
        msg.range_min = 0.0  # Update as per your lidar specs
        msg.range_max = 12000.0  # Update as per your lidar specs
        msg.ranges = interpolated_scan.tolist()
        return msg
    
    def on_shutdown(self):
        # Signal the background thread to stop
        self.is_active = False

        # Wait for the background thread to finish
        if self.lidar_thread.is_alive():
            self.lidar_thread.join()

        # Stop the lidar and disconnect
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

def main(args=None):
    rclpy.init(args=args)
    node = lidarScanNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            # Custom shutdown logic
            node.on_shutdown()
            # Properly destroy the node
            node.destroy_node()
        except Exception as e:
            node.get_logger().error(f'Error during shutdown: {e}')
        finally:
            if rclpy.ok():
                # Shutdown the ROS2 Python client library, if not already down
                rclpy.shutdown()

if __name__ == '__main__':
    main()


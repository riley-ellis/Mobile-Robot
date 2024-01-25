#!/usr/bin/env python3
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan
from .kevin_components import Robot
import numpy as np
import threading
import queue
from time import sleep

class lidarToCSVNode(Node):
	def __init__(self):

		super().__init__('lidar_to_csv')
		self.get_logger().info("initializing node...")

		#define robot's max motor speed:
		max_motor_power = 25 #edit this to change overall power of robot (100 being max)
		#create robot
		self.robot = Robot(max_motor_power)
		#define max speeds of left and right motors
		self.robot.left_motor.max_speed = round(self.robot.max_motor_power) #edit these to make max speed of each motor equal
		self.robot.right_motor.max_speed = round(self.robot.max_motor_power*0.93) #for now this can be done by visual observation (not perfect but close)

		#subscribe to joy
		self.ps4_subscriber = self.create_subscription(
            Joy, 'joy', self.control_callback, 10)

		#subscriber to lidar
		self.lidar_subscriber = self.create_subscription(
			LaserScan, 'scan', self.lidar_callback, 10)

		#control data
		self.current_control_states = []

		#create que for data writing
		self.write_queue = queue.Queue()
		#create thread for execution of data writing from the queue
		self.writer_thread = threading.Thread(target=self.write_to_file)
		self.writer_thread.daemon = True #this ensures that thread automatically closes when program ends
		self.writer_thread.start()

		#initialize csv
		self.csv_file_name = 'lidar_data.csv'
		with open(self.csv_file_name, mode='w', newline='') as file:
			writer = csv.writer(file)
			#csv header
			writer.writerow(['Lidar Data', 'Average Control State'])

	#create method that writes to file (which is called in the thread)
	def write_to_file(self):
		while True:
			item = self.write_queue.get()
			if item is None:
				break  # Exit loop if shutdown signal is received
			scan_data, avg_state = item
			with open(self.csv_file_name, mode='a', newline='') as file:
				writer = csv.writer(file)
				writer.writerow([scan_data, avg_state])
				print(scan_data)
			self.write_queue.task_done()
			if not rclpy.ok():
				self.get_logger().info("logging within thread")
				break

	def control_callback(self, msg):
		#capture control state (left straight or right)
		state = msg.axes[6]
		x = msg.buttons[0]
		#append it to list
		self.current_control_states.append(state)

		#inputs to class Robot.skid() for left_speed and right_speed, should be between [-1, 1]
		#i think this means i nee to divide my current inputs by 100
		#i should think of my left and right speed inputs as percentage of the pwm value i want for a given motor

		#determine control ouput
		if round(state) == -1:
			#set left speed slower to turn left (edit these for desired turning rate)
			self.robot.skid(self.robot.left_motor.max_speed/100, self.robot.right_motor.max_speed*0.6/100)
		elif round(state) == 1:
			#set right speed slower to turn right (edit these for desired turning rate)
			self.robot.skid(self.robot.left_motor.max_speed*0.6/100, self.robot.right_motor.max_speed/100)
		elif round(state) == 0:
			#robot goes straight
			self.robot.skid(self.robot.left_motor.max_speed/100, self.robot.right_motor.max_speed/100)
		if round(x) == 1:
			self.robot.skid(0, 0)

	def lidar_callback(self, msg):
		#get lidar data
		scan_data = list(msg.ranges) #distances

		#compute the average state of this scan
		if self.current_control_states:
			avg_state = np.mean(self.current_control_states)
		else:
			avg_state = 0 #if no data is available this is the default

		#put data in the queue to be written to csv in separate thread
		self.get_logger().info('hey')
		self.write_queue.put((scan_data, avg_state))

		#reset control states
		self.current_control_states = []

	def on_shutdown(self):
		# Check if the writer_thread was created and exists
		if hasattr(self, 'writer_thread'):
			# Tell the writing thread it's time to exit
			self.write_queue.put(None)
			# Wait for the thread to finish
			self.writer_thread.join()

def main(args=None):
	rclpy.init(args=args)
	node = lidarToCSVNode()

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		try:
			# Custom shutdown logic
			node.on_shutdown()
			# sleep(5)
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


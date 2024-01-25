import RPi.GPIO as GPIO
import csv
from kevin_components import Robot
from collections import deque
import time
import numpy as np
import sys
sys.path.insert(0, '/home/riley/ros_projects/mobile_robot_ws/src/robot_controller/robot_controller/BerryIMU/')
import IMU

G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly

IMU.detectIMU()
IMU.detectIMU()     #Detect if BerryIMU is connected.
if(IMU.BerryIMUversion == 99):
    print(" No BerryIMU found... exiting ")
    sys.exit()
IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass


# Initialize the robot
target_power = 25 # Approximate power level for the motors
robot = Robot(target_power)

# Moving average window
window_size = 10
rate_gyr_zs = deque(maxlen=window_size)

# CSV file setup
csv_file = open('robot_ang_vel_data_rev.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Time', 'Moving Average Angular Vel (deg/s)'])

# Apply specific PWM signals and record data
robot.skid(0.25, -0.25)  # Positive for left motor, negative for right motor
start_time = time.time()
last_time = start_time

last_rate_gyr_z = 0


#main loop

try:
    while time.time() - start_time < 3: #record for 3 sec
        current_time = time.time()
        delta_time = current_time - last_time

        if delta_time >= 0.01: #update every 10ms
            rate_gyr_z = IMU.readGYRz()*G_GAIN
            rate_gyr_zs.append(rate_gyr_z)
            avg_rate_gyr_zs = np.mean(rate_gyr_zs)

            #write to csv
            csv_writer.writerow([current_time - start_time, avg_rate_gyr_zs])

            #reset time
            last_time = current_time

except KeyboardInterrupt: 
    print("Program stopped by user")
finally:
    # Stop the robot and clean up
    robot.skid(0, 0)
    csv_file.close()
    GPIO.cleanup()
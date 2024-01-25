import RPi.GPIO as GPIO
import time
import numpy as np
from collections import deque
from kevin_components import Robot
import sys
sys.path.insert(0, '/home/riley/ros_projects/mobile_robot_ws/src/robot_controller/robot_controller/BerryIMU/')
import IMU

# Constants and Initialization
G_GAIN = 0.070  # Gyroscope gain
WHEEL_BASE = 0.158  # Distance between wheels in meters (change as per your robot)
PULSES_PER_REV = 40  # Pulses per revolution from encoders
WHEEL_RADIUS = 0.066  # Radius of the wheel
ALPHA = 0.5  # Complementary filter constant

# Initialize the IMU

IMU.detectIMU()     #Detect if BerryIMU is connected.
if(IMU.BerryIMUversion == 99):
    print(" No BerryIMU found... exiting ")
    sys.exit()
IMU.initIMU() 

# Initialize GPIO and Robot
GPIO.setmode(GPIO.BOARD)
left_encoder = 36
right_encoder = 38
GPIO.setup(left_encoder, GPIO.IN)
GPIO.setup(right_encoder, GPIO.IN)
robot = Robot(25)  # Assuming 25 is the target power

# Helper function to calculate wheel speed
def calculate_speed(pulse_count, delta_time):
    return (pulse_count / PULSES_PER_REV) / delta_time * (2 * np.pi * WHEEL_RADIUS)

# Moving average window for encoder data
encoder_window_size = 10
encoder_data = deque(maxlen=encoder_window_size)

# Main control loop
try:
    left_pulse_count = 0
    right_pulse_count = 0
    last_left_state = GPIO.input(left_encoder)
    last_right_state = GPIO.input(right_encoder)
    last_time = time.time()

    while True:
        # Read gyroscope data
        rate_gyr_z = IMU.readGYRz() * G_GAIN #degrees per second
        #convert to rad/s
        rate_gyr_z = rate_gyr_z * np.pi/180


        # Read current state of wheel encoders
        current_left_state = GPIO.input(left_encoder)
        current_right_state = GPIO.input(right_encoder)

        # Update pulse counts on state change
        if current_left_state != last_left_state:
            left_pulse_count += 1
            last_left_state = current_left_state

        if current_right_state != last_right_state:
            right_pulse_count += 1
            last_right_state = current_right_state

        current_time = time.time()
        delta_time = current_time - last_time

        if delta_time >= 1:  # Update every x ms
            # Calculate wheel speeds and add to moving average deque
            left_speed = calculate_speed(left_pulse_count, delta_time)
            right_speed = calculate_speed(right_pulse_count, delta_time)
            rotational_velocity_wheel = (right_speed - left_speed) / WHEEL_BASE
            encoder_data.append(rotational_velocity_wheel)
            avg_rotational_velocity_wheel = np.mean(encoder_data)


            # Apply complementary filter to the averaged values
            filtered_velocity = ALPHA * rate_gyr_z + (1 - ALPHA) * avg_rotational_velocity_wheel

            # Print velocities for debugging
            print(filtered_velocity, rate_gyr_z, avg_rotational_velocity_wheel)

            # Reset for next iteration
            left_pulse_count = 0
            right_pulse_count = 0
            last_time = current_time

except KeyboardInterrupt: 
    print("Program stopped by user")
finally:
    # Clean up
    robot.cleanup()
    GPIO.cleanup()

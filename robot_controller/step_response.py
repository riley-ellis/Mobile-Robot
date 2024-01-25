import RPi.GPIO as GPIO
import time
import numpy as np
import csv
from kevin_components import Robot
from collections import deque

GPIO.setmode(GPIO.BOARD)

# Encoder pins
left_encoder = 36
right_encoder = 38

# Set up the pins
GPIO.setup(left_encoder, GPIO.IN)
GPIO.setup(right_encoder, GPIO.IN)

# Wheel parameters
wheel_radius = 0.066  # meters
pulses_per_rev = 40  # 40 half pulses per revolution

# Initialize the robot
target_power = 25 # Approximate power level for the motors
robot = Robot(target_power)

# Variables to keep track of pulses and time
left_pulse_count = 0
right_pulse_count = 0
last_left_state = 0
last_right_state = 0

# Moving average window
window_size = 10
speed_diffs = deque(maxlen=window_size)

# CSV file setup
csv_file = open('robot_speed_data.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Time', 'Left Speed', 'Right Speed', 'Speed Difference', 'Moving Average Difference'])

# Apply specific PWM signals and record data
robot.skid(-0.25, 0.25)  # Positive for left motor, negative for right motor
start_time = time.time()
last_time = start_time

# Main loop for recording the speed
try:
    while time.time() - start_time < 3:  # Record for 3 seconds
        current_time = time.time()
        delta_time = current_time - last_time

        if delta_time >= 0.01:  # Update every 10 ms
            # Read encoder states
            left_state = GPIO.input(left_encoder)
            right_state = GPIO.input(right_encoder)

            # Update pulse count
            if left_state != last_left_state:
                left_pulse_count += 1
                last_left_state = left_state

            if right_state != last_right_state:
                right_pulse_count += 1
                last_right_state = right_state

            # Calculate wheel speeds (rotations per second)
            left_speed = (left_pulse_count / pulses_per_rev) / delta_time
            right_speed = (right_pulse_count / pulses_per_rev) / delta_time

            # Calculate speed difference
            speed_diff = left_speed - right_speed

            # Add to moving average window
            speed_diffs.append(speed_diff)
            avg_speed_diff = np.mean(speed_diffs)

            # Write data to CSV
            csv_writer.writerow([current_time - start_time, left_speed, right_speed, speed_diff, avg_speed_diff])

            # Reset pulse counts and time
            left_pulse_count = 0
            right_pulse_count = 0
            last_time = current_time

except KeyboardInterrupt: 
    print("Program stopped by user")
finally:
    # Stop the robot and clean up
    robot.skid(0, 0)
    csv_file.close()
    GPIO.cleanup()

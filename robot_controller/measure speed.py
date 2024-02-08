import RPi.GPIO as GPIO
import time
import numpy as np
from kevin_components import Robot
from collections import deque

#this script will meausre rad/s of each motor and calibrate them to same speed

GPIO.setmode(GPIO.BOARD)

# Encoder pins
left_encoder = 36
right_encoder = 37

# Set up the pins
GPIO.setup(left_encoder, GPIO.IN)
GPIO.setup(right_encoder, GPIO.IN)

# Wheel parameters
wheel_radius = 0.066  # meters
pulses_per_rev = 40  # 40 half pulses per revolution
#wheel_circumference = 2 * np.pi * wheel_radius #this is not needed atm

#initialize encoder counts
left_count = 0
right_count = 0

#initialize timers
last_pulse_time_left = time.time()
last_pulse_time_right = time.time()

#initialize angular velocities
left_angular_velocity = None
right_angular_velocity = None
avg_left_angular_velocity = deque(maxlen=20)
avg_right_angular_velocity = deque(maxlen=20)

def measure_speed(delta_time, pulse_count): #speed in radians per second...
    angular_velocity = (pulse_count/delta_time)*(2*np.pi/pulses_per_rev)
    return angular_velocity

def left_encoder_callback(channel):
    global left_count, last_pulse_time_left, left_angular_velocity, avg_left_angular_velocity, desired_pulse_count
    left_count += 1
    current_time = time.time()
    if left_count % desired_pulse_count == 0:
         delta_time = current_time - last_pulse_time_left
         left_angular_velocity = measure_speed(delta_time, desired_pulse_count)
        #  print('left ang vel: ', left_angular_velocity)
         left_ang_vels.append(left_angular_velocity)
         avg_left_angular_velocity.append(left_angular_velocity)
        #  print('avg left ang vel: ', np.mean(avg_left_angular_velocity))
         last_pulse_time_left = current_time

def right_encoder_callback(channel):
    global right_count, last_pulse_time_right, right_angular_velocity, avg_right_angular_velocity, desired_pulse_count
    right_count += 1
    current_time = time.time()
    if right_count % desired_pulse_count == 0:
         delta_time = current_time - last_pulse_time_right
         right_angular_velocity = measure_speed(delta_time, desired_pulse_count)
        #  print('right ang vel', right_angular_velocity)
         right_ang_vels.append(right_angular_velocity)
         avg_right_angular_velocity.append(right_angular_velocity)
        #  print('avg right ang vel: ', np.mean(avg_right_angular_velocity))
         last_pulse_time_right = current_time

# target_rev_per_sec = 10.5

#add event detection
GPIO.add_event_detect(left_encoder, GPIO.BOTH, callback=left_encoder_callback)
GPIO.add_event_detect(right_encoder, GPIO.BOTH, callback=right_encoder_callback)

# Initialize last states for encoders
last_left_state = GPIO.input(left_encoder)
last_right_state = GPIO.input(right_encoder)

#keep track of angular velocities in lists
left_ang_vels = []
right_ang_vels = []

#set desired pulse count for speed measuring resolution
#i think this may affect noise too
desired_pulse_count = 1


robot = Robot()

left_speed_cmd = 0
right_speed_cmd = 1


try:
    robot.skid(left_speed_cmd, right_speed_cmd)
    time.sleep(.5)
    last_time_left = time.time()
    last_time_right = time.time()
    while True:
        loop_start_time = time.time()

        #implement PIDs
        #left
        current_time_left = time.time()
        delta_time_left = current_time_left - last_time_left
        last_time_left = current_time_left
        
        #right
        current_time_right = time.time()
        delta_time_right = current_time_right - last_time_right
        last_time_right = current_time_right

        robot.skid(left_speed_cmd, right_speed_cmd)

        print(np.mean(avg_right_angular_velocity))

        #try to get loop to run as close to .1 sec as possible
        loop_end_time = time.time()
        # print('loop time: ', loop_end_time-loop_start_time)
        time.sleep(max(0, 0.1- (loop_end_time-loop_start_time)))


except KeyboardInterrupt:
    # Handle any cleanup here
    print('left ang vel: ', np.mean(avg_left_angular_velocity))
    print('right ang vel: ', np.mean(avg_right_angular_velocity))
    robot.cleanup()
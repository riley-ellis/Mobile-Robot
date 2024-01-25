import RPi.GPIO as GPIO
import time
import numpy as np
from kevin_components import Robot
from collections import deque
import matplotlib.pyplot as plt

#this script will meausre rad/s of each motor and calibrate them to same speed

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, delta_time):
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

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

target_rev_per_sec = 10.5

#for graphing:
left_ang_vels_graph = deque([target_rev_per_sec]*20, maxlen=20)
right_ang_vels_graph = deque([target_rev_per_sec]*20, maxlen=20)

#initialize plot
initial_x_data = np.linspace(target_rev_per_sec-5, target_rev_per_sec+5, 20)
initial_left_y_data = list(left_ang_vels_graph)
initial_right_y_data = list(right_ang_vels_graph)

plt.ion()  # Interactive mode on
fig, ax = plt.subplots()
lines_left, = ax.plot(initial_x_data, initial_left_y_data, label='Left Motor')
lines_right, = ax.plot(initial_x_data, initial_right_y_data, label='Right Motor')
ax.set_ylim(target_rev_per_sec-5, target_rev_per_sec+5)
ax.set_xlim(-1, 1)
plt.axhline(y=target_rev_per_sec, color='r', linestyle='-', label='Setpoint')
plt.legend(loc='upper left')

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

#initialize pids
kp_left = .005
ki_left = 0
kd_left = 0

kp_right = .005
ki_right = 0
kd_right = 0

pid_left = PIDController(kp_left, ki_left, kd_left)
pid_right = PIDController(kp_right, ki_right, kd_right)




robot = Robot()

left_speed_cmd = 0.22
right_speed_cmd = 0.22

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
        left_error = target_rev_per_sec-np.mean(avg_left_angular_velocity)
        print('left error: ', left_error)
        left_control = pid_left.update(left_error, delta_time_left)
        print('left control: ', left_control)
        left_speed_cmd = np.clip(left_speed_cmd + left_control, -1, 1)
        print('left_speed_cmd: ', left_speed_cmd)
        last_time_left = current_time_left
        
        #right
        current_time_right = time.time()
        delta_time_right = current_time_right - last_time_right
        right_error = target_rev_per_sec-np.mean(avg_right_angular_velocity)
        print('right error: ', right_error)
        right_control = pid_right.update(right_error, delta_time_right)
        print('right_control: ', right_control)
        right_speed_cmd = np.clip(right_speed_cmd + right_control, -1, 1)
        print('right_speed_cmd: ', right_speed_cmd)
        last_time_right = current_time_right

        robot.skid(left_speed_cmd, right_speed_cmd)
        

        #graphing PID

        #update graphing points
        left_ang_vels_graph.append(np.mean(avg_left_angular_velocity))
        right_ang_vels_graph.append(np.mean(avg_right_angular_velocity))
        #update plot data
        lines_left.set_xdata(np.linspace(-1, 1, 20))
        lines_left.set_ydata(list(left_ang_vels_graph))
        lines_right.set_xdata(np.linspace(-1, 1, 20))
        lines_right.set_ydata(list(right_ang_vels_graph))
        #redraw plot
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.01)

        #try to get loop to run as close to .1 sec as possible
        loop_end_time = time.time()
        print('loop time: ', loop_end_time-loop_start_time)
        time.sleep(max(0, 0.1- (loop_end_time-loop_start_time)))


except KeyboardInterrupt:
    # Handle any cleanup here
    print('left ang vel: ', np.mean(avg_left_angular_velocity))
    print('right ang vel: ', np.mean(avg_right_angular_velocity))
    robot.cleanup()
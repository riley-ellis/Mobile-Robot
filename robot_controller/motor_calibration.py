import RPi.GPIO as GPIO
import time
import numpy as np
from kevin_components import Robot
from collections import deque
import csv
from scipy.signal import butter, lfilter, lfilter_zi

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
        # print('derivative ', derivative)

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

class RealTimeLowPassFilter:
    def __init__(self, cutoff_frequency, sampling_rate, order=5):
        self.cutoff_frequency = cutoff_frequency
        self.sampling_rate = sampling_rate
        self.order = order
        self.nyquist_frequency = 0.5 * sampling_rate
        self.b, self.a = butter(self.order, self.cutoff_frequency / self.nyquist_frequency, btype='low', analog=False)
        self.zi = lfilter_zi(self.b, self.a) * 0  # Initialize the filter state to zero

    def apply_filter(self, data):
        filtered_data, self.zi = lfilter(self.b, self.a, [data], zi=self.zi)
        return filtered_data[0]

GPIO.setmode(GPIO.BOARD)

csv_file = open('vel_filter.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['dt', 'speed', 'filtered speed'])

#implement butterworth low pass filter with 3 Hz cuttoff frequency to filter out high frequency noise
sampling_rate_left = 1/0.0401 #0.0201 is delta_time_left
cutoff_frequency_hz = 3

#create filter
lp_filter = RealTimeLowPassFilter(cutoff_frequency_hz, sampling_rate_left)

# Encoder pins

left_encoder = 36
right_encoder = 38

# Set up the pins
GPIO.setup(left_encoder, GPIO.IN)
GPIO.setup(right_encoder, GPIO.IN)

# Wheel parameters
wheel_radius = 0.066  # meters
pulses_per_rev = 20 #full pulses per revolution

#initialize encoder counts
left_count = 0
right_count = 0

#DO I NEED THESE?
#initialize angular velocities
left_angular_velocity = None
right_angular_velocity = None
avg_left_angular_velocity = deque(maxlen=20)
avg_right_angular_velocity = deque(maxlen=20)

def measure_speed(delta_time, pulse_count): #speed in radians per second...
    angular_velocity = (pulse_count/delta_time)*(2*np.pi/pulses_per_rev)
    return angular_velocity

def left_encoder_callback(channel):
    global left_count
    left_count += 1
    print(left_count)

def right_encoder_callback(channel):
    global right_count
    right_count += 1

target_rev_per_sec = 30

#add event detection
GPIO.add_event_detect(left_encoder, GPIO.FALLING, callback=left_encoder_callback)
GPIO.add_event_detect(right_encoder, GPIO.FALLING, callback=right_encoder_callback)

#initialize pids
kp_left = .00065
ki_left = .00005
kd_left = 0.00017

kp_right = .00065
ki_right = .000
kd_right = 0.000

pid_left = PIDController(kp_left, ki_left, kd_left)
pid_right = PIDController(kp_right, ki_right, kd_right)

robot = Robot()


#free floating PID tuning
left_speed_cmd = 0
right_speed_cmd = 0 #keeping at zero for purpose of tuning just the left PID first

loop_aim_time = .04
ct = 0

try:
    robot.skid(left_speed_cmd, right_speed_cmd)
    time.sleep(1)
    prev_time_left = time.time()
    prev_time_right = time.time()
    condition = True
    while condition:
        loop_start_time = time.time()
        if (ct>50):
            condition = False

        #implement PIDs
        #left
        current_time_left = time.time()
        delta_time_left = current_time_left - prev_time_left

        if delta_time_left < .012:
            left_count=0
            continue
        print('left_count: ', left_count)
        left_speed = measure_speed(delta_time_left, left_count)
        print('left: , ', left_speed)
        filtered_left_speed = lp_filter.apply_filter(left_speed)
        # print('filtered_left: ', filtered_left_speed)

        csv_writer.writerow([delta_time_left, left_speed, filtered_left_speed])

        left_error = target_rev_per_sec-filtered_left_speed
        left_control = pid_left.update(left_error, delta_time_left)
        left_speed_cmd = np.clip(left_speed_cmd + left_control, -1, 1)

        left_count = 0
        prev_time_left = current_time_left
        
        #right
            
        current_time_right = time.time()
        delta_time_right = current_time_right - prev_time_right
        # print(delta_time_right)
        if delta_time_right < .012:
            right_count = 0
            continue

        right_speed = measure_speed(delta_time_right, right_count)
        filtered_right_speed = lp_filter.apply_filter(right_speed)

        # csv_writer.writerow([delta_time_right, right_speed, filtered_right_speed])
        
        right_error = target_rev_per_sec - filtered_right_speed
        right_control = pid_right.update(right_error, delta_time_right)
        right_speed_cmd = np.clip(right_speed_cmd + right_control, -1, 1)

        right_count = 0
        prev_time_right = current_time_right
        #comment out for PID tuning
        robot.skid(left_speed_cmd, right_speed_cmd)
        ct+=1
        #try to get loop to run as close to .1 sec as possible
        loop_end_time = time.time()
        # # print('loop time: ', loop_end_time-loop_start_time)
        time.sleep(max(0, loop_aim_time- (loop_end_time-loop_start_time)))
    csv_file.close()
    robot.cleanup()

except KeyboardInterrupt:
    # Handle any cleanup here
    # print('left ang vel: ', np.mean(avg_left_angular_velocity))
    # print('right ang vel: ', np.mean(avg_right_angular_velocity))
    csv_file.close()
    robot.cleanup()
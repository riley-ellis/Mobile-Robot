import RPi.GPIO as GPIO
import time
import numpy as np
from kevin_components import Robot
import csv

GPIO.setmode(GPIO.BOARD)

# Encoder pins
left_encoder = 36

# Set up the pins
GPIO.setup(left_encoder, GPIO.IN)

# Wheel parameters
wheel_radius = 0.066  # meters
pulses_per_rev = 20  # 20 full pulses per revolution

left_count = 0

def measure_speed(delta_time, pulse_count): #speed in radians per second...
    angular_velocity = (pulse_count/delta_time)*(2*np.pi/pulses_per_rev)
    return angular_velocity

def left_encoder_callback(channel):
    global left_count
    left_count +=1

GPIO.add_event_detect(left_encoder, GPIO.FALLING, callback=left_encoder_callback)

csv_file = open('vel_method_1.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['dt', 'speed'])


#initialize robot
robot=Robot()


try:
    robot.skid(.3, 0)
    #let it get up to speed
    time.sleep(1)
    prev_time = time.time()
    ct = 0
    while True:
        time.sleep(.02)
        if ct>250:
            robot.skid(.2, 0)
            ct = 0
        current_time = time.time()
        dt = current_time-prev_time
        angular_velocity = measure_speed(dt, left_count)

        csv_writer.writerow([dt, angular_velocity])

        print(angular_velocity)

        left_count = 0
        prev_time = current_time

        ct+=1
        print(ct)

except KeyboardInterrupt:
    csv_file.close()
    robot.cleanup()


#scp riley@104.194.125.114:/home/riley/ros_projects/mobile_robot_ws/src/robot_controller/robot_controller/vel_method_1.csv "C:\Users\Riley\Desktop\Viri\VelocityFilter"
    
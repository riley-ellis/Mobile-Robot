import RPi.GPIO as GPIO

#i would rather use GPIO.BOARD for simplicity, figure out how to label them
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

class Motor:

    def __init__(self, in1, in2, enable, max_speed=100):
        self.in1 = in1
        self.in2 = in2
        self.enable = enable

        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        GPIO.setup(enable, GPIO.OUT)

        self.max_speed = max_speed #between 0 and 100 (it's a percent)
        self.pwm = GPIO.PWM(self.enable, 1000) #setting to 1KHz for now
        self.pwm.start(0)

    def forward(self):
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)

    def backward(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)

    def set_speed(self, speed):
        #speed is essentially a pwm value
        #its in the range [-self.max_speed, self.max_speed]
        speed = max(min(speed, round(self.max_speed)), -round(self.max_speed))
        if speed > 0:
            self.forward()
        elif speed < 0:
            self.backward()
            speed = abs(speed)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)

        self.pwm.ChangeDutyCycle(speed)

    def stop(self):
        self.pwm.stop()
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)

    def start(self):
        self.pwm.start(0)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup([self.in1, self.in2, self.enable])

class Robot:

    def __init__(self, max_motor_power=100):
        #define a max motor speed (must be between 0 and 100)
        self.max_motor_power = max_motor_power #this is a percentage between 1 and 100%
        self.left_motor = Motor(11, 13, 15, self.max_motor_power)
        self.right_motor = Motor(18, 16, 12, self.max_motor_power)

    def skid(self, left_speed, right_speed):
        #left_speed and right_speed are expected to be in range [-1.0, 1.0]
        left_speed_cmd = left_speed * 100
        right_speed_cmd = right_speed * 100
        #these set speeds are in range [-100, 100]
        self.left_motor.set_speed(left_speed_cmd)
        self.right_motor.set_speed(right_speed_cmd)

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def start(self):
        self.left_motor.start()
        self.right_motor.start()

    def cleanup(self):
        self.left_motor.cleanup()
        self.right_motor.cleanup()

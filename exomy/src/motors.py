import rospy
from std_msgs.msg import String

import RPi.GPIO as GPIO
import time
import numpy as np

class Motors():
    '''
    Motors class contains all functions to control the steering and driving motors.
    ''' 

    # Define wheel names
    # rr- -rl
    # cr- -cl
    #    |
    # fr- -fl
    FL, FR, CL, CR, RL, RR = range(0, 6)

    def __init__(self):
        # Set the GPIO modes
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        # Set variables for the GPIO motor pins

        pin_drive_fr =12 
        pin_steer_fr =11 


        # Set the GPIO Pin mode to be Output
        GPIO.setup(pin_drive_fr, GPIO.OUT)
        GPIO.setup(pin_steer_fr, GPIO.OUT)
                
        # PWM characteristics 
        pwm_frequency = 50 		# Hz

        self.steering_pwm_low_limit = 2.5#5 	# %
        self.steering_pwm_neutral = 6.5 			# %
        self.steering_pwm_upper_limit = 10.5#10	# %
        self.steering_pwm_range = 4.0

        self.driving_pwm_low_limit = 3.0#5 	# %
        self.driving_pwm_neutral = 7.0 			# %
        self.driving_pwm_upper_limit = 11.0#10	# %
        self.driving_pwm_range = 4.0

        # Set the GPIO to software PWM at 'Frequency' Hertz
        self.driving_motors = [None] * 6
        self.steering_motors = [None] * 6
        self.driving_motors[self.FR] = GPIO.PWM(pin_drive_fr, pwm_frequency)
        self.steering_motors[self.FR] = GPIO.PWM(pin_steer_fr, pwm_frequency)

        for motor in self.driving_motors:
            if motor is not None:
                motor.start(self.driving_pwm_neutral)
        for motor in self.steering_motors:
            if motor is not None:
                motor.start(self.steering_pwm_neutral)

    def maybe(self):
        #for speed in range(FullBack*100, FullForward*100, 25):
        #	pwm_drive_fr.start(speed/100)
        #	time.sleep(1)

        angles = np.linspace(pwm_low_limit, pwm_upper_limit, 5)
        pwm_steer_fr.start(pwm_stop)
        time.sleep(1)


        for angle in angles:
                pwm_steer_fr.ChangeDutyCycle(angle)	
                print(angle)
                time.sleep(1)

    def setSteering(self, steering_command):
        duty_cycle = self.steering_pwm_neutral + steering_command[self.FR]/100* self.steering_pwm_range 
        self.steering_motors[self.FR].ChangeDutyCycle(duty_cycle)
            
    def setDriving(self, driving_command):
#            duty_cycle = self.pwm_neutral + driving_command[self.FL]/100 * self.pwm_range 
#            self.driving_motors[self.FL].ChangeDutyCycle(duty_cycle)

        duty_cycle = self.driving_pwm_neutral + driving_command[self.FR]/100 * self.driving_pwm_range 
        self.driving_motors[self.FR].ChangeDutyCycle(duty_cycle)

#            duty_cycle = self.pwm_neutral + driving_command[self.CL]/100 * self.pwm_range 
#            self.driving_motors[self.CL].ChangeDutyCycle(duty_cycle)

#            duty_cycle = self.pwm_neutral + driving_command[self.CR]/100 * self.pwm_range 
#            self.driving_motors[self.CR].ChangeDutyCycle(duty_cycle)

#            duty_cycle = self.pwm_neutral + driving_command[self.RL]/100 * self.pwm_range 
#            self.driving_motors[self.RL].ChangeDutyCycle(duty_cycle)

#            duty_cycle = self.pwm_neutral + driving_command[self.RR]/100 * self.pwm_range 
#            self.driving_motors[self.RR].ChangeDutyCycle(duty_cycle)

    def stopMotors(self):
        for motor in self.driving_motors:
            if motor is not None:
                motor.ChangeDutyCycle(self.driving_pwm_neutral)

    def cleanup(self):
        GPIO.cleanup()

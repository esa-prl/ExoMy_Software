#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import time
import numpy as np

import Adafruit_PCA9685


class Motors():
    '''
    Motors class contains all functions to control the steering and driving
    '''

    # Define wheel names
    FL, FR, CL, CR, RL, RR = range(0, 6)

    # Motor commands are assuming positiv=driving_forward, negative=driving_backwards.
    # The driving direction of the left side has to be inverted for this to apply to all wheels.
    wheel_directions = [-1, 1, -1, 1, -1, 1]

    # 1 fl-||-fr 2
    #      ||
    # 3 cl-||-cr 4
    # 5 rl====rr 6

    def __init__(self):

        # Dictionary containing the pins of all motors
        self.pins = {
            'drive': {},
            'steer': {}
        }

        # Set variables for the GPIO motor pins
        self.pins['drive'][self.FL] = rospy.get_param("pin_drive_fl")
        self.pins['steer'][self.FL] = rospy.get_param("pin_steer_fl")

        self.pins['drive'][self.FR] = rospy.get_param("pin_drive_fr")
        self.pins['steer'][self.FR] = rospy.get_param("pin_steer_fr")

        self.pins['drive'][self.CL] = rospy.get_param("pin_drive_cl")
        self.pins['steer'][self.CL] = rospy.get_param("pin_steer_cl")

        self.pins['drive'][self.CR] = rospy.get_param("pin_drive_cr")
        self.pins['steer'][self.CR] = rospy.get_param("pin_steer_cr")

        self.pins['drive'][self.RL] = rospy.get_param("pin_drive_rl")
        self.pins['steer'][self.RL] = rospy.get_param("pin_steer_rl")

        self.pins['drive'][self.RR] = rospy.get_param("pin_drive_rr")
        self.pins['steer'][self.RR] = rospy.get_param("pin_steer_rr")

        # PWM characteristics
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)  # Hz

        self.steering_pwm_neutral = [None] * 6

        self.steering_pwm_neutral[self.FL] = rospy.get_param("steer_pwm_neutral_fl")
        self.steering_pwm_neutral[self.FR] = rospy.get_param("steer_pwm_neutral_fr")
        self.steering_pwm_neutral[self.CL] = rospy.get_param("steer_pwm_neutral_cl")
        self.steering_pwm_neutral[self.CR] = rospy.get_param("steer_pwm_neutral_cr")
        self.steering_pwm_neutral[self.RL] = rospy.get_param("steer_pwm_neutral_rl")
        self.steering_pwm_neutral[self.RR] = rospy.get_param("steer_pwm_neutral_rr")
        self.steering_pwm_range = rospy.get_param("steer_pwm_range")

        self.driving_pwm_low_limit = 100
        self.driving_pwm_neutral = rospy.get_param("drive_pwm_neutral")
        self.driving_pwm_upper_limit = 500
        self.driving_pwm_range = rospy.get_param("drive_pwm_range")

        # Set steering motors to neutral values (straight)
        for wheel_name, motor_pin in self.pins['steer'].items():
            self.pwm.set_pwm(motor_pin, 0,
                             self.steering_pwm_neutral[wheel_name])
            time.sleep(0.1)

        self.wiggle()

    def wiggle(self):
        time.sleep(0.1)
        self.pwm.set_pwm(self.pins['steer'][self.FL], 0,
                         int(self.steering_pwm_neutral[self.FL] + self.steering_pwm_range * 0.3))
        time.sleep(0.1)
        self.pwm.set_pwm(self.pins['steer'][self.FR], 0,
                         int(self.steering_pwm_neutral[self.FR] + self.steering_pwm_range * 0.3))
        time.sleep(0.3)
        self.pwm.set_pwm(self.pins['steer'][self.FL], 0,
                         int(self.steering_pwm_neutral[self.FL] - self.steering_pwm_range * 0.3))
        time.sleep(0.1)
        self.pwm.set_pwm(self.pins['steer'][self.FR], 0,
                         int(self.steering_pwm_neutral[self.FR] - self.steering_pwm_range * 0.3))
        time.sleep(0.3)
        self.pwm.set_pwm(self.pins['steer'][self.FL], 0,
                         int(self.steering_pwm_neutral[self.FL]))
        time.sleep(0.1)
        self.pwm.set_pwm(self.pins['steer'][self.FR], 0,
                         int(self.steering_pwm_neutral[self.FR]))
        time.sleep(0.3)

    def setSteering(self, steering_command):
        # Loop through pin dictionary. The items key is the wheel_name and the value the pin.
        for wheel_name, motor_pin in self.pins['steer'].items():
            duty_cycle = int(
                self.steering_pwm_neutral[wheel_name] + steering_command[wheel_name]/90.0 * self.steering_pwm_range)

            self.pwm.set_pwm(motor_pin, 0, duty_cycle)

    def setDriving(self, driving_command):
        # Loop through pin dictionary. The items key is the wheel_name and the value the pin.
        for wheel_name, motor_pin in self.pins['drive'].items():
            duty_cycle = int(self.driving_pwm_neutral +
                             driving_command[wheel_name]/100.0 * self.driving_pwm_range * self.wheel_directions[wheel_name])

            self.pwm.set_pwm(motor_pin, 0, duty_cycle)

    def stopMotors(self):
        # Set driving wheels to neutral position to stop them
        duty_cycle = int(self.driving_pwm_neutral)

        for wheel_name, motor_pin in self.pins['drive'].items():
            self.pwm.set_pwm(motor_pin, 0, duty_cycle)

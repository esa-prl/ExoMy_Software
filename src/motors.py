#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import time
import numpy as np

import Adafruit_PCA9685


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

        # Set variables for the GPIO motor pins
        self.pin_drive_fl = rospy.get_param("pin_drive_fl")
        self.pin_steer_fl = rospy.get_param("pin_steer_fl")

        self.pin_drive_fr = rospy.get_param("pin_drive_fr")
        self.pin_steer_fr = rospy.get_param("pin_steer_fr")

        self.pin_drive_cl = rospy.get_param("pin_drive_cl")
        self.pin_steer_cl = rospy.get_param("pin_steer_cl")

        self.pin_drive_cr =rospy.get_param("pin_drive_cr")
        self.pin_steer_cr =rospy.get_param("pin_steer_cr")

        self.pin_drive_rl =rospy.get_param("pin_drive_rl")
        self.pin_steer_rl =rospy.get_param("pin_steer_rl")

        self.pin_drive_rr =rospy.get_param("pin_drive_rr")
        self.pin_steer_rr = rospy.get_param("pin_steer_rr")

        # PWM characteristics
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)  # Hz

        self.steering_pwm_neutral = [None] * 6

        self.steering_pwm_neutral[self.FL] = rospy.get_param("steer_offset_fl")
        self.steering_pwm_neutral[self.FR] = rospy.get_param("steer_offset_fr")
        self.steering_pwm_neutral[self.CL] = rospy.get_param("steer_offset_cl")
        self.steering_pwm_neutral[self.CR] = rospy.get_param("steer_offset_cr")
        self.steering_pwm_neutral[self.RL] = rospy.get_param("steer_offset_rl")
        self.steering_pwm_neutral[self.RR] = rospy.get_param("steer_offset_rr")
        self.steering_pwm_range = 200

        self.driving_pwm_low_limit = 100
        self.driving_pwm_neutral = 300
        self.driving_pwm_upper_limit = 500
        self.driving_pwm_range = 200

        # Set the GPIO to software PWM at 'Frequency' Hertz
        self.driving_motors = [None] * 6
        self.steering_motors = [None] * 6

        # Set steering motors to neutral values
        self.pwm.set_pwm(self.pin_steer_fl, 0,
                         self.steering_pwm_neutral[self.FL])
        time.sleep(0.1)
        self.pwm.set_pwm(self.pin_steer_fr, 0,
                         self.steering_pwm_neutral[self.FR])
        time.sleep(0.1)
        self.pwm.set_pwm(self.pin_steer_cl, 0,
                         self.steering_pwm_neutral[self.CL])
        time.sleep(0.1)
        self.pwm.set_pwm(self.pin_steer_cr, 0,
                         self.steering_pwm_neutral[self.CR])
        time.sleep(0.1)
        self.pwm.set_pwm(self.pin_steer_rl, 0,
                         self.steering_pwm_neutral[self.RL])
        time.sleep(0.1)
        self.pwm.set_pwm(self.pin_steer_rr, 0,
                         self.steering_pwm_neutral[self.RR])
        time.sleep(0.1)

    def setSteering(self, steering_command):
        duty_cycle = int(
            self.steering_pwm_neutral[self.FL] + steering_command[self.FL]/90.0 * self.steering_pwm_range)
        self.pwm.set_pwm(self.pin_steer_fl, 0, duty_cycle)

        duty_cycle = int(
            self.steering_pwm_neutral[self.FR] + steering_command[self.FR]/90.0 * self.steering_pwm_range)
        self.pwm.set_pwm(self.pin_steer_fr, 0, duty_cycle)

        duty_cycle = int(
            self.steering_pwm_neutral[self.CL] + steering_command[self.CL]/90.0 * self.steering_pwm_range)
        self.pwm.set_pwm(self.pin_steer_cl, 0, duty_cycle)

        duty_cycle = int(
            self.steering_pwm_neutral[self.CR] + steering_command[self.CR]/90.0 * self.steering_pwm_range)
        self.pwm.set_pwm(self.pin_steer_cr, 0, duty_cycle)

        duty_cycle = int(
            self.steering_pwm_neutral[self.RL] + steering_command[self.RL]/90.0 * self.steering_pwm_range)
        self.pwm.set_pwm(self.pin_steer_rl, 0, duty_cycle)

        duty_cycle = int(
            self.steering_pwm_neutral[self.RR] + steering_command[self.RR]/90.0 * self.steering_pwm_range)
        self.pwm.set_pwm(self.pin_steer_rr, 0, duty_cycle)

    def setDriving(self, driving_command):
        if(driving_command[self.FL] == 0.0):
            duty_cycle = 0
        else:
            duty_cycle = int(self.driving_pwm_neutral -
                             driving_command[self.FL]/100.0 * self.driving_pwm_range)
        self.pwm.set_pwm(self.pin_drive_fl, 0, duty_cycle)

        if(driving_command[self.FR] == 0.0):
            duty_cycle = 0
        else:
            duty_cycle = int(self.driving_pwm_neutral +
                             driving_command[self.FR]/100.0 * self.driving_pwm_range)
        self.pwm.set_pwm(self.pin_drive_fr, 0, duty_cycle)

        if(driving_command[self.CL] == 0.0):
            duty_cycle = 0
        else:
            duty_cycle = int(self.driving_pwm_neutral -
                             driving_command[self.CL]/100.0 * self.driving_pwm_range)
        self.pwm.set_pwm(self.pin_drive_cl, 0, duty_cycle)

        if(driving_command[self.CR] == 0.0):
            duty_cycle = 0
        else:
            duty_cycle = int(self.driving_pwm_neutral +
                             driving_command[self.CR]/100.0 * self.driving_pwm_range)
        self.pwm.set_pwm(self.pin_drive_cr, 0, duty_cycle)

        if(driving_command[self.RL] == 0.0):
            duty_cycle = 0
        else:
            duty_cycle = int(self.driving_pwm_neutral -
                             driving_command[self.RL]/100.0 * self.driving_pwm_range)
        self.pwm.set_pwm(self.pin_drive_rl, 0, duty_cycle)

        if(driving_command[self.RR] == 0.0):
            duty_cycle = 0
        else:
            duty_cycle = int(self.driving_pwm_neutral +
                             driving_command[self.RR]/100.0 * self.driving_pwm_range)
        self.pwm.set_pwm(self.pin_drive_rr, 0, duty_cycle)

    def stopMotors(self):
        self.pwm.set_pwm(self.pin_drive_fl, 0, 0)
        self.pwm.set_pwm(self.pin_drive_fr, 0, 0)
        self.pwm.set_pwm(self.pin_drive_cl, 0, 0)
        self.pwm.set_pwm(self.pin_drive_cr, 0, 0)
        self.pwm.set_pwm(self.pin_drive_rl, 0, 0)
        self.pwm.set_pwm(self.pin_drive_rr, 0, 0)

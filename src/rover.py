#!/usr/bin/env python
import rospy
import time
import math
import enum
from locomotion_modes import LocomotionMode
import numpy as np


class Rover():
    '''
    Rover class contains all the math and motor control algorithms to move the rover
    '''

    # Defining wheel names
    FL, FR, CL, CR, RL, RR = range(0, 6)

    # Defining locomotion modes
    FAKE_ACKERMANN, ACKERMANN, POINT_TURN, CRABBING = range(0, 4)

    def __init__(self):
        self.locomotion_mode = LocomotionMode.FAKE_ACKERMANN

        self.wheel_x = 12.0
        self.wheel_y = 20.0

        max_steering_angle = 45
        self.ackermann_r_min = abs(
            self.wheel_y) / math.tan(max_steering_angle * math.pi / 180.0) + self.wheel_x

        self.ackermann_r_max = 250

    def setLocomotionMode(self, locomotion_mode_command):
        '''
        Sets the locomotion mode
        '''
        if(self.locomotion_mode != locomotion_mode_command):
            self.locomotion_mode = locomotion_mode_command
            rospy.loginfo('Set locomotion mode to: %s',
                          LocomotionMode(locomotion_mode_command).name)

    def joystickToSteeringAngle(self, driving_command, steering_command):
        '''
        Converts the steering command [angle of joystick] to angles for the different motors

        :param int driving_command: Drive speed command range from -100 to 100
        :param int stering_command: Turning radius command with the values 0(left) +90(forward) -90(backward)  +-180(right)
        '''

        steering_angles = [0]*6
        deg = steering_command

        if(self.locomotion_mode == LocomotionMode.FAKE_ACKERMANN.value):
            if (driving_command == 0):
                # Stop
                steering_angles[self.FL] = 0
                steering_angles[self.FR] = 0
                steering_angles[self.CR] = 0
                steering_angles[self.CL] = 0
                steering_angles[self.RL] = 0
                steering_angles[self.RR] = 0
                return steering_angles

            if(80 < deg < 100):
                # Drive straight forward
                steering_angles[self.FL] = 0
                steering_angles[self.FR] = 0
                steering_angles[self.CR] = 0
                steering_angles[self.CL] = 0
                steering_angles[self.RL] = 0
                steering_angles[self.RR] = 0
            elif(-80 < deg < -100):
                # Drive straight backwards
                steering_angles[self.FL] = 0
                steering_angles[self.FR] = 0
                steering_angles[self.CR] = 0
                steering_angles[self.CL] = 0
                steering_angles[self.RL] = 0
                steering_angles[self.RR] = 0
            elif(100 < deg <= 180):
                # Drive right forwards
                steering_angles[self.FL] = 45
                steering_angles[self.FR] = 45
                steering_angles[self.CR] = 0
                steering_angles[self.CL] = 0
                steering_angles[self.RL] = -45
                steering_angles[self.RR] = -45
            elif(-100 > deg >= -180):
                # Drive right backwards
                steering_angles[self.FL] = 45
                steering_angles[self.FR] = 45
                steering_angles[self.CR] = 0
                steering_angles[self.CL] = 0
                steering_angles[self.RL] = -45
                steering_angles[self.RR] = -45
            elif(80 > deg >= 0):
                # Drive left forwards
                steering_angles[self.FL] = -45
                steering_angles[self.FR] = -45
                steering_angles[self.CR] = 0
                steering_angles[self.CL] = 0
                steering_angles[self.RL] = 45
                steering_angles[self.RR] = 45
            elif(0 > deg > -80):
                # Drive left backwards
                steering_angles[self.FL] = -45
                steering_angles[self.FR] = -45
                steering_angles[self.CR] = 0
                steering_angles[self.CL] = 0
                steering_angles[self.RL] = 45
                steering_angles[self.RR] = 45

            return steering_angles
        if(self.locomotion_mode == LocomotionMode.ACKERMANN.value):

            # No steering if robot is not driving
            if(driving_command is 0):
                return steering_angles

            # Scale between min and max Ackermann radius
            if math.cos(math.radians(steering_command)) == 0:
                r = self.ackermann_r_max
            else:
                r = self.ackermann_r_max - \
                    abs(math.cos(math.radians(steering_command))) * \
                    ((self.ackermann_r_max-self.ackermann_r_min))

            # No steering
            if r == self.ackermann_r_max:
                return steering_angles

            inner_angle = int(math.degrees(
                math.atan(self.wheel_x/(abs(r)-self.wheel_y))))
            outer_angle = int(math.degrees(
                math.atan(self.wheel_x/(abs(r)+self.wheel_y))))

            if steering_command > 90 or steering_command < -90:
                # Steering to the right
                steering_angles[self.FL] = outer_angle
                steering_angles[self.FR] = inner_angle
                steering_angles[self.RL] = -outer_angle
                steering_angles[self.RR] = -inner_angle
            else:
                # Steering to the left
                steering_angles[self.FL] = -inner_angle
                steering_angles[self.FR] = -outer_angle
                steering_angles[self.RL] = inner_angle
                steering_angles[self.RR] = outer_angle

            return steering_angles

        if(self.locomotion_mode == LocomotionMode.POINT_TURN.value):
            steering_angles[self.FL] = 45
            steering_angles[self.FR] = -45
            steering_angles[self.RL] = -45
            steering_angles[self.RR] = 45

            return steering_angles
        if(self.locomotion_mode == LocomotionMode.CRABBING.value):
            if(driving_command != 0):
                wheel_direction = 0
                if(steering_command > 0):
                    wheel_direction = steering_command - 90

                elif(steering_command <= 0):
                    wheel_direction = steering_command + 90

                wheel_direction = np.clip(wheel_direction, -75, 75)

                steering_angles[self.FL] = wheel_direction
                steering_angles[self.FR] = wheel_direction
                steering_angles[self.CL] = wheel_direction
                steering_angles[self.CR] = wheel_direction
                steering_angles[self.RL] = wheel_direction
                steering_angles[self.RR] = wheel_direction

        return steering_angles

    def joystickToVelocity(self, driving_command, steering_command):
        '''
        Converts the steering and drive command to the speeds of the individual motors

        :param int driving_command: Drive speed command range from -100 to 100
        :param int stering_command: Turning radius command with the values 0(left) +90(forward) -90(backward)  +-180(right)
        '''

        motor_speeds = [0]*6
        if (self.locomotion_mode == LocomotionMode.FAKE_ACKERMANN.value):
            if(driving_command > 0 and steering_command >= 0):
                motor_speeds[self.FL] = 50
                motor_speeds[self.FR] = 50
                motor_speeds[self.CR] = 50
                motor_speeds[self.CL] = 50
                motor_speeds[self.RL] = 50
                motor_speeds[self.RR] = 50

            elif(driving_command > 0 and steering_command <= 0):
                motor_speeds[self.FL] = -50
                motor_speeds[self.FR] = -50
                motor_speeds[self.CR] = -50
                motor_speeds[self.CL] = -50
                motor_speeds[self.RL] = -50
                motor_speeds[self.RR] = -50

            return motor_speeds

        if (self.locomotion_mode == LocomotionMode.ACKERMANN.value):
            v = driving_command
            if(steering_command < 0):
                v *= -1

            # Scale between min and max Ackermann radius
            radius = self.ackermann_r_max - \
                abs(math.cos(math.radians(steering_command))) * \
                ((self.ackermann_r_max-self.ackermann_r_min))

            if (v == 0):
                return motor_speeds

            if (radius == self.ackermann_r_max):
                return [v] * 6
            else:
                rmax = radius + self.wheel_x

                a = math.pow(self.wheel_y, 2)
                b = math.pow(abs(radius) + self.wheel_x, 2)
                c = math.pow(abs(radius) - self.wheel_x, 2)
                rmax_float = float(rmax)

                r1 = math.sqrt(a+b)
                r2 = rmax_float
                r3 = r1
                r4 = math.sqrt(a+c)
                r5 = abs(radius) - self.wheel_x
                r6 = r4

                v1 = int(v)
                v2 = int(v*r2/r1)
                v3 = v1
                v4 = int(v*r4/r1)
                v5 = int(v*r5/r1)
                v6 = v4

                if (steering_command > 90 or steering_command < -90):
                    motor_speeds = [v1, v2, v3, v4, v5, v6]
                else:
                    motor_speeds = [v6, v5, v4, v3, v2, v1]

                return motor_speeds

        if (self.locomotion_mode == LocomotionMode.POINT_TURN.value):
            deg = steering_command
            if(driving_command is not 0):
                # Left turn
                if(deg < 85 and deg > -85):
                    motor_speeds[self.FL] = -50
                    motor_speeds[self.FR] = 50
                    motor_speeds[self.CL] = -50
                    motor_speeds[self.CR] = 50
                    motor_speeds[self.RL] = -50
                    motor_speeds[self.RR] = 50
                # Right turn
                elif(deg > 95 or deg < -95):
                    motor_speeds[self.FL] = 50
                    motor_speeds[self.FR] = -50
                    motor_speeds[self.CL] = 50
                    motor_speeds[self.CR] = -50
                    motor_speeds[self.RL] = 50
                    motor_speeds[self.RR] = -50
            else:
                # Stop
                motor_speeds[self.FL] = 0
                motor_speeds[self.FR] = 0
                motor_speeds[self.CL] = 0
                motor_speeds[self.CR] = 0
                motor_speeds[self.RL] = 0
                motor_speeds[self.RR] = 0

            return motor_speeds

        if(self.locomotion_mode == LocomotionMode.CRABBING.value):
            if(driving_command > 0):
                if(steering_command > 0):
                    motor_speeds[self.FL] = 50
                    motor_speeds[self.FR] = 50
                    motor_speeds[self.CL] = 50
                    motor_speeds[self.CR] = 50
                    motor_speeds[self.RL] = 50
                    motor_speeds[self.RR] = 50
                elif(steering_command <= 0):
                    motor_speeds[self.FL] = -50
                    motor_speeds[self.FR] = -50
                    motor_speeds[self.CL] = -50
                    motor_speeds[self.CR] = -50
                    motor_speeds[self.RL] = -50
                    motor_speeds[self.RR] = -50

        return motor_speeds

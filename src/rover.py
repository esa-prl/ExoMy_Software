#!/usr/bin/env python
import rospy
import time
import math
import enum
from locomotion_modes import LocomotionMode


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

        self.wheel_x = 14.0
        self.wheel_y = 16.0

        max_steering_angle = 50
        self.ackermann_r_min = abs(
            self.wheel_y) / math.tan(max_steering_angle * math.pi / 180.0) + self.wheel_x

        self.ackermann_r_max = 200

    def setLocomotionMode(self, locomotion_mode_command):
        '''
        Sets the locomotion mode
        '''
        self.locomotion_mode = locomotion_mode_command

    def joystickToSteeringAngle(self, driving_command, steering_command):
        '''
        Converts the steering command [angle of joystick] to angles for the different motors
        '''

        steering_angles = [0]*6
        deg = steering_command

        if(self.locomotion_mode == LocomotionMode.FAKE_ACKERMANN):
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
        if(self.locomotion_mode == LocomotionMode.ACKERMANN):
            # Scale between min and max Ackermann radius
            if steering_command == 0:
                r = self.ackermann_r_max
            elif -100 <= steering_command <= 100:
                r = self.ackermann_r_max - \
                    abs(steering_command) * \
                    ((self.ackermann_r_max-self.ackermann_r_min)/100)
            else:
                r = 250

            # No steering
            if r == self.ackermann_r_max:
                return steering_angles

            inner_angle = int(math.degrees(
                math.atan(self.wheel_x/(abs(r)-self.wheel_y))))
            outer_angle = int(math.degrees(
                math.atan(self.wheel_x/(abs(r)+self.wheel_y))))

            if steering_command > 0:
                # Steering to the right
                steering_angles[self.FL] = outer_angle
                steering_angles[self.FR] = inner_angle
                steering_angles[self.RL] = -outer_angle
                steering_angles[self.RR] = -inner_angle
            else:
                steering_angles[self.FL] = -outer_angle
                steering_angles[self.FR] = -inner_angle
                steering_angles[self.RL] = outer_angle
                steering_angles[self.RR] = inner_angle

            return steering_angles

        if(self.locomotion_mode == self.POINT_TURN):
            steering_angles[self.FL] = 45
            steering_angles[self.FR] = -45
            steering_angles[self.RL] = -45
            steering_angles[self.RR] = 45

            return steering_angles

        return steering_angles

    def joystickToVelocity(self, driving_command, steering_command):
        '''
        Converts the steering and drive command to the speeds of the individual motors

        :param int v: Drive speed command range from -100 to 100
        :param int r: Turning radius command range from -100 to 100
        '''

        motor_speeds = [0]*6

        if (self.locomotion_mode == LocomotionMode.FAKE_ACKERMANN):
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

        if (self.locomotion_mode == LocomotionMode.FAKE_ACKERMANN):
            v = driving_command
            radius = self.ackermann_r_max - \
                abs(steering_command) * \
                ((self.ackermann_r_max-self.ackermann_r_min)/100)

            if (v == 0):
                return motor_speeds

            if (abs(steering_command) <= self.ackermann_r_min):
                return [v] * 6
            else:
                rmax = radius + self.wheel_x
                if radius < 0:
                    rmax *= -1

                a = math.pow(self.wheel_y, 2)
                b = math.pow(self.wheel_y, 2)
                c = math.pow(abs(radius) + self.wheel_x, 2)
                d = math.pow(abs(radius) - self.wheel_x, 2)
                e = abs(radius) - self.wheel_x
                rmax_float = float(rmax)

                v1 = int(v*(math.sqrt(b + d))/rmax_float)
                # Slowest wheel
                v2 = int((v*e/rmax_float))
                v3 = int((v*math.sqrt(a + d))/rmax_float)
                v4 = int((v*math.sqrt(a + c))/rmax_float)
                v5 = int(v)                            # Fastest wheel
                v6 = int((v*math.sqrt(b + c))/rmax_float)

                if (steering_command < 0):
                    motor_speeds = [v1, v2, v3, v4, v5, v6]
                elif (steering_command > 0):
                    motor_speeds = [v6, v5, v4, v3, v2, v1]

                return motor_speeds

        if (self.locomotion_mode == self.POINT_TURN):
            return motor_speeds

        return motor_speeds

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
        
        # x = axes distance
        # y = axes width
        
        # Rear
        self.wheel_x = 14.0
        self.wheel_y = 20.3
        
        # Front
        self.wheel_fx = 16.0
        self.wheel_fy = 20.3
        

        max_steering_angle = 45
        
        # Rear
        self.ackermann_r_min = abs(self.wheel_x) / math.tan(max_steering_angle * math.pi / 180.0) + (self.wheel_y / 2)

        self.ackermann_r_max = 250

        # Front
        self.ackermann_fr_min = abs(self.wheel_fx) / math.tan(max_steering_angle * math.pi / 180.0) + (self.wheel_fy / 2)

        self.ackermann_fr_max = 250
        
        # Check minimum radius for front and back and set the bigger one for calculations (ExoMy can't turn narrower)
        if(self.ackermann_fr_min > self.ackermann_r_min):
            self.ackermann_r_min = self.ackermann_fr_min
        else:
            self.ackermann_fr_min = self.ackermann_r_min
        
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

            # Rear: Scale between min and max Ackermann radius
            if math.cos(math.radians(steering_command)) == 0:
                r = self.ackermann_r_max
            else:
                r = self.ackermann_r_max - \
                    abs(math.cos(math.radians(steering_command))) * \
                    ((self.ackermann_r_max-self.ackermann_r_min))

            # Front: Scale between min and max Ackermann radius
            if math.cos(math.radians(steering_command)) == 0:
                fr = self.ackermann_fr_max
            else:
                fr = self.ackermann_fr_max - \
                    abs(math.cos(math.radians(steering_command))) * \
                    ((self.ackermann_fr_max-self.ackermann_fr_min))
 
               
            # No steering
            if r == self.ackermann_r_max:
                return steering_angles

            # Rear
            inner_angle = int(math.degrees(math.atan(self.wheel_x/(abs(r)-(self.wheel_y/2)))))
            outer_angle = int(math.degrees(math.atan(self.wheel_x/(abs(r)+(self.wheel_y/2)))))
            
            # Front
            front_inner_angle = int(math.degrees(math.atan(self.wheel_fx/(abs(fr)-(self.wheel_fy/2)))))
            front_outer_angle = int(math.degrees(math.atan(self.wheel_fx/(abs(fr)+(self.wheel_fy/2)))))

            if steering_command > 90 or steering_command < -90:
                # Steering to the right
                steering_angles[self.FL] = front_outer_angle
                steering_angles[self.FR] = front_inner_angle
                steering_angles[self.RL] = -outer_angle
                steering_angles[self.RR] = -inner_angle
            else:
                # Steering to the left
                steering_angles[self.FL] = -front_inner_angle
                steering_angles[self.FR] = -front_outer_angle
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
            
            #Speed parameters
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
                # radius (r) and speed (v) definition for left turn
                r1 = ( radius - (self.wheel_fy / 2) ) / math.sin ( (90 - int(math.degrees(math.atan(self.wheel_fx/(abs(radius)-(self.wheel_fy/2)))))) * math.pi / 180.0)
                r2 = ( radius + (self.wheel_fy / 2) ) / math.sin ( (90 - int(math.degrees(math.atan(self.wheel_fx/(abs(radius)+(self.wheel_fy/2)))))) * math.pi / 180.0)
                r3 = ( radius - (self.wheel_fy / 2) )
                r4 = ( radius + (self.wheel_fy / 2) )
                r5 = ( radius - (self.wheel_y / 2) ) / math.sin ( (90 - int(math.degrees(math.atan(self.wheel_x/(abs(radius)-(self.wheel_y/2)))))) * math.pi / 180.0)
                r6 = ( radius + (self.wheel_y / 2) ) / math.sin ( (90 - int(math.degrees(math.atan(self.wheel_x/(abs(radius)+(self.wheel_y/2)))))) * math.pi / 180.0)
                
                # Select the biggest radius from all 6 to keep maximum speed of motors below max speed of the motors
                reference_radius = max(r1, r2, r3, r4, r5, r6)
                
                v1 = int(v*r1/reference_radius)
                v2 = int(v*r2/reference_radius)
                v3 = int(v*r3/reference_radius)
                v4 = int(v*r4/reference_radius)
                v5 = int(v*r5/reference_radius)
                v6 = int(v*r6/reference_radius)
                
                # FL, FR, CL, CR, RL, RR
                # right steering
                if (steering_command > 90 or steering_command < -90):
                    motor_speeds = [v2, v1, v4, v3, v6, v5]
                # left steering
                else:
                    motor_speeds = [v1, v2, v3, v4, v5, v6]

                return motor_speeds

        if (self.locomotion_mode == LocomotionMode.POINT_TURN.value):
            deg = steering_command
            if(driving_command is not 0):
                # Left turn
                if(deg < 85 and deg > -85):
                    motor_speeds[self.FL] = -v
                    motor_speeds[self.FR] = v
                    motor_speeds[self.CL] = -v
                    motor_speeds[self.CR] = v
                    motor_speeds[self.RL] = -v
                    motor_speeds[self.RR] = v
                # Right turn
                elif(deg > 95 or deg < -95):
                    motor_speeds[self.FL] = v
                    motor_speeds[self.FR] = -v
                    motor_speeds[self.CL] = v
                    motor_speeds[self.CR] = -v
                    motor_speeds[self.RL] = v
                    motor_speeds[self.RR] = -v
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
                    motor_speeds[self.FL] = v
                    motor_speeds[self.FR] = v
                    motor_speeds[self.CL] = v
                    motor_speeds[self.CR] = v
                    motor_speeds[self.RL] = v
                    motor_speeds[self.RR] = v
                elif(steering_command <= 0):
                    motor_speeds[self.FL] = -v
                    motor_speeds[self.FR] = -v
                    motor_speeds[self.CL] = -v
                    motor_speeds[self.CR] = -v
                    motor_speeds[self.RL] = -v
                    motor_speeds[self.RR] = -v

        return motor_speeds

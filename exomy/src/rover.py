#!/usr/bin/env python
import rospy
import time
import math
import enum
class Robot():
	'''
	Robot class contains all the math and motor control algorithms to move the rover
	In order to call command the robot the only method necessary is the sendCommands() method
	with drive velocity and turning amount
	'''

    
    # Defining wheel names
    FL, FR, CL, CR, RL, RR = range(0,6)

    # Defining locomotion modes
    ACKERMANN, POINT_TURN, CRABBING = range(0,3)

    zero_steering={90, 90, 90, 90, 90, 90}


	def __init__(self):
        self.locomotion_mode=ACKERMANN

        self.wheel_x=14.0
        self.wheel_y=16.0

        self.ackermann_r_min = max(
            abs(self.wheel_y) / math.tan(max_steering_angle * math.pi / 180.0) + self.wheel_x,
        )

        self.ackermann_r_max = 200
    
    def joystickToSteeringAngle(self, steering_command):
        '''
        Converts the steering command [left=-100 right=+100] to angles for the different motors
        '''

        steering_angles = [0]*6

        if(self.locomotion_mode==ACKERMANN):
            # Scale between min and max Ackermann radius
            if steering_command == 0: r = self.ackermann_r_max
            elif -100 <= steering_command <= 100: r = self.ackermann_r_max - abs(steering_command)*((self.ackermann_r_max-self.ackermann_r_min)/100)
		    else: r = 250
            
            # No steering
            if r == ackermann_r_max: return steering_angles

            inner_angle = int(math.degrees(math.atan(self.wheel_x/(abs(r)-self.wheel_y))))
            outer_angle = int(math.degrees(math.atan(self.wheel_x/(abs(r)+self.wheel_y))))

            if steering_command > 0:
                # Steering to the right
                steering_angles[FL]=outer_angle
                steering_angles[FR]=inner_angle
                steering_angles[RL]=-outer_angle
                steering_angles[RR]=-inner_angle
            else:
                steering_angles[FL]=-outer_angle
                steering_angles[FR]=-inner_angle
                steering_angles[RL]=outer_angle
                steering_angles[RR]=inner_angle

            return steering_angles

        if(self.locomotion_mode==POINT_TURN)
            steering_angles[FL]=45
            steering_angles[FR]=-45
            steering_angles[RL]=-45
            steering_angles[RR]=45

            return steering_angles

        return steering_angles
    
    def joystickToVelocity(self,driving_command, steering_command):
        '''
        Converts the steering and drive command to the speeds of the individual motors

        :param int v: Drive speed command range from -100 to 100
        :param int r: Turning radius command range from -100 to 100
        '''

        motor_speeds = [0]*6

        if (self.locomotion_mode==ACKERMANN):
            v=driving_command
            r = self.ackermann_r_max - abs(steering_command)*((self.ackermann_r_max-self.ackermann_r_min)/100)

            if (v == 0):
                return motor_speeds

            if (abs(r) == ackermann_r_min):
                return [v] * 6
            else:
                if radius < 0: rmax *= -1
                rmax = radius + self.d4

                a = math.pow(self.wheel_y,2)
                b = math.pow(self.wheel_y,2)
                c = math.pow(abs(radius) + self.wheel_x,2)
                d = math.pow(abs(radius) - self.wheel_x,2)
                e = abs(radius) - self.wheel_x
                rmax_float = float(rmax)

                v1 = int(v*(math.sqrt(b + d))/rmax_float)
                v2 = int((v*e/rmax_float))                        # Slowest wheel
                v3 = int((v*math.sqrt(a + d))/rmax_float)
                v4 = int((v*math.sqrt(a + c))/rmax_float)
                v5 = int(v)                            # Fastest wheel
                v6 = int((v*math.sqrt(b + c))/rmax_float)

                if (r < 0):
                    motor_speeds = [v1,v2,v3,v4,v5,v6]
                elif (r > 0):
                    motor_speeds = [v6,v5,v4,v3,v2,v1]

                return velocity

        if (self.locomotion_mode==POINT_TURN):
            return motor_speeds

            




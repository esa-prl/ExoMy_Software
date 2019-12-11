#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Joy
from exomy.msg import Joystick
import math
import enum

global mode
global screen_mode
global last
global counter

mode, screen_mode, counter = 0,0,0
last = time.time()


def callback(data):
    global mode
    global screen_mode
    global counter
    global last

    joy_out = Joystick()

    # Function map for the Logitech F710 joystick 
    # Button on pad | function
    # --------------|----------------------
    # B				| Toggle crabbing mode
    # X				| Toggle point turn mode
    # left stick	| control speed and direction

    # Reading out joystick data
    y =  data.axes[1]
    x =  data.axes[0]

    # Reading out button data
    if (data.buttons[1] == 1):
        if (screen_mode == 0):
            screen_mode = 1
        else:
            screen_mode = 0
    if (data.buttons[2] == 1):
        if (screen_mode != 2):
            screen_mode = 2
        else:
            screen_mode = 0
    if (data.buttons[3] == 1):
        if (screen_mode != 0):
            screen_mode = 0
        else:
            screen_mode = 3

    # The velocity is decoded as value between 0...100 
    joy_out.vel = 100 * math.sqrt(x*x + y*y)

    # The steering is described as an angle between -180...180
    # Which describe the joystick position as follows:
    #   +90
    # 0      +-180
    #   -90
    #
    joy_out.steering = math.atan2(y, x)*180.0/math.pi 

    joy_out.screen_mode = screen_mode
    joy_out.connected = True

    pub.publish(joy_out)

if __name__ == '__main__':
    global pub

    rospy.init_node('joystick')
    rospy.loginfo('joystick started')

    sub = rospy.Subscriber("/joy", Joy, callback)
    pub = rospy.Publisher('joystick', Joystick, queue_size=1)

    rospy.spin()


#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Joy
from exomy_msgs.msg import Joystick
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
    # LT			| decrease the max speed
    # LB			| increase the max speed
    # RT			| increase the angular velocity
    # RB			| decrease the angular velocity
    # B				| Toggle crabbing mode
    # X				| Toggle point turn mode
    # left stick	| control speed and direction

    y =  data.axes[1]
    x =  data.axes[0]

    dpad = data.buttons[11:]
    if 1 in dpad: mode = dpad.index(1)

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

    joy_out.vel = 100 * math.sqrt(x*x + y*y)
    joy_out.steering = math.atan2(y, x)*180.0/math.pi 

    joy_out.mode = mode
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


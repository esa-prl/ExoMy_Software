#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Joy
from exomy.msg import Joystick
from locomotion_modes import LocomotionMode
import math
import enum

# Define locomotion modes


def callback(data):

    joy_out = Joystick()

    # Function map for the Logitech F710 joystick
    # Button on pad | function
    # --------------|----------------------
    # B				| Toggle crabbing mode
    # X				| Toggle point turn mode
    # left stick	| control speed and direction

    # Reading out joystick data
    y = data.axes[1]
    x = data.axes[0]

    # Reading out button data
    if (data.buttons[0] == 1):
        joy_out.locomotion_mode = LocomotionMode.FAKE_ACKERMANN
        rospy.loginfo('FAKE_ACKERMANN')
    if (data.buttons[1] == 1):
        joy_out.locomotion = LocomotionMode.CRABBING
        rospy.loginfo('CRABBING')
    if (data.buttons[2] == 1):
        joy_out.locomotion_mode = LocomotionMode.POINT_TURN
        rospy.loginfo('POINT_TURN')

    # The velocity is decoded as value between 0...100
    joy_out.vel = 100 * math.sqrt(x*x + y*y)

    # The steering is described as an angle between -180...180
    # Which describe the joystick position as follows:
    #   +90
    # 0      +-180
    #   -90
    #
    joy_out.steering = math.atan2(y, x)*180.0/math.pi

    joy_out.connected = True

    pub.publish(joy_out)


def button_pressed(self, index):
    return self.prev_data.buttons[index] != self.curr_data.buttons[index] and self.curr_data.buttons[index]


def any_button_pressed(self, buttons_index):
    for index in buttons_index:
        if self.button_pressed(index):
            return True

    return False


if __name__ == '__main__':
    global pub

    rospy.init_node('joystick')
    rospy.loginfo('joystick started')

    sub = rospy.Subscriber("/joy", Joy, callback)
    pub = rospy.Publisher('joystick', Joystick, queue_size=1)

    rospy.spin()

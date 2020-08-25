#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from exomy.msg import RoverCommand
from locomotion_modes import LocomotionMode
import math

# Define locomotion modes
global locomotion_mode
global motors_enabled

locomotion_mode = LocomotionMode.ACKERMANN.value
motors_enabled = True


def callback(data):

    global locomotion_mode
    global motors_enabled

    rover_cmd = RoverCommand()

    # Function map for the Logitech F710 joystick
    # Button on pad | function
    # --------------|----------------------
    # A             | Ackermann mode
    # X             | Point turn mode
    # Y             | Crabbing mode
    # Left Stick    | Control speed and direction
    # START Button  | Enable and disable motors

    # Reading out joystick data
    y = data.axes[1]
    x = data.axes[0]

    # Reading out button data to set locomotion mode
    # X Button
    if (data.buttons[0] == 1):
        locomotion_mode = LocomotionMode.POINT_TURN.value
    # A Button
    if (data.buttons[1] == 1):
        locomotion_mode = LocomotionMode.ACKERMANN.value
    # B Button
    if (data.buttons[2] == 1):
        pass
    # Y Button
    if (data.buttons[3] == 1):
        locomotion_mode = LocomotionMode.CRABBING.value

    rover_cmd.locomotion_mode = locomotion_mode

    # Enable and disable motors
    # START Button
    if (data.buttons[9] == 1):
        if motors_enabled is True:
            motors_enabled = False
            rospy.loginfo("Motors disabled!")
        elif motors_enabled is False:
            motors_enabled = True
            rospy.loginfo("Motors enabled!")
        else:
            rospy.logerr(
                "Exceptional value for [motors_enabled] = {}".format(motors_enabled))
            motors_enabled = False

    rover_cmd.motors_enabled = motors_enabled

    # The velocity is decoded as value between 0...100
    rover_cmd.vel = 100 * min(math.sqrt(x*x + y*y), 1.0)

    # The steering is described as an angle between -180...180
    # Which describe the joystick position as follows:
    #   +90
    # 0      +-180
    #   -90
    #
    rover_cmd.steering = math.atan2(y, x)*180.0/math.pi

    rover_cmd.connected = True

    pub.publish(rover_cmd)


if __name__ == '__main__':
    global pub

    rospy.init_node('joystick_parser_node')
    rospy.loginfo('joystick_parser_node started')

    sub = rospy.Subscriber("/joy", Joy, callback, queue_size=1)
    pub = rospy.Publisher('/rover_command', RoverCommand, queue_size=1)

    rospy.spin()

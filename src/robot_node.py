#!/usr/bin/env python
import time
from exomy.msg import Joystick, Commands, Screen
import rospy
from rover import Rover
import message_filters


global exomy
exomy = Rover()


def joy_callback(message):
    cmds = Commands()

    exomy.setLocomotionMode(message.locomotion_mode)
    cmds.motor_angles = exomy.joystickToSteeringAngle(
        message.vel, message.steering)
    cmds.motor_speeds = exomy.joystickToVelocity(message.vel, message.steering)

    robot_pub.publish(cmds)


if __name__ == '__main__':
    rospy.init_node('robot')
    rospy.loginfo("Starting the robot node")
    global robot_pub
    joy_sub = rospy.Subscriber("/rover_commands", Joystick, joy_callback, queue_size=1)

    rate = rospy.Rate(10)

    robot_pub = rospy.Publisher("/motor_commands", Commands, queue_size=1)

    rospy.spin()

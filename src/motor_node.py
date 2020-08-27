#!/usr/bin/env python
import time
import rospy

from exomy.msg import MotorCommands
from motors import Motors

motors = Motors()
global watchdog_timer


def callback(cmds):
    motors.setSteering(cmds.motor_angles)
    motors.setDriving(cmds.motor_speeds)

    global watchdog_timer
    watchdog_timer.shutdown()
    # If this timer runs longer than the duration specified,
    # then watchdog() is called stopping the driving motors.
    watchdog_timer = rospy.Timer(rospy.Duration(5.0), watchdog, oneshot=True)


def shutdown():
    motors.stopMotors()


def watchdog(event):
    rospy.loginfo("Watchdog fired. Stopping driving motors.")
    motors.stopMotors()


if __name__ == "__main__":
    # This node waits for commands from the robot and sets the motors accordingly
    rospy.init_node("motors")
    rospy.loginfo("Starting the motors node")
    rospy.on_shutdown(shutdown)

    global watchdog_timer
    watchdog_timer = rospy.Timer(rospy.Duration(1.0), watchdog, oneshot=True)

    sub = rospy.Subscriber(
        "/motor_commands", MotorCommands, callback, queue_size=1)

    rate = rospy.Rate(10)

    rospy.spin()

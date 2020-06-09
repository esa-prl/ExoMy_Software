#!/usr/bin/env python
import time
import rospy

from exomy.msg import Commands
from motors import Motors

motors = Motors()
global watchdog_timer  

def callback(cmds):
    motors.setSteering(cmds.motor_angles)
    motors.setDriving(cmds.motor_speeds)
    global watchdog_timer
    watchdog_timer.shutdown()
    watchdog_timer = rospy.Timer(rospy.Duration(1), watchdog, oneshot=True)

def shutdown():
    motors.stopMotors()

def watchdog(event):
    rospy.loginfo("Watchdog fired")
    motors.stopMotors()

if __name__ == "__main__":
    # This node waits for commands from the robot and sets the motors accordingly
    rospy.init_node("motors")
    rospy.loginfo("Starting the motors node")
    rospy.on_shutdown(shutdown)
    global watchdog_timer
    watchdog_timer =  rospy.Timer(rospy.Duration(1), watchdog, oneshot=True)
    

    sub = rospy.Subscriber("/motor_commands",Commands, callback)

    rate = rospy.Rate(10)

    rospy.spin()

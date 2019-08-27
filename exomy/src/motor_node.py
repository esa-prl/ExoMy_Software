#!/usr/bin/env python
import time
import rospy

from exomy_msgs.msg import Commands
from motors import Motors

motors = Motors()

def callback(cmds):
    motors.setDriving(cmds.motor_speeds)
    time.sleep(0.5)
    motors.setSteering(cmds.motor_angles)
    
def shutdown():
    motors.stopMotors()
    motors.cleanup()

if __name__ == "__main__":
    rospy.init_node("motors")
    rospy.loginfo("Starting the motors node")
    rospy.on_shutdown(shutdown)

    sub = rospy.Subscriber("/robot_commands",Commands, callback)

    rate = rospy.Rate(5)

    rospy.spin()

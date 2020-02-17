#!/usr/bin/env python
import time
import rospy

from exomy.msg import Commands
from motors import Motors

motors = Motors()

def callback(cmds):
    motors.setSteering(cmds.motor_angles)
    motors.setDriving(cmds.motor_speeds)
    
def shutdown():
    motors.stopMotors()

if __name__ == "__main__":
    # This node waits for commands from the robot and sets the motors accordingly
    rospy.init_node("motors")
    rospy.loginfo("Starting the motors node")
    rospy.on_shutdown(shutdown)

    sub = rospy.Subscriber("/robot_commands",Commands, callback)

    rate = rospy.Rate(10)

    rospy.spin()

#!/usr/bin/env python
import time
from exomy_msgs.msg import Joystick, Commands
import rospy
from rover import Rover 
import message_filters


global exomy 
exomy = Rover()

def joy_callback(message):
 	cmds = Commands()
        cmds.motor_angles = exomy.joystickToSteeringAngle(message.steering)
        cmds.motor_speeds = exomy.joystickToVelocity(message.vel, message.steering)
 	try:
 		pub.publish(cmds)
 	except:
 		pass
        print(message)

if __name__ == '__main__':
	rospy.init_node('robot')
	rospy.loginfo("Starting the robot node")
	global pub
	joy_sub = rospy.Subscriber("/joystick",Joystick, joy_callback)

	rate = rospy.Rate(10)

 	pub = rospy.Publisher("/robot_commands", Commands, queue_size = 1)
	rate.sleep()
	rospy.spin()




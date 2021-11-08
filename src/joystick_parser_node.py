#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from exomy.msg import RoverCommand
from locomotion_modes import LocomotionMode
import math
import time

# Define locomotion modes
global locomotion_mode
global motors_enabled

locomotion_mode = LocomotionMode.ACKERMANN.value
motors_enabled = True


def callback(data):

    global locomotion_mode
    global motors_enabled

    rover_cmd = RoverCommand()

    # Function map for joystick
    # Button on pad | function
    # --------------|----------------------
    # A             | Ackermann mode
    # X             | Point turn mode
    # Y             | Crabbing mode
    # Left Stick    | Control speed and direction
    # START Button  | Enable and disable motors

    # More info on mapping: https://wiki.ros.org/joy
    if data.header.frame_id == "webgui":
        # Logitech WebGUI
        controller_funtion_map = {
            "x_axis": 0,
            "y_axis": 1,
            "invert_x_axis": True,
            "X_button": 0,
            "Y_button": 3,
            "A_button": 1,
            "B_button": 2,
            "start_button": 9,
            "select_button": 8,
            "sensitivity": 0.15
        }
    elif rospy.get_param("controller") == "logitech-F710": 
        # Logitech F710 joystick
        controller_funtion_map = {
            "x_axis": 0,
            "y_axis": 1,
            "invert_x_axis": True,
            "X_button": 0,
            "Y_button": 3,
            "A_button": 1,
            "B_button": 2,
            "start_button": 9,
            "select_button": 8,
            "sensitivity": 0.1
        }
    elif rospy.get_param("controller") == "xbox-one": 
        #X-Box One controller
        controller_funtion_map = {
            "x_axis": 0,
            "y_axis": 1,
            "invert_x_axis": True,
            "X_button": 2,
            "Y_button": 3,
            "A_button": 0,
            "B_button": 1,
            "start_button": 7,
            "select_button": 6,
            "sensitivity": 0.11
        }
    else:
        rospy.logerr("No controller identified. Using fallback.")
        controller_funtion_map = {
            "x_axis": 0,
            "y_axis": 1,
            "invert_x_axis": True,
            "X_button": 0,
            "Y_button": 3,
            "A_button": 1,
            "B_button": 2,
            "start_button": 9,
            "select_button": 8,
            "sensitivity": 0.0
        }
   
    # Reading out joystick data
    y = data.axes[1]
    x = data.axes[0]

    if controller_funtion_map["invert_x_axis"] == True:
        x = x * -1

    # Reading out button data to set locomotion mode
    # X Button
    if (data.buttons[controller_funtion_map["X_button"]] == 1):
        locomotion_mode = LocomotionMode.POINT_TURN.value
    # A Button
    if (data.buttons[controller_funtion_map["A_button"]] == 1):
        locomotion_mode = LocomotionMode.ACKERMANN.value
    # B Button
    if (data.buttons[controller_funtion_map["B_button"]] == 1):
        pass
    # Y Button
    if (data.buttons[controller_funtion_map["Y_button"]] == 1):
        locomotion_mode = LocomotionMode.CRABBING.value

    rover_cmd.locomotion_mode = locomotion_mode

    # Enable and disable motors
    # START Button
    if (data.buttons[controller_funtion_map["start_button"]] == 1):
        if motors_enabled is True:
            motors_enabled = False
            rospy.loginfo("Motors disabled!")
            # Set a sleep timer, if not a button movement could be triggered falsely
            time.sleep(0.5)
        elif motors_enabled is False:
            motors_enabled = True
            rospy.loginfo("Motors enabled!")
            # Set a sleep timer, if not a button movement could be triggered falsely
            time.sleep(0.5)
        else:
            rospy.logerr(
                "Exceptional value for [motors_enabled] = {}".format(motors_enabled))
            motors_enabled = False

    rover_cmd.motors_enabled = motors_enabled

    # The velocity is decoded as value between 0...100
    # Sensitivity defines an area in the center, where no steering or speed commands are send to allow for lower speeds without loosing directions
    # Similar to "joy_node"-"deadzone"-parameter. The parameter is not touched, as every controller has its own sensitivity
    rover_cmd.vel = min(math.sqrt(x*x + y*y), 1.0)
    if rover_cmd.vel < controller_funtion_map["sensitivity"]:
        rover_cmd.vel = 0
    else:
        rover_cmd.vel = (rover_cmd.vel - controller_funtion_map["sensitivity"]) * (1 / (1 - controller_funtion_map["sensitivity"])) * 100

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

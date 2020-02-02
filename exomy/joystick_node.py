#!/usr/bin/env python
import rclpy 
from rclpy.node import Node

import time
from sensor_msgs.msg import Joy
from exomy.msg import Joystick
from locomotion_modes import LocomotionMode
import math
import enum


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.callback,
            10)

        self.pub = self.create_publisher(Joystick, 'joystick', 1)

        self.locomotion_mode = LocomotionMode.FAKE_ACKERMANN.value



    def callback(self, joy_msg):

        global locomotion_mode
        joy_out = Joystick()

        # Function map for the Logitech F710 joystick
        # Button on pad | function
        # --------------|----------------------
        # X 			| Fake Ackermann mode
        # B				| Point turn mode
        # A				| Crabbing mode
        # left stick	| control speed and direction

        # Reading out joystick data
        y = joy_msg.axes[1]
        x = joy_msg.axes[0]

        # Reading out button data to set locomotion mode
        if (joy_msg.buttons[0] == 1):
            self.locomotion_mode = LocomotionMode.FAKE_ACKERMANN.value
        if (joy_msg.buttons[1] == 1):
            self.locomotion_mode = LocomotionMode.CRABBING.value
        if (joy_ms.buttons[2] == 1):
            self.locomotion_mode = LocomotionMode.POINT_TURN.value
        joy_out.locomotion_mode=self.locomotion_mode

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

        self.pub.publish(joy_out)




def main(args=None):
    rclpy.init(args=args)

    joystick_node = JoystickNode()

    rclpy.spin(joystick_node)

    joystick_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()


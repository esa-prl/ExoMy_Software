#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from exomy_msgs.msg import RoverCommand
from .locomotion_modes import LocomotionMode
import math


class GamepadParserNode(Node):
    def __init__(self):
        self.node_name = 'gamepad_parser_node'
        super().__init__(self.node_name)

        self.sub = self.create_subscription(
            Joy,
            'joy',
            self.callback,
            10)

        self.pub = self.create_publisher(
            RoverCommand,
            'rover_command',
            1)

        self.locomotion_mode = LocomotionMode.ACKERMANN.value
        self.motors_enabled = True

        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))

    def callback(self, data):

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
            self.locomotion_mode = LocomotionMode.POINT_TURN.value
        # A Button
        if (data.buttons[1] == 1):
            self.locomotion_mode = LocomotionMode.ACKERMANN.value
        # B Button
        if (data.buttons[2] == 1):
            pass
        # Y Button
        if (data.buttons[3] == 1):
            self.locomotion_mode = LocomotionMode.CRABBING.value

        rover_cmd.locomotion_mode = self.locomotion_mode

        # Enable and disable motors
        # START Button
        if (data.buttons[9] == 1):
            if self.motors_enabled is True:
                self.motors_enabled = False
                self.get_logger().info("Motors disabled!")
            elif self.motors_enabled is False:
                self.motors_enabled = True
                self.get_logger().info("Motors enabled!")
            else:
                self.get_logger().error(
                    "Exceptional value for [motors_enabled] \
                    = {}".format(self.motors_enabled))
                self.motors_enabled = False

        rover_cmd.motors_enabled = self.motors_enabled

        # The velocity is decoded as value between 0...100
        rover_cmd.vel = int(100 * min(math.sqrt(x*x + y*y), 1.0))

        # The steering is described as an angle between -180...180
        # Which describe the joystick position as follows:
        #   +90
        # 0      +-180
        #   -90
        #
        rover_cmd.steering = int(math.atan2(y, x)*180.0/math.pi)

        rover_cmd.connected = True

        self.pub.publish(rover_cmd)


def main(args=None):
    rclpy.init(args=args)

    try:
        gamepad_parser_node = GamepadParserNode()
        try:
            rclpy.spin(gamepad_parser_node)
        finally:
            gamepad_parser_node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

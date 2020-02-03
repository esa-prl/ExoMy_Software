#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import time

from exomy_msgs.msg import Commands
from exomy.motors import Motors


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')

        self.subscription = self.create_subscription(
            Commands,
            '/robot_commands',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning
        self.motors = Motors()

    def __del__(self):
        self.motors.stopMotors()

    def callback(self, cmds):
        self.motors.setSteering(cmds.motor_angles)
        self.motors.setDriving(cmds.motor_speeds)


def main(args=None):
    rclpy.init(args=args)

    motor_node = MotorNode()

    rclpy.spin(motor_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

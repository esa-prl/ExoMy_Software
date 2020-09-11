#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from exomy_msgs.msg import RoverCommand
from exomy_core.motors import Motors


class MotorNode(Node):
    def __init__(self):
        self.node_name = 'motor_node'
        super().__init__(self.node_name)

        self.subscription = self.create_subscription(
            RoverCommand,
            'rover_command',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning
        self.motors = Motors()

        self.watchdog_timer

        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))

    def __del__(self):
        self.motors.stopMotors()

    def callback(self, cmds):
        self.motors.setSteering(cmds.motor_angles)
        self.motors.setDriving(cmds.motor_speeds)

        self.watchdog_timer.shutdown()
        # If this timer runs longer than the duration specified,
        # then watchdog() is called stopping the driving motors.
        self.watchdog_timer = rclpy.Timer(
            rclpy.Duration(5.0), self.watchdog, oneshot=True)

    def watchdog(self, event):
        self.get_logger().info('Watchdog fired. Stopping driving motors.')
        self.motors.stopMotors()


def main(args=None):
    rclpy.init(args=args)

    motor_node = MotorNode()
    motor_node.watchdog_timer = rclpy.Timer(
        rclpy.Duration(5.0), motor_node.watchdog, oneshot=True)

    rclpy.spin(motor_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from exomy_msgs.msg import MotorCommands
from .motors import Motors


class MotorNode(Node):
    def __init__(self):
        self.node_name = 'motor_node'
        super().__init__(self.node_name)

        self.subscription = self.create_subscription(
            MotorCommands,
            'motor_commands',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning
        self.motors = Motors()

        self.watchdog_timer = self.create_timer(5.0, self.watchdog)

        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))

    def __del__(self):
        self.motors.stopMotors()

    def callback(self, cmds):
        self.motors.setSteering(cmds.motor_angles)
        self.motors.setDriving(cmds.motor_speeds)

        self.watchdog_timer.cancel()
        # If this timer runs longer than the duration specified,
        # then watchdog() is called stopping the driving motors.
        self.watchdog_timer = self.create_timer(5.0, self.watchdog)

    def watchdog(self):
        self.get_logger().info('Watchdog fired. Stopping driving motors.')
        self.motors.stopMotors()


def main(args=None):
    rclpy.init(args=args)

    try:
        motor_node = MotorNode()
        try:
            rclpy.spin(motor_node)
        finally:
            motor_node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

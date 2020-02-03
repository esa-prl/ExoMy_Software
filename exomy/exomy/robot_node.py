#!/usr/bin/env python
import time
from exomy_msgs.msg import Joystick, Commands
import rclpy
from rclpy.node import Node
from exomy.rover import Rover


class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        self.joy_sub = self.create_subscription(
            Joystick,
            '/joystick',
            self.joy_callback,
            10)

        self.robot_pub = self.create_publisher(
            Commands,
            '/robot_commands',
            1)
        self.robot = Rover()

    def joy_callback(self, msg):
        cmds = Commands()

        self.robot.setLocomotionMode(msg.locomotion_mode)
        cmds.motor_angles = self.robot.joystickToSteeringAngle(
            msg.vel, msg.steering)
        cmds.motor_speeds = self.robot.joystickToVelocity(
            msg.vel, msg.steering)

        self.robot_pub.publish(cmds)


def main(args=None):
    rclpy.init(args=args)

    robot_node = RobotNode()

    rclpy.spin(robot_node)

    rover_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

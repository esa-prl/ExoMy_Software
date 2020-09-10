import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node

namespace_ = 'exomy'


def generate_launch_description():
    robot = Node(
        package='exomy',
        node_executable='robot_node',
        node_name='robot_node',
        node_namespace=namespace_,
        output='screen'
    )
    joy = Node(
        package='joy',
        node_executable='joy_node',
        node_name='joy_node',
        node_namespace=namespace_,
        output='screen'
    )
    joystick = Node(
        package='exomy',
        node_executable='joystick_node',
        node_name='joystick_node',
        node_namespace=namespace_,
        output='screen'
    )
    motors = Node(
        package='exomy',
        node_executable='motor_node',
        node_name='motor_node',
        node_namespace=namespace_,
        output='screen'
    )

    return LaunchDescription([
        robot,
        joystick,
        joy,
        motors
    ])

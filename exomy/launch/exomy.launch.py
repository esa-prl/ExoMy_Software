
from launch import LaunchDescription
from launch_ros.actions import Node

namespace_ = 'exomy'


def generate_launch_description():
    robot = Node(
        package='exomy',
        executable='robot_node',
        name='robot_node',
        namespace=namespace_,
        output='screen'
    )
    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=namespace_,
        output='screen'
    )
    joystick = Node(
        package='exomy',
        executable='joystick_parser_node',
        name='joystick_parser_node',
        namespace=namespace_,
        output='screen'
    )
    motors = Node(
        package='exomy',
        executable='motor_node',
        name='motor_node',
        namespace=namespace_,
        output='screen'
    )

    return LaunchDescription([
        robot,
        joystick,
        joy,
        motors
    ])

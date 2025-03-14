from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'joy',
            executable = 'joy_node',
            name = 'launch'
        ),
        Node(
            package = 'my_saab',
            executable = 'pedal',
            name = 'launch'
        ),
        Node(
            package = 'my_saab',
            executable = 'steering',
            name = 'launch'
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "--dev", "/dev/ttyUSB0"]
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "--dev", "/dev/ttyUSB1"]
        )
    ])
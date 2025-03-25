from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'joy',
            executable = 'joy_node',
            name = 'joy'
        ),
        Node(
            package = 'my_saab',
            executable = 'pedal',
            name = 'pedal'
        ),
        Node(
            package = 'my_saab',
            executable = 'steering',
            name = 'steering'
        ),
        Node(
            package = 'my_saab',
            executable = 'shutdown',
            name = 'shutdown'
        ),
        Node(
            package = 'my_saab',
            executable = 'machine_vision',
            name = 'machine_vision'
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent_pedal',
            arguments=["serial", "--dev", "/dev/ttyUSB0"]
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent_steering',
            arguments=["serial", "--dev", "/dev/ttyUSB1"]
        )
        
    ])
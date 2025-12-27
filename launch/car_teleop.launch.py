from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.05}]
        ),
        Node(
            package='rc_car_control',
            executable='donkey_control',
            name='donkey_control'
        )
    ])

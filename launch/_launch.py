from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='twister',
            executable='launch'
        ),
        Node(
            package='veler',
            executable='launch'
        ),
        Node(
            package='joy',
            executable='joy_node',
        )
    ])
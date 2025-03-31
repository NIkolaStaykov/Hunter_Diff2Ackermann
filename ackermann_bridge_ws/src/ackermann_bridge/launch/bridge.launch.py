from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ackermann_bridge',
            executable='control_transformer',
            name='control_transformer',
            output='screen'
        )
    ])

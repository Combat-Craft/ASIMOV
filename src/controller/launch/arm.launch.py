from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller',
            executable='arm_driver',
            name='arm_driver',
            output='screen'
        ),
    ])

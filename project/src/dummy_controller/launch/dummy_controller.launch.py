"""Start the example controller without the autonomous-driving stack."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # This node also publishes /car_command; do not run it with control.
        Node(
            package='dummy_controller',
            executable='dummy_controller_node',
            name='dummy_controller_node',
            output='screen',
        ),
    ])

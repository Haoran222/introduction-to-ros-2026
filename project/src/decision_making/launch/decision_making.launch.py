"""Launch obstacle monitoring and behavior decisions."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Monitor forward obstacles.
        Node(
            package='decision_making',
            executable='forward_obstacle_monitor_node',
            name='forward_obstacle_monitor',
            output='screen',
        ),

        # Select the driving state.
        Node(
            package='decision_making',
            executable='decision_state_machine_node',
            name='decision_state_machine',
            output='screen',
        ),
    ])

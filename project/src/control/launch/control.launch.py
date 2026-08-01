"""Launch the trajectory follower."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control',
            executable='trajectory_follower_node',
            name='trajectory_follower',
            output='screen',
        ),
    ])

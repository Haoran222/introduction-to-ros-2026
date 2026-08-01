"""Launch goal selection and trajectory planning."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_waypoints_file = os.path.join(
        get_package_share_directory('planning'), 'config', 'waypoints.yaml')

    waypoints_file = LaunchConfiguration('waypoints_file')
    max_speed = LaunchConfiguration('max_speed')
    max_lateral_accel = LaunchConfiguration('max_lateral_accel')
    lookahead_count = LaunchConfiguration('lookahead_count')
    terminal_decel = LaunchConfiguration('terminal_decel')
    final_arrival_radius = LaunchConfiguration('final_arrival_radius')

    return LaunchDescription([
        DeclareLaunchArgument('waypoints_file', default_value=default_waypoints_file),
        DeclareLaunchArgument('max_speed', default_value='4.0'),
        DeclareLaunchArgument('max_lateral_accel', default_value='1.0'),
        DeclareLaunchArgument('lookahead_count', default_value='15'),
        DeclareLaunchArgument('terminal_decel', default_value='0.3'),
        DeclareLaunchArgument('final_arrival_radius', default_value='3.0'),

        # Select the next route goal.
        Node(
            package='planning',
            executable='goal_selector_node',
            name='goal_selector',
            output='screen',
            parameters=[{
                'waypoints_file': waypoints_file,
                'lookahead_count': lookahead_count,
                'final_arrival_radius': final_arrival_radius,
            }],
        ),

        # Build a smooth, speed-limited trajectory.
        Node(
            package='planning',
            executable='trajectory_planner_node',
            name='trajectory_planner',
            output='screen',
            parameters=[{
                'max_speed': max_speed,
                'max_lateral_accel': max_lateral_accel,
                'local_path_capacity': lookahead_count,
                'terminal_decel': terminal_decel,
            }],
        ),
    ])

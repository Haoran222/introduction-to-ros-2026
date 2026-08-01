"""Launch a manual waypoint-recording session."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_file = LaunchConfiguration('output_file')
    min_spacing = LaunchConfiguration('min_spacing')
    min_corner_spacing = LaunchConfiguration('min_corner_spacing')
    corner_yaw_threshold_deg = LaunchConfiguration('corner_yaw_threshold_deg')

    return LaunchDescription([
        DeclareLaunchArgument('output_file', default_value='/tmp/waypoints_recorded.yaml'),
        DeclareLaunchArgument('min_spacing', default_value='2.0'),
        DeclareLaunchArgument('min_corner_spacing', default_value='0.5'),
        DeclareLaunchArgument('corner_yaw_threshold_deg', default_value='12.0'),

        Node(
            package='planning',
            executable='waypoint_recorder_node',
            name='waypoint_recorder',
            output='screen',
            parameters=[{
                'output_file': output_file,
                'min_spacing': min_spacing,
                'min_corner_spacing': min_corner_spacing,
                'corner_yaw_threshold_deg': corner_yaw_threshold_deg,
            }],
        ),
    ])

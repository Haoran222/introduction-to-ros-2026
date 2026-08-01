"""Start the Unity parameter reader and the bidirectional ROS bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def default_config_path():
    """Configuration installed beside the bundled Unity player."""

    return PathJoinSubstitution([
        FindPackageShare('simulation'),
        'unity_sim',
        'Build_Ubuntu',
        'AD_Sim_Data',
        'StreamingAssets',
        'simulation_config.json',
    ])


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_path(),
            description='Unity bridge and sensor configuration.',
        ),

        # Read the JSON first so parameters are available to the bridge nodes.
        Node(
            package='simulation',
            executable='JSON_param_reader',
            name='JSON_param_reader',
            output='screen',
            parameters=[{'config_file': config_file}],
        ),

        # Unity -> ROS sensor stream.
        Node(
            package='simulation',
            executable='unity_TCP_stream_receiver',
            name='Unity_ROS_message_Rx',
            output='screen',
            parameters=[{'config_file': config_file, 'bind_host': '0.0.0.0'}],
        ),

        # ROS -> Unity vehicle command stream.
        Node(
            package='simulation',
            executable='ROS_command_transmitter',
            name='ROS_Unity_command_Tx',
            output='screen',
            parameters=[{'config_file': config_file}],
        ),
    ])

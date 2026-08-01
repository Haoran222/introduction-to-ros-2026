"""Launch the complete driving stack and optional Unity player."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def package_launch(package, filename, launch_arguments=None):
    """Include a package launch file."""

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', filename])
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def simulation_config_path():
    """Return the installed simulation configuration path."""

    return PathJoinSubstitution([
        FindPackageShare('simulation'),
        'unity_sim',
        'Build_Ubuntu',
        'AD_Sim_Data',
        'StreamingAssets',
        'simulation_config.json',
    ])


def create_unity_process():
    """Create the Unity process."""

    unity_directory = PathJoinSubstitution([
        FindPackageShare('simulation'),
        'unity_sim',
        'Build_Ubuntu',
    ])

    unity_executable = PathJoinSubstitution([
        unity_directory,
        'AD_Sim.x86_64',
    ])

    return ExecuteProcess(
        cmd=[unity_executable],
        cwd=unity_directory,
        additional_env={'GALLIUM_DRIVER': 'd3d12'},
        output='screen',
    )


def generate_launch_description():
    # Shared file and runtime options.
    config_file = LaunchConfiguration('config_file')
    waypoints_file = LaunchConfiguration('waypoints_file')
    start_unity = LaunchConfiguration('start_unity')
    unity_start_delay = LaunchConfiguration('unity_start_delay')

    # Planning and control limits.
    max_speed = LaunchConfiguration('max_speed')
    max_lateral_accel = LaunchConfiguration('max_lateral_accel')
    planning_lookahead_count = LaunchConfiguration('planning_lookahead_count')
    terminal_decel = LaunchConfiguration('terminal_decel')
    final_arrival_radius = LaunchConfiguration('final_arrival_radius')

    # Perception inputs.
    depth_topic = LaunchConfiguration('depth_topic')
    rgb_topic = LaunchConfiguration('rgb_topic')
    octomap_resolution = LaunchConfiguration('octomap_resolution')
    max_range = LaunchConfiguration('max_range')

    unity_process = create_unity_process()

    launch_arguments = [
        DeclareLaunchArgument(
            'config_file',
            default_value=simulation_config_path(),
            description='Simulation and Unity JSON configuration file.',
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('planning'), 'config', 'waypoints.yaml'
            ]),
            description='Recorded route used by the planning stack.',
        ),
        DeclareLaunchArgument(
            'start_unity',
            default_value='true',
            choices=['true', 'false'],
            description='Start the vendored Linux Unity player.',
        ),
        DeclareLaunchArgument(
            'unity_start_delay',
            default_value='2.0',
            description='Seconds to wait for the TCP bridge before starting Unity.',
        ),
        DeclareLaunchArgument('max_speed', default_value='4.0'),
        DeclareLaunchArgument(
            'max_lateral_accel',
            default_value='1.0',
            description='Lateral acceleration cap used to slow sharp bends.',
        ),
        DeclareLaunchArgument(
            'planning_lookahead_count',
            default_value='15',
            description='Number of recorded waypoints in the local planning window.',
        ),
        DeclareLaunchArgument(
            'terminal_decel',
            default_value='0.3',
            description='Measured full-brake deceleration used for the final stop.',
        ),
        DeclareLaunchArgument(
            'final_arrival_radius',
            default_value='3.0',
            description='Completion tolerance for the final recorded waypoint.',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/OurCar/Sensors/DepthCamera/image_raw',
        ),
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/OurCar/Sensors/RGBCameraLeft/image_raw',
        ),
        DeclareLaunchArgument('octomap_resolution', default_value='0.2'),
        DeclareLaunchArgument('max_range', default_value='20.0'),
    ]

    stack = [
        # Unity/ROS TCP bridge.
        package_launch(
            'simulation',
            'simulation.launch.py',
            {'config_file': config_file},
        ),

        # Sensor transforms, depth processing, and traffic-light detection.
        package_launch(
            'perception',
            'perception.launch.py',
            {
                'depth_topic': depth_topic,
                'rgb_topic': rgb_topic,
                'octomap_resolution': octomap_resolution,
                'max_range': max_range,
            },
        ),

        # Waypoint selection and local trajectory generation.
        package_launch(
            'planning',
            'planning.launch.py',
            {
                'waypoints_file': waypoints_file,
                'max_speed': max_speed,
                'max_lateral_accel': max_lateral_accel,
                'lookahead_count': planning_lookahead_count,
                'terminal_decel': terminal_decel,
                'final_arrival_radius': final_arrival_radius,
            },
        ),

        # Safety gating and vehicle command generation.
        package_launch('decision_making', 'decision_making.launch.py'),
        package_launch('control', 'control.launch.py'),
    ]

    unity_lifecycle = [
        # Start Unity only after the TCP receiver has had time to bind its port.
        TimerAction(
            period=unity_start_delay,
            actions=[unity_process],
            condition=IfCondition(start_unity),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=unity_process,
                on_exit=[
                    EmitEvent(event=Shutdown(
                        reason='Unity exited; shutting down the driving stack.'
                    ))
                ],
            ),
            condition=IfCondition(start_unity),
        ),
    ]

    return LaunchDescription([
        *launch_arguments,
        *stack,
        *unity_lifecycle,
    ])

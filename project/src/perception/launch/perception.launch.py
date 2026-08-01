"""Launch sensor transforms, depth processing, OctoMap, and light detection."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depth_topic = LaunchConfiguration('depth_topic')
    rgb_topic = LaunchConfiguration('rgb_topic')
    octomap_resolution = LaunchConfiguration('octomap_resolution')
    max_range = LaunchConfiguration('max_range')

    return LaunchDescription([
        DeclareLaunchArgument('depth_topic', default_value='/OurCar/Sensors/DepthCamera/image_raw'),
        DeclareLaunchArgument('rgb_topic', default_value='/OurCar/Sensors/RGBCameraLeft/image_raw'),
        DeclareLaunchArgument('octomap_resolution', default_value='0.2'),
        DeclareLaunchArgument('max_range', default_value='20.0'),

        # Publish static sensor transforms.
        Node(
            package='perception',
            executable='sensor_tf_broadcaster_node',
            name='sensor_tf_broadcaster',
            output='screen',
        ),

        # Convert depth images to a point cloud.
        Node(
            package='perception',
            executable='depth_to_pointcloud_node',
            name='depth_to_pointcloud',
            output='screen',
            parameters=[{
                'depth_topic': depth_topic,
                'max_range': max_range,
            }],
            remappings=[('points', '/perception/points')],
        ),

        # Build the occupancy map.
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'frame_id': 'world',
                'resolution': octomap_resolution,
                'sensor_model.max_range': max_range,
            }],
            remappings=[('cloud_in', '/perception/points')],
        ),

        # Traffic Light Recognition: classical color/shape detection on the RGB
        # camera image (works without the semantic camera).
        Node(
            package='perception',
            executable='traffic_light_detector_node',
            name='traffic_light_detector',
            output='screen',
            parameters=[{
                'image_topic': rgb_topic,
            }],
        ),
    ])

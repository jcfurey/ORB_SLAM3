"""Generic monocular ORB-SLAM3 launch. Point image_topic at any camera's stream."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('settings_file', description='Path to the ORB-SLAM3 .yaml settings'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera'),
        DeclareLaunchArgument('world_frame_id', default_value='map'),
        DeclareLaunchArgument('qos_reliability', default_value='sensor_data'),
    ]
    node = Node(
        package='orb_slam3',
        executable='mono_node',
        name='orb_slam3_mono',
        output='screen',
        parameters=[{
            'settings_file': LaunchConfiguration('settings_file'),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'world_frame_id': LaunchConfiguration('world_frame_id'),
            'qos_reliability': LaunchConfiguration('qos_reliability'),
            'autostart': True,
        }],
    )
    return LaunchDescription(args + [node])

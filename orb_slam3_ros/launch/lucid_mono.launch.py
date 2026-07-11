"""Monocular ORB-SLAM3 for a LUCID Vision camera (Triton/Phoenix via the
arena_camera ROS 2 driver). The arena driver typically publishes on
/arena_camera_node/image_raw -- override image_topic to match your setup.
LUCID drivers often use RELIABLE QoS, so qos_reliability defaults to 'reliable'
here (switch to 'sensor_data' if your driver uses best-effort).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('settings_file', description='Path to the ORB-SLAM3 monocular .yaml'),
        DeclareLaunchArgument('image_topic', default_value='/arena_camera_node/image_raw'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera'),
        DeclareLaunchArgument('qos_reliability', default_value='reliable'),
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
            'qos_reliability': LaunchConfiguration('qos_reliability'),
            'autostart': True,
        }],
    )
    return LaunchDescription(args + [node])

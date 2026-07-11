"""RGB-D ORB-SLAM3 for an Intel RealSense (e.g. D435/D435i/D455).

Run the camera with aligned depth, e.g.:
  ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true \
       rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('settings_file', description='Path to the ORB-SLAM3 RGB-D .yaml'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera_color_optical_frame'),
    ]
    node = Node(
        package='orb_slam3',
        executable='rgbd_node',
        name='orb_slam3_rgbd',
        output='screen',
        parameters=[{
            'settings_file': LaunchConfiguration('settings_file'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'qos_reliability': 'sensor_data',
            'autostart': True,
        }],
    )
    return LaunchDescription(args + [node])

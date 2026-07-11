"""Stereo-inertial (VIO) ORB-SLAM3 for an Intel RealSense D435i / D455.

Run the camera with the IR stereo pair, a united IMU, and the projector OFF:
  ros2 launch realsense2_camera rs_launch.py \
       enable_infra1:=true enable_infra2:=true enable_gyro:=true enable_accel:=true \
       unite_imu_method:=2 depth_module.emitter_enabled:=0 \
       infra_rgb:=false depth_module.depth_profile:=640x480x30
(unite_imu_method 1=copy, 2=linear_interpolation; 2 is recommended.)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('settings_file',
                              description='Path to the ORB-SLAM3 stereo-inertial .yaml'),
        DeclareLaunchArgument('left_topic', default_value='/camera/camera/infra1/image_rect_raw'),
        DeclareLaunchArgument('right_topic', default_value='/camera/camera/infra2/image_rect_raw'),
        DeclareLaunchArgument('imu_topic', default_value='/camera/camera/imu'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera_infra1_optical_frame'),
    ]
    node = Node(
        package='orb_slam3',
        executable='stereo_inertial_node',
        name='orb_slam3_stereo_inertial',
        output='screen',
        parameters=[{
            'settings_file': LaunchConfiguration('settings_file'),
            'left_topic': LaunchConfiguration('left_topic'),
            'right_topic': LaunchConfiguration('right_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'qos_reliability': 'sensor_data',
            'autostart': True,
        }],
    )
    return LaunchDescription(args + [node])

"""Stereo-inertial (VIO) ORB-SLAM3 for a Luxonis OAK-D (depthai_ros_driver).

Run the camera so it publishes rectified mono stereo + a combined IMU, e.g. a
depthai driver config with left/right mono rectified outputs and imu enabled.
Topic names differ by driver version -- override *_topic below to match
`ros2 topic list`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('settings_file',
                              description='Path to the ORB-SLAM3 stereo-inertial .yaml'),
        DeclareLaunchArgument('left_topic', default_value='/oak/left/image_rect'),
        DeclareLaunchArgument('right_topic', default_value='/oak/right/image_rect'),
        DeclareLaunchArgument('imu_topic', default_value='/oak/imu/data'),
        DeclareLaunchArgument('camera_frame_id', default_value='oak_left_camera_optical_frame'),
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

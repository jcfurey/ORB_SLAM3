"""ORB-SLAM3 + robot_localization (map-frame EKF), REP-105 compliant.

Brings up, as a turnkey example (monocular by default):
  * the ORB-SLAM3 node in **base-frame mode** (base_frame_id:=base_link) with
    publish_tf:=false, so it publishes an absolute ``~/odom`` of base_link-in-map
    and lets robot_localization own the ``map -> odom`` transform;
  * a static ``base_link -> camera`` transform (edit x/y/z/roll/pitch/yaw to your
    mounting; child MUST equal camera_frame_id);
  * a robot_localization ``ekf_node`` (map EKF) fusing ``~/odom`` and publishing
    ``map -> odom`` (config: orb_slam3_ros/config/ekf_map.yaml).

Pair this with your own odom-frame EKF (wheel odometry / IMU -> ``odom -> base_link``)
for a full dual-EKF stack. See docs/ROBOT_LOCALIZATION.md.

Example:
  ros2 launch orb_slam3 robot_localization.launch.py \
       settings_file:=/abs/mono.yaml image_topic:=/camera/image_raw \
       cam_x:=0.10 cam_z:=0.20 cam_roll:=-1.5708 cam_yaw:=-1.5708
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os


def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('orb_slam3'), 'config', 'ekf_map.yaml')

    args = [
        DeclareLaunchArgument('settings_file', description='Path to the ORB-SLAM3 .yaml settings'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera'),
        DeclareLaunchArgument('base_frame_id', default_value='base_link'),
        DeclareLaunchArgument('world_frame_id', default_value='map'),
        DeclareLaunchArgument('qos_reliability', default_value='sensor_data'),
        # Camera mounting in base_link (metres / radians, optical convention default).
        DeclareLaunchArgument('cam_x', default_value='0.0'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.0'),
        DeclareLaunchArgument('cam_roll', default_value='-1.5708'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_yaw', default_value='-1.5708'),
        DeclareLaunchArgument('ekf_config', default_value=ekf_config),
    ]

    slam = Node(
        package='orb_slam3',
        executable='mono_node',
        name='orb_slam3_mono',
        output='screen',
        parameters=[{
            'settings_file': LaunchConfiguration('settings_file'),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'world_frame_id': LaunchConfiguration('world_frame_id'),
            'qos_reliability': LaunchConfiguration('qos_reliability'),
            # robot_localization owns map -> odom, so the node must NOT broadcast it.
            'publish_tf': False,
            'publish_odom': True,
            # Publish ORB-SLAM3's own SE3 marginal so the EKF weights VO correctly.
            'covariance_mode': 'g2o',
            'autostart': True,
        }],
    )

    base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_static_tf',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--roll', LaunchConfiguration('cam_roll'),
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--yaw', LaunchConfiguration('cam_yaw'),
            '--frame-id', LaunchConfiguration('base_frame_id'),
            '--child-frame-id', LaunchConfiguration('camera_frame_id'),
        ],
    )

    ekf_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config')],
    )

    return LaunchDescription(args + [base_to_camera, slam, ekf_map])

"""Publish a static transform from the robot body frame to the ORB-SLAM3 camera
frame, so the SLAM `map -> camera` chain connects to your robot's `base_link`.

Set the translation (m) and rotation (rad) of the camera in the parent frame to
match your mounting. `child_frame` MUST equal the node's `camera_frame_id`.

Example (camera 10 cm forward, 20 cm up, looking forward, optical convention):
  ros2 launch orb_slam3 base_to_camera.launch.py \
       parent_frame:=base_link child_frame:=camera x:=0.10 z:=0.20 \
       roll:=-1.5708 yaw:=-1.5708
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('parent_frame', default_value='base_link'),
        DeclareLaunchArgument('child_frame', default_value='camera'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]
    stp = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_static_tf',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('x'),
            '--y', LaunchConfiguration('y'),
            '--z', LaunchConfiguration('z'),
            '--roll', LaunchConfiguration('roll'),
            '--pitch', LaunchConfiguration('pitch'),
            '--yaw', LaunchConfiguration('yaw'),
            '--frame-id', LaunchConfiguration('parent_frame'),
            '--child-frame-id', LaunchConfiguration('child_frame'),
        ],
    )
    return LaunchDescription(args + [stp])

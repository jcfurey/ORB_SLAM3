"""Open RViz2 with the ORB-SLAM3 config (TF, camera pose, path, map points).

The bundled config subscribes to the mono node's topics (/orb_slam3_mono/*).
For the rgbd/stereo nodes, change the topic on each display or pass your own
config with rviz_config:=/abs/your.rviz.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_rviz = PathJoinSubstitution([FindPackageShare('orb_slam3'), 'rviz', 'orb_slam3.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])

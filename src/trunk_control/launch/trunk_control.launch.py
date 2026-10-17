import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('trunk_control'), 'config', 'trunk_control.yaml')
    return LaunchDescription([
        Node(
            package='trunk_control',
            executable='trunk_control_node',
            name='trunk_control',
            output='screen',
            parameters=[config],
        ),
    ])

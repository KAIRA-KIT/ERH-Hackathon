import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from gripper_control import urdf
from gripper_control.geometry import Geometry


def generate_launch_description():
    trunk_config = os.path.join(
        get_package_share_directory('trunk_control'), 'config', 'trunk_control.yaml')
    gripper_config = os.path.join(
        get_package_share_directory('gripper_control'), 'config', 'gripper_control.yaml')
    description = urdf.generate(Geometry())
    return LaunchDescription([
        Node(
            package='trunk_control',
            executable='trunk_control_node',
            name='trunk_control',
            output='screen',
            parameters=[trunk_config],
        ),
        Node(
            package='gripper_control',
            executable='gripper_control_node',
            name='gripper_control',
            output='screen',
            parameters=[gripper_config],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(description, value_type=str)}],
        ),
    ])

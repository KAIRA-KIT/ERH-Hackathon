import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from gripper_control import urdf
from gripper_control.geometry import Geometry


def generate_launch_description():
    share = get_package_share_directory('gripper_control')
    description = urdf.generate(Geometry())
    return LaunchDescription([
        Node(
            package='gripper_control',
            executable='gripper_control_node',
            name='gripper_control',
            output='screen',
            parameters=[os.path.join(share, 'config', 'gripper_control.yaml'),
                        {'dry_run': True}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(description, value_type=str)}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'trunk.rviz')],
        ),
    ])

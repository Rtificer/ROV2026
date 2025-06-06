from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


import os

def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=[
                FindPackageShare('rov_control'),
                '/config/control_parameters.yaml'
            ],
            description='Path to the controller parameter file'
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=["/home/artificer/Desktop/ROV2026/src/rov_control/config/control_parameters.yaml"],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['thruster_pid_controller'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['axis_to_command_controller'],
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='rov_control',
            executable='gamepad_parser_node',
            name='gamepad_parser',
            parameters=[config_file],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['/opt/ros/jazzy/bin/xacro ', "/home/artificer/Desktop/ROV2026/src/description/urdf/ROV2026.urdf.xacro "]),
            }]
        ),
    ])
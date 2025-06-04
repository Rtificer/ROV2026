from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare('description'),
        'urdf',
        'ROV2026.urdf.xacro'
    ])
    urdf_path_str = str(urdf_path)

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                '/home/artificer/Desktop/ROV2026/src/rov_control/config/control_parameters.yaml',
                '/home/artificer/Desktop/ROV2026/src/description/urdf/ROV2026.urdf.xacro'
            ],
            output='screen'
        ),
        Node(
            package='rov_control',
            executable='gamepad_parser_node',
            parameters=[
                '/home/artificer/Desktop/ROV2026/src/rov_control/config/control_parameters.yaml'
            ],
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            parameters=[
                '/home/artificer/Desktop/ROV2026/src/rov_control/config/control_parameters.yaml'
            ],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    '/opt/ros/jazzy/bin/xacro ', # DO NOT DELETE THE SPACE FOR SOME REASON IT DOES NOT WORK WITHOUT IT
                    urdf_path
                ])
            }]
        ),
    ])
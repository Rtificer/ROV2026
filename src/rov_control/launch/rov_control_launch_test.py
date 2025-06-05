from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare('description'),
        'urdf',
        'ROV2026.urdf.xacro'
    ])
    
    param_path = PathJoinSubstitution([
        FindPackageShare('rov_control'),
        'config'
    ])
    

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                '/home/student/Desktop/ROV2026/src/rov_control/config/control_parameters.yaml'
            ],
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
            package='rov_control',
            executable='gamepad_parser_node',
            parameters=[
                PathJoinSubstitution([param_path, "gamepad_parameters.yaml"])
            ],
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            parameters=[
                PathJoinSubstitution([param_path, "joy_parameters.yaml"])
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
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ct6b_teleop')
    default_params_file = os.path.join(pkg_dir, 'config', 'ct6b_params.yaml')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the CT6B receiver (/dev/ttyUSB0, COM6, etc.)'
    )

    max_linear_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.5',
        description='Maximum linear velocity in m/s'
    )

    max_angular_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='1.0',
        description='Maximum angular velocity in rad/s'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic to publish Twist messages to'
    )

    invert_pitch_arg = DeclareLaunchArgument(
        'invert_pitch',
        default_value='true',
        description='Reverse pitch direction (stick UP -> forward)'
    )

    invert_roll_arg = DeclareLaunchArgument(
        'invert_roll',
        default_value='false',
        description='Reverse roll direction'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to ROS 2 parameters YAML file'
    )

    teleop_node = Node(
        package='ct6b_teleop',
        executable='ct6b_teleop_node',
        name='ct6b_teleop_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'invert_pitch': LaunchConfiguration('invert_pitch'),
                'invert_roll': LaunchConfiguration('invert_roll'),
            }
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        max_linear_arg,
        max_angular_arg,
        cmd_vel_topic_arg,
        invert_pitch_arg,
        invert_roll_arg,
        params_file_arg,
        teleop_node,
    ])

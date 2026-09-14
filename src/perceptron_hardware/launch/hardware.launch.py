import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_desc = get_package_share_directory('perceptron_robot_description')
    pkg_hw = get_package_share_directory('perceptron_hardware')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'perceptron_robot.xacro')
    default_config = os.path.join(pkg_hw, 'config', 'hardware_params.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to hardware params YAML'
    )

    port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='STM32 Serial Port'
    )

    # Robot State Publisher (is_sim:=false)
    # Quoted: Command() shlex-splits, so a workspace path with a space would
    # otherwise reach xacro as several arguments.
    robot_description = ParameterValue(
        Command(['xacro "', xacro_file, '" is_sim:=false']), value_type=str)

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # STM32 Hardware Bridge
    stm32_bridge = Node(
        package='perceptron_hardware',
        executable='stm32_bridge_node',
        name='stm32_bridge_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {'serial_port': LaunchConfiguration('serial_port')}
        ],
        output='screen'
    )

    battery = Node(
        package='perceptron_hardware',
        executable='battery_node',
        name='battery_node',
        parameters=[
            os.path.join(pkg_hw, 'config', 'battery_params.yaml'),
            {'use_sim_time': False},
        ],
        output='screen'
    )

    return LaunchDescription([
        config_arg,
        port_arg,
        robot_state_pub,
        stm32_bridge,
        battery
    ])

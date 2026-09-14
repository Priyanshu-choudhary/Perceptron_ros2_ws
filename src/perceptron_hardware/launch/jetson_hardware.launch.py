import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_desc = get_package_share_directory('perceptron_robot_description')
    pkg_hw = get_package_share_directory('perceptron_hardware')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'perceptron_robot.xacro')

    jetson_ip_arg = DeclareLaunchArgument(
        'jetson_ip',
        default_value='192.168.1.11',
        description='Jetson Nano Wi-Fi IP address'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='false',
        description='Publish odom->base_footprint TF directly (set false if using robot_localization EKF)'
    )

    # Robot State Publisher
    robot_description = ParameterValue(
        Command(['xacro "', xacro_file, '" is_sim:=false']), value_type=str)

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Jetson High-Speed Wi-Fi Bridge Node
    jetson_bridge = Node(
        package='perceptron_hardware',
        executable='jetson_bridge_node',
        name='jetson_bridge_node',
        parameters=[{
            'jetson_ip': LaunchConfiguration('jetson_ip'),
            'telemetry_port': 5555,
            'cmd_port': 5556,
            'laser_frame_id': 'laser_frame',
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'imu_frame_id': 'imu_link',
            'publish_tf': LaunchConfiguration('publish_tf'),
        }],
        output='screen'
    )

    # Battery Monitor
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
        jetson_ip_arg,
        publish_tf_arg,
        robot_state_pub,
        jetson_bridge,
        battery
    ])

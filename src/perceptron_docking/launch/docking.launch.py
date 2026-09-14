"""ArUco perception + docking state machine.

Hardware use (no Gazebo):
    ros2 launch perceptron_docking docking.launch.py \
        use_sim_time:=false cmd_vel_topic:=/cmd_vel
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('perceptron_docking')
    default_config = os.path.join(pkg_share, 'config', 'docking_params.yaml')

    args = [
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument(
            'auto_start',
            default_value='false',
            description='Begin searching for the marker as soon as the node starts',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/diff_drive_controller/cmd_vel_unstamped',
            description='Use /cmd_vel when driving the real STM32 base',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/diff_drive_controller/odom',
            description='Use /odom when driving the real STM32 base',
        ),
        # Every node in the sim must agree on the clock. Without this the
        # detector stamps poses with wall time while TF is on sim time, and
        # every lookup_transform fails with an extrapolation error.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    common_params = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    detector_node = Node(
        package='perceptron_docking',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        parameters=[LaunchConfiguration('config_file'), common_params],
        output='screen',
        emulate_tty=True,
    )

    controller_node = Node(
        package='perceptron_docking',
        executable='docking_controller_node',
        name='docking_controller_node',
        parameters=[
            LaunchConfiguration('config_file'),
            common_params,
            {
                'auto_start': LaunchConfiguration('auto_start'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
            },
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription(args + [detector_node, controller_node])

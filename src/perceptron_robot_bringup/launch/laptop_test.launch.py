"""Real lidar + real STM32 ECU, driven from the laptop instead of the Jetson.

    ros2 launch perceptron_robot_bringup laptop_test.launch.py

The robot is too big to drive around a desk, so this is the bench version of
the stack: both real sensors on the laptop's USB, everything else identical to
what would run on board. It gives you /scan, /odom, /imu/data_raw and the TF
tree, which is everything SLAM needs, without the motors being able to take the
robot anywhere.

Ports are detected at launch, not assumed. Both the lidar and the ECU are on
CP2102 adapters that report the same vendor, product AND serial ("0001"), so
udev cannot name them apart and ttyUSB numbering follows attach order. The
launch asks each port what it is by listening for a validly checksummed frame;
see perceptron_hardware/port_detect.py.

Arguments:
    lidar_port    override detection with an explicit device
    stm32_port    override detection with an explicit device
    rviz          open RViz. Default true.
    stm32         set false to bring up the lidar alone, which is the useful
                  thing when the ECU is unplugged or you are isolating a fault.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _resolve_ports(context):
    """Fill in whichever of the two ports the user did not pin explicitly."""
    lidar = LaunchConfiguration('lidar_port').perform(context)
    stm32 = LaunchConfiguration('stm32_port').perform(context)

    if lidar and stm32:
        return []

    try:
        from perceptron_hardware.port_detect import detect_ports
        found = detect_ports()
    except Exception as exc:  # detection must never be the reason nothing starts
        return [LogInfo(msg='port detection failed (%s); using /dev/ttyUSB0' % exc)]

    msgs = []
    if not lidar:
        lidar = found.get('lidar', '')
        msgs.append(LogInfo(msg='detected lidar on %s' % (lidar or 'NOTHING')))
    if not stm32:
        stm32 = found.get('stm32', '')
        msgs.append(LogInfo(msg='detected stm32 on %s' % (stm32 or 'NOTHING')))

    context.launch_configurations['lidar_port'] = lidar or '/dev/ttyUSB0'
    context.launch_configurations['stm32_port'] = stm32 or '/dev/ttyUSB1'
    return msgs


def generate_launch_description():
    pkg_desc = get_package_share_directory('perceptron_robot_description')
    pkg_hw = get_package_share_directory('perceptron_hardware')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'perceptron_robot.xacro')

    lidar_port = LaunchConfiguration('lidar_port')
    stm32_port = LaunchConfiguration('stm32_port')
    rviz = LaunchConfiguration('rviz')
    stm32 = LaunchConfiguration('stm32')

    args = [
        DeclareLaunchArgument('lidar_port', default_value=''),
        DeclareLaunchArgument('stm32_port', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('stm32', default_value='true'),
        OpaqueFunction(function=_resolve_ports),
    ]

    # The URDF supplies base_link -> laser_link, so unlike lidar_real.launch.py
    # there is no static_transform_publisher here. Running both would put two
    # publishers on one transform and TF would flap between them.
    robot_description = ParameterValue(
        Command(['xacro "', xacro_file, '" is_sim:=false']), value_type=str)

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}],
    )

    lidar = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ldlidar_node',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'topic_name': 'scan',
            'frame_id': 'laser_link',
            'port_name': lidar_port,
            'port_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'angle_crop_min': 135.0,
            'angle_crop_max': 225.0,
        }],
    )

    # publish_tf stays false: hardware_params.yaml hands odom -> base_footprint
    # to the EKF. On the bench with no EKF running there is simply no odom
    # frame, which is correct -- an unfused wheel-only odom would look right
    # and drift badly the moment the robot turned.
    stm32_bridge = Node(
        package='perceptron_hardware',
        executable='stm32_bridge_node',
        name='stm32_bridge_node',
        output='screen',
        condition=IfCondition(stm32),
        parameters=[
            os.path.join(pkg_hw, 'config', 'hardware_params.yaml'),
            {'serial_port': stm32_port, 'use_sim_time': False},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('perceptron_robot_bringup'), 'rviz', 'lidar_test.rviz'])],
    )

    return LaunchDescription(
        args + [robot_state_pub, lidar, stm32_bridge, rviz_node])

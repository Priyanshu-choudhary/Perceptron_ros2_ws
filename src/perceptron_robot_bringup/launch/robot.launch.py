"""Real lidar + real STM32 ECU via ZeroMQ Jetson LAN Bridge or direct USB Serial.

    ros2 launch perceptron_robot_bringup robot.launch.py
    ros2 launch perceptron_robot_bringup robot.launch.py use_jetson:=true jetson_ip:=192.168.1.7 
    ros2 launch perceptron_robot_bringup robot.launch.py use_jetson:=false (for local USB cables)
    ros2 launch perceptron_robot_bringup robot.launch.py jetson_ip:=192.168.1.7 ekf:=false  (lidar + wheels only)

This starts, in dependency order:
    robot_state_publisher   the URDF, so RViz has a model and TF has the joints
    joint_state_publisher   publishes wheel joint angles
    jetson_bridge_node      connects via ZeroMQ to Jetson Nano (streams /scan, /odom, /imu/data_raw)
    battery_node            /battery/state from measured voltage/current
    ekf_filter_node         fuses wheels + gyro, owns odom -> base_footprint
    slam_toolbox            builds the map, owns map -> odom
    rviz2                   model, scan, map, TF

Arguments:
    use_jetson   connect via ZeroMQ to Jetson Nano bridge over Wi-Fi. Default true.
    jetson_ip    IP address of the Jetson Nano. Default 192.168.1.7 .
    slam         run slam_toolbox. Default true. false gives sensors only.
    rviz         open RViz. Default true.
    ekf          run the EKF. Default true. false switches the IMU off COMPLETELY
                 as well: no EKF, no /imu/data_raw, no gyro conditioning, and
                 odom -> base_footprint comes straight from the wheel encoders,
                 so slam_toolbox builds the map from lidar + wheels alone.
    stm32        run the ECU bridge when use_jetson is false. Default true.
    lidar_port   override port detection when use_jetson is false.
    stm32_port   override port detection when use_jetson is false.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _resolve_ports(context):
    """Fill in whichever port the user did not pin explicitly when using local USB."""
    use_jetson = LaunchConfiguration('use_jetson').perform(context).lower() in ('true', '1')
    if use_jetson:
        return []

    lidar = LaunchConfiguration('lidar_port').perform(context)
    stm32 = LaunchConfiguration('stm32_port').perform(context)

    if lidar and stm32:
        return []

    try:
        from perceptron_hardware.port_detect import detect_ports
        found = detect_ports()
    except Exception as exc:  # never let detection be why nothing starts
        return [LogInfo(msg='port detection failed (%s); falling back' % exc)]

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
    pkg_bringup = get_package_share_directory('perceptron_robot_bringup')
    pkg_nav = get_package_share_directory('perceptron_navigation')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'perceptron_robot.xacro')

    use_jetson = LaunchConfiguration('use_jetson')
    jetson = LaunchConfiguration('jetson')
    jetson_ip = LaunchConfiguration('jetson_ip')
    resolved_jetson_ip = PythonExpression([
        "'", jetson, "'.strip() if '", jetson, "'.strip() != '' else '", jetson_ip, "'.strip()"
    ])
    lidar_port = LaunchConfiguration('lidar_port')
    stm32_port = LaunchConfiguration('stm32_port')
    rviz = LaunchConfiguration('rviz')
    slam = LaunchConfiguration('slam')
    ekf = LaunchConfiguration('ekf')
    stm32 = LaunchConfiguration('stm32')
    use_imu = LaunchConfiguration('use_imu')
    imu = LaunchConfiguration('imu')

    args = [
        DeclareLaunchArgument('use_jetson', default_value='true',
                              description='Connect via ZeroMQ to Jetson Nano over LAN'),
        DeclareLaunchArgument('jetson', default_value='',
                              description='Jetson Nano IP address (e.g. jetson:=192.168.1.7)'),
        DeclareLaunchArgument('jetson_ip',
                              default_value=os.environ.get('JETSON_IP', '192.168.1.7'),
                              description='IP address of Jetson Nano on local network'),
        DeclareLaunchArgument('lidar_port', default_value=''),
        DeclareLaunchArgument('stm32_port', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('ekf', default_value='true',
                              description='false = lidar + wheels only: no EKF and no IMU at all'),
        DeclareLaunchArgument('stm32', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true',
                              description='Enable IMU (set false to disable IMU and run encoders only)'),
        DeclareLaunchArgument('imu', default_value='true',
                              description='Alias for use_imu (set false to disable IMU and run encoders only)'),
        OpaqueFunction(function=_resolve_ports),
    ]


    # ekf:=false means lidar + wheels ONLY, so it switches the IMU off too.
    #
    # The EKF is the only thing on this path that reads the gyro: the bridge's
    # own odometry dead-reckons from the wheel-derived vx/wz and never looks at
    # the IMU. Leaving the IMU on without the EKF therefore changes nothing
    # about the map, while still streaming an unused topic and running the gyro
    # ZUPT and bias tracker for nobody -- and it made ekf:=false a half-switch
    # that looked like "IMU off" in RViz's topic list but was not.
    #
    # So IMU, EKF, and who owns odom -> base_footprint are now ONE decision:
    #   ekf:=true  with imu on       -> EKF fuses wheels + gyro, and owns the TF
    #   ekf:=false, or imu:=false    -> no IMU, no EKF, the bridge owns the TF
    enable_imu = PythonExpression([
        "'", ekf, "'.lower() in ('true', '1') and '",
        use_imu, "'.lower() in ('true', '1') and '",
        imu, "'.lower() in ('true', '1')"
    ])
    # Exactly one node may publish odom -> base_footprint. Two publishers make
    # tf2 interleave them and the robot visibly teleports between two poses.
    bridge_publish_tf = PythonExpression(["not ", enable_imu])
    run_ekf = enable_imu

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

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # ── Jetson ZeroMQ LAN Bridge Mode ──────────────────────────────────────────
    jetson_bridge = Node(
        package='perceptron_hardware',
        executable='jetson_bridge_node',
        name='jetson_bridge_node',
        output='screen',
        condition=IfCondition(use_jetson),
        parameters=[
            # gyro_params.yaml FIRST so the inline dict below can still override.
            # Without this the LAN-bridge path got no gyro bias, no scale and no
            # ZUPT, because hardware_params.yaml is only loaded for the USB path
            # below - and use_jetson defaults to true, so this is the path that
            # actually runs.
            os.path.join(pkg_hw, 'config', 'gyro_params.yaml'),
            {
                'jetson_ip': resolved_jetson_ip,
                'telemetry_port': 5555,
                'cmd_port': 5556,
                'laser_frame_id': 'laser_link',
                'base_frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'imu_frame_id': 'imu_link',
                'publish_tf': ParameterValue(bridge_publish_tf, value_type=bool),
                'auto_arm': True,
                'use_imu': ParameterValue(enable_imu, value_type=bool),
            },
        ],
    )

    # ── Direct USB Serial Mode (Fallback when use_jetson:=false) ──────────────
    lidar = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ldlidar_node',
        output='screen',
        condition=UnlessCondition(use_jetson),
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

    stm32_bridge = Node(
        package='perceptron_hardware',
        executable='stm32_bridge_node',
        name='stm32_bridge_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", stm32, "' == 'true' and '", use_jetson, "' != 'true'"])),
        parameters=[
            os.path.join(pkg_hw, 'config', 'gyro_params.yaml'),
            os.path.join(pkg_hw, 'config', 'hardware_params.yaml'),
            {
                'serial_port': stm32_port,
                'use_sim_time': False,
                'auto_arm': True,
                'publish_tf': ParameterValue(bridge_publish_tf, value_type=bool),
                'use_imu': ParameterValue(enable_imu, value_type=bool),
            },
        ],
    )

    # ── Shared Nodes (run in both modes) ───────────────────────────────────────
    battery = Node(
        package='perceptron_hardware',
        executable='battery_node',
        name='battery_node',
        output='screen',
        parameters=[os.path.join(pkg_hw, 'config', 'battery_real.yaml')],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(run_ekf),
        parameters=[os.path.join(pkg_bringup, 'config', 'ekf_real.yaml')],
    )

    # robot_localization cannot recover from a NaN: one corrupt sample ends the
    # run until someone restarts the node. This resets it in place instead.
    ekf_watchdog = Node(
        package='perceptron_hardware',
        executable='ekf_watchdog_node',
        name='ekf_watchdog',
        output='screen',
        condition=IfCondition(run_ekf),
        parameters=[{'use_sim_time': False}],
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(slam),
        parameters=[
            os.path.join(pkg_nav, 'config', 'slam_toolbox.yaml'),
            {
                'use_sim_time': False,
                'min_laser_range': 0.05,
                'max_laser_range': 10.0,
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': False}],
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('perceptron_robot_bringup'), 'rviz', 'robot.rviz'])],
    )

    return LaunchDescription(args + [
        robot_state_pub,
        joint_state_pub,
        jetson_bridge,
        lidar,
        stm32_bridge,
        battery,
        ekf_node,
        ekf_watchdog,
        slam_node,
        rviz_node,
    ])

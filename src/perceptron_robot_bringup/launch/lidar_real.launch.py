"""The physical LDROBOT D500 (STL-19P), not the simulated one.

    ros2 launch perceptron_robot_bringup lidar_real.launch.py

This is the hardware counterpart to the `lidar_type:=2d` scanner in Gazebo. It
publishes the same topic in the same frame, /scan in laser_link, so everything
downstream (slam_toolbox, the Nav2 costmaps) works against the real sensor
without a single parameter change. The only thing you must remember is
`use_sim_time:=false` everywhere, because there is no /clock now.

The D500 kit is an STL-19P, which speaks the LD19 protocol at 230400 baud, so
the driver is told product_name LDLiDAR_LD19.

Arguments:
    port          serial device. Defaults to /dev/ldlidar, the stable symlink
                  created by tools/setup_d500_wsl.sh. Pass /dev/ttyUSB0 if you
                  skipped the udev rule.
    rviz          open RViz with a scan-only view. Default true, since the
                  point of this launch file is usually "is it working".
    publish_tf    publish a static base_link -> laser_link transform. Default
                  true for a bare bench test; set false when the full robot
                  bringup is already running robot_state_publisher, otherwise
                  two nodes fight over the same transform.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Must match lidar_x / lidar_y / lidar_z in perceptron_robot.xacro. If you move
# the scanner on the chassis, change it there and mirror it here.
LIDAR_XYZ = ('0.0', '0.0', '0.1475')


def generate_launch_description():
    port = LaunchConfiguration('port')
    rviz = LaunchConfiguration('rviz')
    publish_tf = LaunchConfiguration('publish_tf')

    args = [
        DeclareLaunchArgument('port', default_value='/dev/ldlidar'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
    ]

    lidar = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ldlidar_node',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'topic_name': 'scan',
            # base_laser is the driver's own default; we override it so the
            # scan lands in the frame the URDF and slam_toolbox already expect.
            'frame_id': 'laser_link',
            'port_name': port,
            'port_baudrate': 230400,
            'laser_scan_dir': True,          # counter-clockwise, REP-103 sense
            'enable_angle_crop_func': False,  # set true to mask a mast or arm
            'angle_crop_min': 135.0,
            'angle_crop_max': 225.0,
        }],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_link',
        output='screen',
        condition=IfCondition(publish_tf),
        arguments=[
            '--x', LIDAR_XYZ[0], '--y', LIDAR_XYZ[1], '--z', LIDAR_XYZ[2],
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'laser_link',
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

    return LaunchDescription(args + [lidar, static_tf, rviz_node])

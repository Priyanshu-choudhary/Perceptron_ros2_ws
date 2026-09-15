"""Full autonomous navigation on a SAVED map. Real robot, one command.

    ros2 launch perceptron_robot_bringup nav.launch.py \\
        map:=$HOME/perceptron_test_ws/src/perceptron_navigation/maps/room2_map.yaml

WHAT STARTS, AND WHO OWNS WHAT

    localization.launch.py   The robot itself: robot_state_publisher,
                             joint_state_publisher, jetson_bridge_node,
                             battery_node, and the EKF that owns
                             odom -> base_footprint. Plus map_server and AMCL,
                             which own map -> odom. And RViz.

    navigation.launch.py     Nav2 proper: planner, controller, both costmaps,
                             recovery behaviours, the behaviour tree,
                             waypoint follower and velocity smoother.

Nothing is redefined here. Both children are the same files the mapping and
localisation workflows already use, so tuning one place fixes every workflow -
there is no fourth copy of the hardware bringup to drift out of sync.

navigation.launch.py is started with localization:=external. It can start AMCL
or slam_toolbox itself, but localization.launch.py has already started AMCL,
and exactly one node may publish map -> odom. Two publishers do not fail
loudly; they interleave, and the robot appears to teleport.

WHY THERE IS NO cmd_vel RELAY

    In Humble, nav2_bringup remaps the controller's `cmd_vel` to /cmd_vel_nav
    and the smoother's `cmd_vel_smoothed` back to /cmd_vel, so the end of the
    Nav2 chain is plain /cmd_vel - exactly what jetson_bridge_node subscribes
    to. The relay exists for simulation, where the base listens on
    /diff_drive_controller/cmd_vel_unstamped instead. Relaying /cmd_vel onto
    /cmd_vel would be a feedback loop, so it is switched off here.

    Verify the chain any time with:  ros2 topic info /cmd_vel --verbose

FIRST RUN

    AMCL does not seed itself on the real robot - localization.launch.py sets
    set_initial_pose false, because unlike simulation there is no ground truth
    for where you put the robot. So:

      1. Place the robot roughly where it is on the map with "2D Pose Estimate"
         in RViz. Roughly is enough; the scan matcher converges from there.
      2. Drive a metre or turn on the spot. The particle cloud should visibly
         tighten. If it spreads instead, the initial pose was wrong - redo it.
      3. Send a goal with "Nav2 Goal".

ARGUMENTS

    map          .yaml written by map_saver_cli. Default: room2_map.
    jetson_ip    address of the Nano. Default 192.168.1.11.
    rviz         open RViz. Default true.
    nav_profile  dwb (default) = NavFn + DWB. mppi = SmacPlanner2D + MPPI,
                 which handles skid-steer scrub better but costs more CPU.
    autostart    bring the Nav2 lifecycle nodes up automatically. Default true.
    use_jetson, ekf, stm32, lidar_port, stm32_port
                 passed straight through to localization.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_bringup = get_package_share_directory('perceptron_robot_bringup')
    pkg_nav = get_package_share_directory('perceptron_navigation')

    map_yaml = LaunchConfiguration('map')
    use_jetson = LaunchConfiguration('use_jetson')
    jetson_ip = LaunchConfiguration('jetson_ip')
    lidar_port = LaunchConfiguration('lidar_port')
    stm32_port = LaunchConfiguration('stm32_port')
    rviz = LaunchConfiguration('rviz')
    ekf = LaunchConfiguration('ekf')
    stm32 = LaunchConfiguration('stm32')
    nav_profile = LaunchConfiguration('nav_profile')
    autostart = LaunchConfiguration('autostart')

    args = [
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_nav, 'maps', 'room2_map.yaml'),
            description='path to the map .yaml'),
        DeclareLaunchArgument('use_jetson', default_value='true'),
        DeclareLaunchArgument('jetson_ip', default_value='192.168.1.11'),
        DeclareLaunchArgument('lidar_port', default_value=''),
        DeclareLaunchArgument('stm32_port', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('ekf', default_value='true'),
        DeclareLaunchArgument('stm32', default_value='true'),
        DeclareLaunchArgument('nav_profile', default_value='dwb',
                              description='dwb or mppi - selects the behaviour tree'),
        DeclareLaunchArgument('autostart', default_value='true'),
    ]

    # Robot + map_server + AMCL + EKF + RViz.
    robot_and_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'localization.launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_jetson': use_jetson,
            'jetson_ip': jetson_ip,
            'lidar_port': lidar_port,
            'stm32_port': stm32_port,
            'rviz': rviz,
            'ekf': ekf,
            'stm32': stm32,
        }.items(),
    )

    # Nav2. localization:=external because map -> odom is already owned above.
    # relay:=false because Nav2's chain already ends on /cmd_vel here.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'localization': 'external',
            'slam': 'false',
            'nav_profile': nav_profile,
            'autostart': autostart,
            'relay': 'false',
            'robot_cmd_vel_topic': '/cmd_vel',
            'nav_cmd_vel_topic': '/cmd_vel',
        }.items(),
    )

    return LaunchDescription(args + [
        LogInfo(msg=['Navigation bringup. Set the initial pose in RViz with '
                     '"2D Pose Estimate" before sending a goal.']),
        robot_and_localization,
        navigation,
    ])

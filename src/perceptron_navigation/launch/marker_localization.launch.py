"""Wall-marker absolute localisation, in its two modes.

    # survey a board into the map (do this first, once per board)
    ros2 launch perceptron_navigation marker_localization.launch.py \
        mode:=teach board_name:=wall_north ids:=0,1,2,3

    # use the surveyed boards to seed AMCL
    ros2 launch perceptron_navigation marker_localization.launch.py

Neither mode brings up the robot. Run this ALONGSIDE localization.launch.py,
which owns the lidar, the ECU, AMCL and the map. Both modes read ArUco corner
pixels straight off the Jetson bridge over ZMQ, so they do not need a camera
topic and do not care whether one exists.

TEACH MODE expects the robot to already be well localised -- it measures the
board against the robot's pose, so it inherits whatever error AMCL had.

LOCALISE MODE expects AMCL to be running and, on the real robot,
`set_initial_pose: false` in nav2_params.yaml. Left true, AMCL starts
confidently at the origin and spends its particle budget before this node ever
gets to correct it.

Arguments:
    mode         teach | localize.  Default localize.
    params       override the parameter file.
    jetson_ip    default 192.168.1.11, as everywhere else.
    marker_map   path to the surveyed board map (localize mode).
    board_name   which board is being surveyed (teach mode).
    ids          the four marker ids, top-left, top-right, bottom-left,
                 bottom-right (teach mode).
    output       where to write the survey (teach mode).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_nav = get_package_share_directory('perceptron_navigation')
    default_params = os.path.join(pkg_nav, 'config', 'aruco_localization.yaml')

    mode = LaunchConfiguration('mode')
    params = LaunchConfiguration('params')
    jetson_ip = LaunchConfiguration('jetson_ip')
    marker_map = LaunchConfiguration('marker_map')
    board_name = LaunchConfiguration('board_name')
    tile_size = LaunchConfiguration('tile_size')
    tile_spacing = LaunchConfiguration('tile_spacing')
    ids = LaunchConfiguration('ids')
    output = LaunchConfiguration('output')

    teaching = IfCondition(PythonExpression(["'", mode, "' == 'teach'"]))
    localising = UnlessCondition(PythonExpression(["'", mode, "' == 'teach'"]))

    args = [
        DeclareLaunchArgument('mode', default_value='localize',
                              choices=['teach', 'localize'],
                              description='survey a board, or use the surveyed ones'),
        DeclareLaunchArgument('params', default_value=default_params),
        DeclareLaunchArgument('jetson_ip', default_value='192.168.1.11'),
        DeclareLaunchArgument(
            'marker_map',
            default_value=os.path.expanduser(
                '~/perceptron_test_ws/config/marker_map.yaml'),
            description='surveyed board poses; written by teach mode'),
        DeclareLaunchArgument('board_name', default_value='wall_north'),
        DeclareLaunchArgument(
            'tile_size', default_value='0.06',
            description='side of the BLACK square in metres, not the printed tile'),
        DeclareLaunchArgument(
            'tile_spacing', default_value='0.08',
            description='CENTRE to CENTRE spacing in metres = tile_size + gap'),
        DeclareLaunchArgument(
            'ids', default_value='0,1,2,3',
            description='marker ids: top-left, top-right, bottom-left, bottom-right'),
        DeclareLaunchArgument(
            'output',
            default_value=os.path.expanduser(
                '~/perceptron_test_ws/config/marker_map.yaml')),
    ]

    localizer = Node(
        package='perceptron_navigation', executable='aruco_localizer_node',
        name='aruco_localizer_node', output='screen', condition=localising,
        parameters=[params, {
            'use_sim_time': False,
            'jetson_ip': jetson_ip,
            'marker_map_path': marker_map,
        }])

    teacher = Node(
        package='perceptron_navigation', executable='teach_marker_node',
        name='teach_marker_node', output='screen', condition=teaching,
        parameters=[params, {
            'use_sim_time': False,
            'jetson_ip': jetson_ip,
            'board_name': board_name,
            'ids': ids,
            'tile_size': ParameterValue(tile_size, value_type=float),
            'tile_spacing': ParameterValue(tile_spacing, value_type=float),
            'output_path': output,
        }])

    return LaunchDescription(args + [localizer, teacher])

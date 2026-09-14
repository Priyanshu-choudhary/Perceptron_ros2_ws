"""Exploration coordinator and all-ID detector; assumes SLAM and Nav2 exist."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('nav_profile', default_value='dwb'),
        DeclareLaunchArgument('search_marker_id', default_value='-1'),
        DeclareLaunchArgument('marker_dictionary', default_value='DICT_5X5_250'),
        DeclareLaunchArgument('marker_size', default_value='0.15'),
        DeclareLaunchArgument('exploration_auto_start', default_value='false'),
        DeclareLaunchArgument(
            'patrol', default_value='false',
            description='Run the patrol supervisor: explore, save the dock '
                        'when the marker is found, drive to it, dock, and '
                        'act on battery low/critical.'),
        DeclareLaunchArgument('dock_store_path',
                              default_value='~/.ros/perceptron_dock/dock.json'),
        DeclareLaunchArgument('exploration_output', default_value='~/.ros/perceptron_exploration'),
        DeclareLaunchArgument('exploration_config', default_value=PathJoinSubstitution([
            FindPackageShare('perceptron_navigation'), 'config', 'exploration.yaml'])),
    ]
    common = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'dictionary_name': LaunchConfiguration('marker_dictionary'),
        'marker_size': ParameterValue(LaunchConfiguration('marker_size'), value_type=float),
    }
    detector = Node(package='perceptron_navigation', executable='aruco_search_detector',
                    name='aruco_search_detector', output='screen',
                    parameters=[LaunchConfiguration('exploration_config'), common])
    coordinator = Node(package='perceptron_navigation', executable='exploration_node',
                       name='exploration_node', output='screen', parameters=[
                           LaunchConfiguration('exploration_config'), common, {
                               'target_marker_id': ParameterValue(
                                   LaunchConfiguration('search_marker_id'), value_type=int),
                               'nav_profile': LaunchConfiguration('nav_profile'),
                               'auto_start': ParameterValue(
                                   LaunchConfiguration('exploration_auto_start'), value_type=bool),
                               'output_directory': LaunchConfiguration('exploration_output'),
                           }])
    # The supervisor. It drives the explorer, Nav2 and the docking controller
    # through their own interfaces, so it is simply one more node alongside
    # them rather than a wrapper around any of them.
    patrol = Node(package='perceptron_navigation', executable='patrol_node',
                  name='patrol_node', output='screen', parameters=[
                      LaunchConfiguration('exploration_config'), common, {
                          'search_marker_id': ParameterValue(
                              LaunchConfiguration('search_marker_id'), value_type=int),
                          'dock_store_path': LaunchConfiguration('dock_store_path'),
                      }],
                  condition=IfCondition(LaunchConfiguration('patrol')))
    return LaunchDescription(args + [detector, coordinator, patrol])
